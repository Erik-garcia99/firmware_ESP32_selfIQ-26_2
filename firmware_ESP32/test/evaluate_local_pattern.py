"""Candidato local C, solo host. Separación fija antes de medir el candidato.

--freeze-split crea una división sin ejecutar inferencias.
--partition development/holdout nunca mezcla calibración con evaluación.
El holdout interno NO es una sesión independiente ni un ensayo ciego: las
fotos ya fueron observadas durante el diagnóstico de los baselines.
"""
import argparse
import ctypes as ct
from datetime import datetime
import json
import os
from pathlib import Path
import statistics
import subprocess
import tempfile

from analyze_shelf_lighting import GROUPS, evaluate_scenario, inventory
from compare_shelf_algorithms import NativeShelf, metrics, roi_pixels
from evaluate_vision_photos import NativeCore, ROOT, SOURCES, digest, load_image

SOURCE = ROOT / 'main/modules/vision/local_pattern.c'
HEADER = SOURCE.with_suffix('.h')
U8 = ct.POINTER(ct.c_uint8)


class LocalResult(ct.Structure):
    _fields_ = [(name, ct.c_uint32) for name in (
        'occupied_pixels', 'total_pixels', 'raw_occupied_pixels',
        'weak_reference_pixels', 'relative_unclamped_q8'
    )] + [('area_pct', ct.c_uint8), ('relative_pct', ct.c_uint8)]


class NativeLocal:
    def __init__(self, directory):
        library = Path(directory) / 'local_pattern.so'
        self.command = [os.environ.get('CC', 'cc'), '-std=c11', '-O2',
                        '-Wall', '-Wextra', '-Werror', '-pedantic',
                        '-shared', '-fPIC', str(SOURCE), '-o', str(library)]
        subprocess.run(self.command, check=True, capture_output=True, text=True)
        self.lib = ct.CDLL(str(library))
        self.lib.lp_context_bytes.argtypes = []
        self.lib.lp_context_bytes.restype = ct.c_size_t
        self.bytes = self.lib.lp_context_bytes()
        self.lib.lp_init.argtypes = [ct.c_void_p] + [ct.c_uint32] * 6
        self.lib.lp_add_empty.argtypes = [ct.c_void_p, U8, ct.c_size_t, ct.c_uint32]
        self.lib.lp_finish_empty.argtypes = [ct.c_void_p]
        self.lib.lp_measure.argtypes = [
            ct.c_void_p, U8, ct.c_size_t, ct.c_uint32, ct.c_uint32,
            ct.POINTER(LocalResult)]
        for name in ('lp_init', 'lp_add_empty', 'lp_finish_empty', 'lp_measure'):
            getattr(self.lib, name).restype = ct.c_int

    def context(self, frame, roi):
        _, w, h = frame
        rect = roi_pixels(roi, w, h)
        ctx = ct.create_string_buffer(self.bytes)
        code = self.lib.lp_init(ctx, w, h, rect.x, rect.y, rect.w, rect.h)
        if code:
            raise ValueError(f'lp_init={code}')
        return ctx, w, h

    @staticmethod
    def frame_args(ctx, frame):
        pixels, w, h = frame
        if (w, h) != ctx[1:] or len(pixels) != w * h:
            raise ValueError('Frame incompatible')
        return ctx[0], NativeCore.buffer(pixels), len(pixels), w

    def calibrate(self, ctx, frames):
        for frame in frames:
            code = self.lib.lp_add_empty(*self.frame_args(ctx, frame))
            if code:
                raise ValueError(f'lp_add_empty={code}')
        code = self.lib.lp_finish_empty(ctx[0])
        if code:
            raise ValueError(f'lp_finish_empty={code}')

    def measure(self, ctx, frame, full_pixels=0):
        result = LocalResult()
        code = self.lib.lp_measure(
            *self.frame_args(ctx, frame), full_pixels, ct.byref(result))
        if code:
            raise ValueError(f'lp_measure={code}')
        values = {field: int(getattr(result, field)) for field, _ in result._fields_}
        values['relative_pct'] = (
            None if values['relative_pct'] == 255 else values['relative_pct'])
        return values


def freeze_split(samples, context_path):
    calibration = []
    for group in ('empty_normal', 'full_normal'):
        members = [s for s in samples if s['group'] == group]
        if len(members) < 5:
            raise ValueError('Se requieren >=5 normales por extremo')
        calibration.extend(s['snapshot_id'] for s in members[:3])
    rows = []
    for group in GROUPS:
        members = [s for s in samples if s['group'] == group
                   and s['snapshot_id'] not in calibration]
        if len(members) < 3:
            raise ValueError(f'Insuficientes muestras para dividir: {group}')
        holdout = {s['snapshot_id'] for s in members[-2:]}
        for sample in samples:
            if sample['group'] != group:
                continue
            partition = ('calibration' if sample['snapshot_id'] in calibration else
                         'holdout' if sample['snapshot_id'] in holdout else 'development')
            rows.append({**sample, 'partition': partition})
    return {
        'schema_version': 1,
        'rule': 'first3 normal endpoints calibration; last2/group internal holdout; rest development',
        'warning': 'Same-session correlated internal holdout, previously inspected for baselines',
        'labels': 'operator filenames, nominal half=50 NOT measured visible area',
        'context_manifest_sha256': digest(context_path.read_bytes()),
        'samples': rows,
    }


def local_summary(rows, key):
    return {
        **metrics(rows, key),
        'groups': {
            group: {
                **metrics([r for r in rows if r['group'] == group], key),
                'median': statistics.median(
                    r[key] for r in rows if r['group'] == group and r[key] is not None)
                if any(r['group'] == group and r[key] is not None for r in rows) else None,
            } for group in GROUPS
        },
    }


def evaluate(session, context_path, split_path, partition, directory):
    split = json.loads(split_path.read_text())
    if split['context_manifest_sha256'] != digest(context_path.read_bytes()):
        raise ValueError('Contexto cambió después de fijar la división')
    samples = split['samples']
    if len({s['snapshot_id'] for s in samples}) != len(samples):
        raise ValueError('División con duplicados')
    frames = {}
    for sample in samples:
        path = session / sample['image']
        if digest(path.read_bytes()) != sample['sha256']:
            raise ValueError(f"Foto cambió: {sample['image']}")
        frames[sample['image']] = load_image(path)
        if frames[sample['image']][1:] != (640, 480):
            raise ValueError('Fotografía no VGA')
    calibration = [[s for s in samples
                    if s['partition'] == 'calibration' and s['group'] == group]
                   for group in ('empty_normal', 'full_normal')]
    if any(len(group) != 3 for group in calibration):
        raise ValueError('División requiere tres referencias normales por extremo')
    selected = [s for s in samples if s['partition'] == partition]
    if not selected:
        raise ValueError('Partición vacía')
    roi = json.loads(context_path.read_text())['roi_millionths']
    core, shelf, local = NativeCore(directory), NativeShelf(directory), NativeLocal(directory)
    refs = [[frames[s['image']] for s in group] for group in calibration]
    descriptors = [core.median([core.descriptor(f, roi) for f in group]) for group in refs]
    baseline = evaluate_scenario('same_references_baselines', selected, set(),
                                 *refs, descriptors, roi, frames, core, shelf)
    ctx = local.context(refs[0][0], roi)
    local.calibrate(ctx, refs[0])
    full_counts = [local.measure(ctx, f)['occupied_pixels'] for f in refs[1]]
    full_pixels = int(statistics.median(full_counts))
    rows = []
    for sample in selected:
        result = local.measure(ctx, frames[sample['image']], full_pixels)
        rows.append({
            **sample, **result, 'local_area_pct': result['area_pct'],
            'local_relative_pct': result['relative_pct'],
            'relative_clipped': result['relative_unclamped_q8'] > 25600,
        })
    return {
        'schema_version': 1, 'mode': 'host_experimental_no_IA',
        'candidate_version': 1, 'partition': partition,
        'integration_gate': {'approved': False},
        'split_sha256': digest(split_path.read_bytes()),
        'context_manifest_sha256': digest(context_path.read_bytes()),
        'roi_millionths': roi, 'calibration_samples': calibration,
        'workspace_bytes': local.bytes,
        'full_reference_occupied_pixels': full_counts,
        'full_reference_median_pixels': full_pixels,
        'compile_commands': [core.command, shelf.command, local.command],
        'sources_sha256': {str(p): digest(p.read_bytes()) for p in (
            SOURCE, HEADER, Path(__file__).resolve(), *SOURCES,
            ROOT / 'test/analyze_shelf_lighting.py',
            ROOT / 'test/compare_shelf_algorithms.py',
            ROOT / 'test/evaluate_vision_photos.py')},
        'warnings': [
            'Not deployed; fixed local NCC .70, noise floor 5, tiles16, neighbor3.',
            'Internal correlated holdout, NOT independent field validation.',
            'Labels nominal, NOT area masks or object counting.',
            'ROI geometry assumed unchanged from user context, not verified per frame.',
            'No guards yet for camera movement, clipping or uninformative background.',
            'weak_reference_pixels exposes low texture; not certified quality/accuracy.',
            'No parameter sweep. Runtime neither learns nor identifies products.',
        ],
        'baseline_metrics': baseline['metrics'],
        'baseline_rows': baseline['rows'],
        'candidate_metrics': {
            key: local_summary(rows, key) for key in ('local_area_pct', 'local_relative_pct')
        },
        'relative_clipped_n': sum(r['relative_clipped'] for r in rows),
        'rows': rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--context-manifest', required=True, type=Path)
    parser.add_argument('--split', required=True, type=Path)
    parser.add_argument('--freeze-split', action='store_true')
    parser.add_argument('--partition', choices=('development', 'holdout'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.freeze_split:
        if args.split.exists():
            parser.error('No sobrescribir una división ya fijada')
        samples, _, _ = inventory(
            args.session, datetime.fromisoformat('2026-11-13T21:00:00+00:00'))
        report = freeze_split(samples, args.context_manifest)
        args.split.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print('Split fixed without inference:', {
            part: sum(s['partition'] == part for s in report['samples'])
            for part in ('calibration', 'development', 'holdout')})
        return
    if not args.partition or not args.output:
        parser.error('Se requieren --partition y --output')
    with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
        report = evaluate(args.session, args.context_manifest, args.split,
                          args.partition, directory)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    for method, result in report['candidate_metrics'].items():
        print(method, 'MAE', result['mae_nominal_points'],
              'within15', result['within_15_points_n'], '/', result['labelled_n'])
        for group, values in result['groups'].items():
            print(' ', group, values['median'])


if __name__ == '__main__':
    main()
