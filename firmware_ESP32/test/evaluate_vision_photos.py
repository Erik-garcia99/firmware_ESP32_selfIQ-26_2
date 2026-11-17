"""Fase 5 E2: evalúa el C real, sin reimplementar descriptor/ocupación en Python.
CLI: python3 test/evaluate_vision_photos.py MANIFEST --output REPORT.json
Requiere compilador C11 y Pillow. No conecta con cloud ni hardware.
"""
import argparse
from collections import Counter, defaultdict
import ctypes as ct
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import tempfile
import time
import uuid
import zlib

from PIL import Image, __version__ as PILLOW_VERSION

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [ROOT / 'test/vision_offline_bridge.c',
           ROOT / 'main/modules/vision/vision_core.c',
           ROOT / 'main/modules/calibration/calibration_core.c']
U8 = ct.POINTER(ct.c_uint8)
U32 = ct.POINTER(ct.c_uint32)
LEVELS = (0, 25, 50, 75, 100)


def digest(data):
    return hashlib.sha256(data).hexdigest()


class NativeCore:
    def __init__(self, directory):
        library = Path(directory) / 'vision.so'
        self.command = [os.environ.get('CC', 'cc'), '-std=c11', '-O2', '-shared',
                        '-fPIC', '-Wall', '-Wextra', '-Werror', '-pedantic',
                        *map(str, SOURCES), '-o', str(library)]
        subprocess.run(self.command, check=True, capture_output=True, text=True)
        self.lib = ct.CDLL(str(library))
        self.lib.offline_descriptor.argtypes = [U8, ct.c_size_t, ct.c_uint32,
                                                ct.c_uint32, U32, U8]
        self.lib.offline_descriptor.restype = ct.c_int
        self.lib.offline_measure.argtypes = [U8, ct.c_size_t, ct.c_uint32,
                                             ct.c_uint32, U32, U8, U8, U32]
        self.lib.offline_measure.restype = ct.c_int
        self.lib.offline_median.argtypes = [U8, ct.c_size_t, U8]
        self.lib.offline_median.restype = ct.c_int
        self.lib.vision_status_code.argtypes = [ct.c_int]
        self.lib.vision_status_code.restype = ct.c_char_p
        self.lib.vision_algorithm_version.argtypes = []
        self.lib.vision_algorithm_version.restype = ct.c_uint32
        self.algorithm_version = int(self.lib.vision_algorithm_version())

    @staticmethod
    def buffer(data):
        return (ct.c_uint8 * len(data)).from_buffer_copy(data)

    def descriptor(self, frame, roi):
        pixels, width, height = frame
        result = (ct.c_uint8 * 66)()
        code = self.lib.offline_descriptor(self.buffer(pixels), len(pixels), width,
                                           height, (ct.c_uint32 * 4)(*roi), result)
        if code:
            raise ValueError(f'No se puede extraer referencia (cal_err={code})')
        return bytes(result)

    def median(self, descriptors):
        """Mediana por celda con el C real; 1..9 descriptores de 66 B."""
        if not 1 <= len(descriptors) <= 9:
            raise ValueError('La mediana admite entre 1 y 9 descriptores')
        if any(len(d) != 66 for d in descriptors):
            raise ValueError('Cada descriptor debe medir 66 bytes')
        blob = b''.join(descriptors)
        result = (ct.c_uint8 * 66)()
        code = self.lib.offline_median(self.buffer(blob), len(descriptors), result)
        if code:
            raise ValueError('No se puede combinar la referencia (mediana)')
        return bytes(result)

    def measure(self, frame, roi, empty, full, repeats):
        pixels, width, height = frame
        args = (self.buffer(pixels), len(pixels), width, height,
                (ct.c_uint32 * 4)(*roi), self.buffer(empty), self.buffer(full))
        values = (ct.c_uint32 * 9)()
        self.lib.offline_measure(*args, values)  # calentamiento fuera del tiempo
        elapsed = []
        previous = tuple(values)
        for _ in range(repeats):
            start = time.perf_counter_ns()
            status = self.lib.offline_measure(*args, values)
            elapsed.append((time.perf_counter_ns() - start) / 1_000_000)
            if status < 0 or tuple(values) != previous:
                raise ValueError('Resultado nativo inválido o no determinista')
        names = ('occupancy_pct', 'confidence', 'mean', 'contrast', 'clipped_pct',
                 'd_empty_q8', 'd_full_q8', 'separation_q8')
        result = dict(zip(names, values))
        result['status'] = self.lib.vision_status_code(status).decode('ascii')
        if status:
            result['occupancy_pct'] = None
            result['confidence'] = None
        result['host_core_ms'] = summary(elapsed)
        return result


def summary(values):
    if not values:
        return None
    return {'n': len(values), 'mean': statistics.mean(values),
            'median': statistics.median(values), 'min': min(values), 'max': max(values),
            'stdev': statistics.pstdev(values)}


def load_image(path):
    with Image.open(path) as im:
        if im.mode != 'L':
            raise ValueError(f'{path.name}: se requiere imagen gris L; no convertir RGB silenciosamente')
        im.load()
        return (im.tobytes(), im.width, im.height)


def valid_context(value):
    try:
        for key in ('roi_id', 'camera_id'):
            if str(uuid.UUID(value[key])) != value[key]:
                return False
        h = value['context_hash']
        return (isinstance(h, str) and len(h) == 64 and
                all(c in '0123456789abcdef' for c in h) and
                all(type(value[k]) is int and 0 < value[k] <= 0xffffffff
                    for k in ('config_version', 'calibration_version')))
    except (KeyError, ValueError, TypeError, AttributeError):
        return False


def evaluate(manifest_path, core, repeats=5, reference_override=None):
    path = Path(manifest_path).resolve()
    manifest_bytes = path.read_bytes()
    m = json.loads(manifest_bytes)
    mode = m.get('mode')
    if m.get('schema_version') != 1 or mode not in ('exploratory', 'acceptance'):
        raise ValueError('schema_version=1 y mode exploratory|acceptance obligatorios')
    if reference_override is not None:
        if mode != 'exploratory':
            raise ValueError('El barrido de referencias solo admite modo exploratorio')
        m['references'] = reference_override
    if type(repeats) is not int or not 1 <= repeats <= 1000:
        raise ValueError('repeats debe estar entre 1 y 1000')
    roi = m.get('roi_millionths')
    if (not isinstance(roi, list) or len(roi) != 4 or
            any(type(v) is not int or v < 0 or v > 1000000 for v in roi) or
            not roi[2] or not roi[3] or roi[0]+roi[2] > 1000000 or roi[1]+roi[3] > 1000000):
        raise ValueError('ROI inválida: cuatro enteros en millonésimas')
    context = m.get('context')
    if mode == 'acceptance' and not valid_context(context):
        raise ValueError('Aceptación requiere contexto real completo')
    used_hashes = set()
    refs = {}
    provenance = {}
    for kind in ('empty', 'full'):
        r = m['references'][kind]
        if mode == 'acceptance' and (r.get('context') != context or 'image' in r):
            raise ValueError('Aceptación requiere descriptores persistidos y contexto coincidente')
        if 'descriptor_hex' in r:
            data = bytes.fromhex(r['descriptor_hex'])
            crc = f'{zlib.crc32(data):08x}'
            if r.get('crc32') != crc:
                raise ValueError(f'CRC incorrecto: {kind}')
            provenance[kind] = {'source': 'persisted_descriptor', 'crc32': crc,
                                'sha256': digest(data)}
        else:
            image_path = path.parent / r['image']
            raw_hash = digest(image_path.read_bytes())
            used_hashes.add(raw_hash)
            data = core.descriptor(load_image(image_path), roi)
            provenance[kind] = {'source': 'reconstructed_from_image_NOT_active_pair',
                                'image': r['image'], 'image_sha256': raw_hash,
                                'crc32': f'{zlib.crc32(data):08x}', 'sha256': digest(data)}
        if len(data) != 66 or data[:2] != b'\x01\x08':
            raise ValueError('Solo se admiten descriptores v1 8x8 de 66 bytes')
        refs[kind] = data
    samples = m.get('samples')
    if not isinstance(samples, list) or not samples:
        raise ValueError('Dataset sin muestras')
    rows = []
    seen = set()
    for sample in samples:
        label = sample.get('expected_pct')
        if label is not None and (type(label) not in (int, float) or
                                  not math.isfinite(label) or not 0 <= label <= 100):
            raise ValueError('expected_pct debe ser finito 0..100 o null')
        if mode == 'acceptance' and sample.get('context') != context:
            raise ValueError('Contexto de muestra distinto al del dataset')
        image_path = path.parent / sample['image']
        image_hash = digest(image_path.read_bytes())
        if mode == 'acceptance' and sample.get('sha256') != image_hash:
            raise ValueError('Aceptación requiere SHA256 correcto por imagen')
        row = {'image': sample['image'], 'sha256': image_hash,
               'group': sample.get('group', str(label)), 'expected_pct': label}
        excluded = sample.get('exclude_reason')
        if image_hash in used_hashes:
            excluded = 'reference_image_not_independent'
        elif image_hash in seen:
            excluded = 'duplicate_image_not_independent'
        seen.add(image_hash)
        row['exclude_reason'] = excluded
        if not excluded:
            frame = load_image(image_path)
            if mode == 'acceptance' and frame[1:] != (640, 480):
                raise ValueError('Aceptación requiere VGA 640x480, sin reescalar')
            row['resolution'] = list(frame[1:])
            row.update(core.measure(frame, roi, refs['empty'], refs['full'], repeats))
            row['absolute_error_pct_points'] = (abs(row['occupancy_pct']-label)
                                                if label is not None and row['status'] == 'ok' else None)
        rows.append(row)
    grouped = defaultdict(list)
    for row in rows:
        grouped[row['group']].append(row)
    groups = {}
    for name, items in grouped.items():
        measured = [r for r in items if not r['exclude_reason']]
        valid = [r for r in measured if r['status'] == 'ok']
        groups[name] = {'total': len(items), 'evaluated': len(measured), 'valid': len(valid),
                        'rejections': dict(Counter(r['status'] for r in measured if r['status'] != 'ok')),
                        'occupancy_pct': summary([r['occupancy_pct'] for r in valid]),
                        'absolute_error_pct_points': summary([r['absolute_error_pct_points'] for r in valid
                                                              if r['absolute_error_pct_points'] is not None]),
                        'host_core_ms': summary([r['host_core_ms']['median'] for r in measured])}
    # Nunca aprobar integración automáticamente por un reporte offline.
    blockers = []
    if mode != 'acceptance':
        blockers.append('historical_or_exploratory_dataset_not_acceptance')
    for level in LEVELS:
        independent = [r for r in rows if r['expected_pct'] == level and not r['exclude_reason']]
        if len(independent) < 3:
            blockers.append(f'level_{level}_requires_3_independent_samples')
    if any(r.get('status') != 'ok' for r in rows if not r['exclude_reason']):
        blockers.append('core_rejections_require_review')
    if not m.get('labels_operator_confirmed', False):
        blockers.append('labels_not_operator_confirmed')
    blockers.append('manual_error_budget_and_physical_E0_gate_pending_review')
    by_level = {}
    for level in LEVELS:
        items = [r for r in rows if r['expected_pct'] == level and not r['exclude_reason']]
        valid = [r for r in items if r['status'] == 'ok']
        by_level[str(level)] = {
            'evaluated': len(items), 'valid': len(valid),
            'absolute_error_pct_points': summary([r['absolute_error_pct_points'] for r in valid]),
            'occupancy_pct': summary([r['occupancy_pct'] for r in valid]),
            'rejections': dict(Counter(r['status'] for r in items if r['status'] != 'ok'))}
    medians = [by_level[str(level)]['occupancy_pct'] for level in LEVELS]
    monotonic = (all(a['median'] <= b['median'] for a, b in zip(medians, medians[1:]))
                 if all(medians) else None)
    if monotonic is False:
        blockers.append('labelled_level_medians_not_monotonic')
    headers = [ROOT / 'main/modules/vision/vision_core.h',
               ROOT / 'main/modules/calibration/calibration_core.h']
    return {'schema_version': 1, 'dataset_id': m['dataset_id'], 'mode': mode,
            'algorithm_version': core.algorithm_version,
            'manifest_sha256': digest(manifest_bytes), 'roi_millionths': roi,
            'context_claimed_by_manifest': context, 'references': provenance,
            'source_sha256': {str(p.relative_to(ROOT)): digest(p.read_bytes()) for p in SOURCES + headers},
            'environment': {'platform': platform.platform(), 'python': platform.python_version(),
                            'pillow': PILLOW_VERSION, 'compiler': core.command[0],
                            'flags': '-std=c11 -O2 -shared -fPIC -Wall -Wextra -Werror -pedantic'},
            'timing_scope': 'host core + ctypes call only; excludes decode, I/O, capture, MQTT; not ESP latency',
            'repeats_per_image': repeats, 'groups': groups, 'by_level': by_level,
            'labelled_level_medians_nondecreasing': monotonic, 'samples': rows,
            'integration_gate': {'approved': False, 'blockers': blockers},
            'publication_candidates_NOT_approved': {'delta_pct': 3, 'max_silence_s': 60},
            'notes': m.get('notes', [])}


def sweep_references(manifest_path, core, empty_group, full_group):
    """Sensibilidad retrospectiva, nunca selección automática del mejor par."""
    m = json.loads(Path(manifest_path).read_text())
    if m.get('mode') != 'exploratory':
        raise ValueError('El barrido solo admite modo exploratory')
    candidates = [[s['image'] for s in m['samples']
                   if s.get('group') == group and not s.get('exclude_reason')]
                  for group in (empty_group, full_group)]
    if not all(candidates) or len(candidates[0])*len(candidates[1]) > 256:
        raise ValueError('Barrido vacío o superior a 256 parejas')
    results = []
    distributions = defaultdict(list)
    statuses = Counter()
    for empty in candidates[0]:
        for full in candidates[1]:
            report = evaluate(manifest_path, core, repeats=1,
                              reference_override={'empty': {'image': empty}, 'full': {'image': full}})
            medians = {}
            for name, group in report['groups'].items():
                if group['occupancy_pct']:
                    median = group['occupancy_pct']['median']
                    medians[name] = median
                    distributions[name].append(median)
            for row in report['samples']:
                if not row['exclude_reason']:
                    statuses[row['status']] += 1
            results.append({'empty_image': empty, 'full_image': full, 'group_medians': medians})
    return {'scope': 'reference sensitivity only; overlapping folds, NOT independent validation or parameter tuning',
            'pairs': len(results), 'status_counts_across_overlapping_folds': dict(statuses),
            'group_median_distribution_across_pairs': {k: summary(v) for k, v in distributions.items()},
            'pair_results': results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--reference-sweep', nargs=2, metavar=('EMPTY_GROUP', 'FULL_GROUP'),
                        help='Barrido exploratorio de parejas por grupos (máximo 256)')
    args = parser.parse_args()
    try:
        with tempfile.TemporaryDirectory(prefix='selfiq-e2-', dir=os.environ.get('TMPDIR', '/tmp/opencode')) as tmp:
            core = NativeCore(tmp)
            report = evaluate(args.manifest, core, args.repeats)
            if args.reference_sweep:
                report['reference_sensitivity'] = sweep_references(args.manifest, core, *args.reference_sweep)
        if args.output.resolve() == args.manifest.resolve():
            raise ValueError('El reporte no puede sobrescribir el manifest')
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f'Evaluación abortada: {exc}\n')
    print(f"Reporte: {args.output}; puerta de integración: NO aprobada")


if __name__ == '__main__':
    main()
