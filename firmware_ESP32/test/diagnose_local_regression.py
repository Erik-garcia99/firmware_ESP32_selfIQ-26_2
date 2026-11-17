"""Host-only diagnosis; no changes to the frozen local-pattern candidate.

ROI registration of labelled EMPTY frames is an oracle diagnostic, NOT a
deployable procedure. Ring registration does not use occupancy labels.
Translations are tested on high-pass texture, +/-24 pixels plus 1 refinement.
"""
import argparse
import ctypes as ct
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

from PIL import Image

from compare_shelf_algorithms import metrics, roi_pixels
from evaluate_local_pattern import HEADER, SOURCE, NativeLocal
from evaluate_vision_photos import ROOT, digest, load_image

PROBE = ROOT / 'test/experimental/registration_probe.c'


class RegistrationResult(ct.Structure):
    _fields_ = [('dx', ct.c_int), ('dy', ct.c_int), ('points', ct.c_int),
                ('unshifted_correlation', ct.c_double), ('best_correlation', ct.c_double)]


class RegistrationProbe:
    def __init__(self, directory):
        library = Path(directory) / 'registration.so'
        self.command = [os.environ.get('CC', 'cc'), '-std=c11', '-O2',
                        '-Wall', '-Wextra', '-Werror', '-pedantic',
                        '-shared', '-fPIC', str(PROBE), '-lm', '-o', str(library)]
        subprocess.run(self.command, check=True, capture_output=True, text=True)
        self.lib = ct.CDLL(str(library))
        self.lib.registration_probe.argtypes = [
            ct.POINTER(ct.c_uint8), ct.POINTER(ct.c_uint8),
            *([ct.c_int] * 7), ct.POINTER(RegistrationResult)]
        self.lib.registration_probe.restype = ct.c_int

    def measure(self, reference, frame, roi, ring):
        a, width, height = reference
        b, w, h = frame
        if (w, h) != (width, height) or len(a) != w * h or len(b) != w * h:
            raise ValueError('Frames incompatibles')
        rect = roi_pixels(roi, width, height)
        result = RegistrationResult()
        code = self.lib.registration_probe(
            (ct.c_uint8 * len(a)).from_buffer_copy(a),
            (ct.c_uint8 * len(b)).from_buffer_copy(b),
            width, height, rect.x, rect.y, rect.w, rect.h, ring, ct.byref(result))
        if code:
            raise ValueError(f'registration_probe={code}')
        return {name: getattr(result, name) for name, _ in result._fields_}


def translate(frame, dx, dy):
    pixels, width, height = frame
    im = Image.frombytes('L', (width, height), pixels)
    shifted = im.transform(im.size, Image.Transform.AFFINE, (1, 0, dx, 0, 1, dy),
                           resample=Image.Resampling.NEAREST)
    return shifted.tobytes(), width, height


def reference_guard(registration):
    """Post-hoc experimental policy, NOT a certified runtime quality gate.

    Reject only a large, strongly supported translation. A weak match is
    inconclusive, never permission to realign or assume the camera moved.
    """
    before = registration['unshifted_correlation']
    best = registration['best_correlation']
    shift = max(abs(registration['dx']), abs(registration['dy']))
    if (registration['points'] < 128 or not math.isfinite(before)
            or not math.isfinite(best) or not -1 <= before <= best <= 1.000001
            or best < 0.30):
        return 'inconclusive_do_not_autocorrect'
    if shift > 4:
        if best >= 0.45 and best - before >= 0.20:
            return 'misaligned_reference_pair_recalibration_required'
        return 'inconclusive_do_not_autocorrect'
    if before >= 0.30:
        return 'consistent_translation_ONLY_not_general_validity'
    return 'inconclusive_do_not_autocorrect'


def diagnose(path, local, probe):
    manifest = json.loads(path.read_text())
    index_path = path.parent / 'export_index.json'
    index = json.loads(index_path.read_text())
    by_name = {s['image']: s for s in index['samples']}
    hashes = {}

    def frame(name):
        image_path = path.parent / name
        sha = digest(image_path.read_bytes())
        if by_name[name]['sha256'] != sha:
            raise ValueError(f'Hash incorrecto: {name}')
        hashes[name] = sha
        return load_image(image_path)

    names = {
        kind: next(s['image'] for s in index['samples']
                   if s['snapshot_id'] == ref['image_id'])
        for kind, ref in manifest['references'].items()
    }
    refs = {kind: frame(name) for kind, name in names.items()}
    roi = manifest['roi_millionths']
    ctx = local.context(refs['empty'], roi)
    local.calibrate(ctx, [refs['empty']])
    full_pixels = local.measure(ctx, refs['full'])['occupied_pixels']
    full_registration = probe.measure(refs['empty'], refs['full'], roi, 1)
    pair_guard = reference_guard(full_registration)
    shifted_full = translate(refs['full'], full_registration['dx'], full_registration['dy'])
    aligned_full_pixels = local.measure(ctx, shifted_full)['occupied_pixels']
    rows = []
    for sample in manifest['samples']:
        if sample.get('exclude_reason'):
            continue
        current = frame(sample['image'])
        result = local.measure(ctx, current, full_pixels)
        registration = probe.measure(refs['empty'], current, roi, 1)
        aligned = local.measure(ctx, translate(
            current, registration['dx'], registration['dy']), aligned_full_pixels)
        row = {
            **sample, 'original_relative_pct': result['relative_pct'],
            'guarded_original_pct': (
                result['relative_pct'] if pair_guard ==
                'consistent_translation_ONLY_not_general_validity' else None),
            'original_area_pct': result['area_pct'],
            'ring_registration': registration,
            'aligned_ring_relative_pct': aligned['relative_pct'],
            'aligned_ring_area_pct': aligned['area_pct'],
        }
        if sample.get('expected_pct') == 0:
            oracle = probe.measure(refs['empty'], current, roi, 0)
            oracle_aligned = local.measure(ctx, translate(current, oracle['dx'], oracle['dy']))
            row['empty_ROI_registration_DIAGNOSTIC_ONLY'] = oracle
            row['empty_area_after_oracle_shift'] = oracle_aligned['area_pct']
        rows.append(row)
    sensitivity = []
    for dx, dy in [(0, 0), (1, 0), (2, 0), (4, 0), (8, 0), (0, 2), (0, 4), (4, 4)]:
        simulated = translate(refs['empty'], dx, dy)
        sensitivity.append({
            'synthetic_shift_px': [dx, dy],
            'area_pct': local.measure(ctx, simulated)['area_pct'],
        })
    return {
        'dataset_id': manifest['dataset_id'],
        'manifest_sha256': digest(path.read_bytes()),
        'index_sha256': digest(index_path.read_bytes()),
        'image_hashes': hashes, 'references': names,
        'full_registration_from_background_ring': full_registration,
        'reference_guard_EXPERIMENTAL': pair_guard,
        'original_full_pixels': full_pixels, 'aligned_full_pixels': aligned_full_pixels,
        'metrics': {key: metrics(rows, key) for key in
                    ('original_relative_pct', 'guarded_original_pct',
                     'aligned_ring_relative_pct')},
        'synthetic_sensitivity': sensitivity, 'rows': rows,
    }


def check_new_session(report_path, probe, active_manifest=None):
    """Replay guard on the already scored reservation, not new validation."""
    previous = json.loads(report_path.read_text())
    session = report_path.parent
    roi = previous['roi_millionths']

    def checked_frame(row):
        path = session / row['image']
        if digest(path.read_bytes()) != row['sha256']:
            raise ValueError(f"Hash incorrecto: {row['image']}")
        return load_image(path)

    reference = checked_frame(previous['calibration_samples'][0][0])
    active_reference = None
    active_evidence = None
    if active_manifest:
        active = json.loads(active_manifest.read_text())
        active_index = json.loads(
            (active_manifest.parent / 'export_index.json').read_text())
        active_frames, active_hashes = [], {}
        if active['roi_millionths'] != roi:
            raise ValueError('No comparar referencias activas con otra geometría')
        for kind in ('empty', 'full'):
            source = next(s for s in active_index['samples']
                          if s['snapshot_id'] == active['references'][kind]['image_id'])
            path = active_manifest.parent / source['image']
            sha = digest(path.read_bytes())
            if sha != source['sha256']:
                raise ValueError('Hash de referencia activa incorrecto')
            active_frames.append(load_image(path))
            active_hashes[kind] = sha
        active_reference = active_frames[0]
        active_pair = probe.measure(*active_frames, roi, 1)
        active_evidence = {
            'manifest_sha256': digest(active_manifest.read_bytes()),
            'image_hashes': active_hashes,
            'context': active['context'],
            'pair_registration': active_pair, 'pair_guard': reference_guard(active_pair),
        }
    pairs = []
    for sample in previous['calibration_samples'][1]:
        registration = probe.measure(reference, checked_frame(sample), roi, 1)
        pairs.append({'image': sample['image'], 'registration': registration,
                      'guard': reference_guard(registration)})
    rows = []
    for sample in previous['rows']:
        registration = probe.measure(reference, checked_frame(sample), roi, 1)
        guard = reference_guard(registration)
        accepted = guard == 'consistent_translation_ONLY_not_general_validity'
        row = {
            'image': sample['image'], 'sha256': sample['sha256'],
            'group': sample['group'], 'expected_pct': sample['expected_pct'],
            'original_candidate_pct': sample['local_relative_pct'],
            'guarded_candidate_pct': sample['local_relative_pct'] if accepted else None,
            'registration': registration, 'guard': guard,
        }
        if active_reference:
            against_active = probe.measure(active_reference, checked_frame(sample), roi, 1)
            row['against_old_active_reference'] = {
                'registration': against_active, 'guard': reference_guard(against_active),
            }
        rows.append(row)
    return {
        'mode': 'post_hoc_guard_replay_NOT_new_validation',
        'input_report_sha256': digest(report_path.read_bytes()),
        'reference_pairs': pairs, 'rows': rows,
        'old_active_reference_evidence': active_evidence,
        'metrics': {key: metrics(rows, key) for key in
                    ('original_candidate_pct', 'guarded_candidate_pct')},
        'warning': 'Policy observed after inspecting this set; no certified thresholds.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifests', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--session-report', type=Path)
    parser.add_argument('--active-manifest', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
        local, probe = NativeLocal(directory), RegistrationProbe(directory)
        report = {
            'mode': 'causal_probe_NOT_acceptance', 'integration_gate': {'approved': False},
            'warnings': [
                'Oracle ROI registration ONLY on known-empty images: diagnostic, not inference.',
                'Ring may contain moving shadows/objects; best shift is not guaranteed geometric truth.',
                'No rotation, homography, photometric gate or changes to frozen candidate.',
                'No threshold sweep. Diagnostic transforms are never written over source photos.',
                'Aligned outputs do not certify the cause or a deployable correction by themselves.',
                'Reference guard is a post-hoc diagnostic policy, NOT field-validated thresholds.',
            ],
            'sources_sha256': {
                str(p): digest(p.read_bytes()) for p in
                (SOURCE, HEADER, PROBE, Path(__file__).resolve())},
            'compile_commands': [local.command, probe.command],
            'datasets': [diagnose(p, local, probe) for p in args.manifests],
            'new_session_guard_replay': (
                check_new_session(args.session_report, probe, args.active_manifest)
                if args.session_report else None),
        }
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    for dataset in report['datasets']:
        print(dataset['dataset_id'])
        print('metrics', dataset['metrics'])
        print('full registration', dataset['full_registration_from_background_ring'])
        for row in dataset['rows']:
            if row.get('expected_pct') == 0:
                print('empty', row['image'], 'area', row['original_area_pct'],
                      'ring', row['ring_registration'],
                      'oracle', row['empty_ROI_registration_DIAGNOSTIC_ONLY'],
                      'oracle area', row['empty_area_after_oracle_shift'])
        print('sensitivity', dataset['synthetic_sensitivity'])


if __name__ == '__main__':
    main()
