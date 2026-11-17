"""Regresión exploratoria del candidato congelado sobre manifests anteriores."""
import argparse
import json
from pathlib import Path
import tempfile

from compare_shelf_algorithms import NativeShelf, compare, metrics
from evaluate_local_pattern import HEADER, SOURCE, NativeLocal
from evaluate_vision_photos import NativeCore, digest, load_image


def replay(path, core, shelf, local):
    manifest = json.loads(path.read_text())
    baseline = compare(path, core, shelf)
    refs = {kind: load_image(path.parent / name)
            for kind, name in baseline['reference_images'].items()}
    ctx = local.context(refs['empty'], manifest['roi_millionths'])
    local.calibrate(ctx, [refs['empty']])
    full = local.measure(ctx, refs['full'])['occupied_pixels']
    rows = []
    for row in baseline['rows']:
        result = local.measure(ctx, load_image(path.parent / row['image']), full)
        rows.append({
            **row, 'local_area_pct': result['area_pct'],
            'local_relative_pct': result['relative_pct'],
            'local_details': result,
        })
    return {
        'dataset_id': manifest['dataset_id'], 'roi_millionths': manifest['roi_millionths'],
        'reference_images': baseline['reference_images'],
        'image_hashes': baseline['image_hashes'],
        'manifest_sha256': digest(path.read_bytes()),
        'calibration_empty_frames_n': 1,
        'full_occupied_pixels': full,
        'baseline_metrics': baseline['metrics'],
        'candidate_metrics': {
            key: metrics(rows, key) for key in ('local_area_pct', 'local_relative_pct')
        },
        'rows': rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifests', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
        core, shelf, local = NativeCore(directory), NativeShelf(directory), NativeLocal(directory)
        report = {
            'mode': 'historical_regression_NOT_acceptance',
            'integration_gate': {'approved': False},
            'warnings': [
                'Already explored photos; NOT independent field validation.',
                'Single original empty JPEG per session, not new calibration.',
                'No changes to C candidate or parameters for these series.',
            ],
            'candidate_sources_sha256': {
                str(p): digest(p.read_bytes()) for p in
                (SOURCE, HEADER, Path(__file__).resolve())},
            'datasets': [replay(p, core, shelf, local) for p in args.manifests],
        }
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    for dataset in report['datasets']:
        print(dataset['dataset_id'])
        print(json.dumps(dataset['candidate_metrics'], ensure_ascii=False))


if __name__ == '__main__':
    main()
