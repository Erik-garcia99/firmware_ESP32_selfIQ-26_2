"""Diagnóstico offline de fotos renombradas: luz, acomodo y referencias.

No ajusta umbrales. No infiere etiquetas de fotos sin nombre del operador.
"""
import argparse
from collections import defaultdict
from datetime import datetime
import json
from pathlib import Path
import statistics
import tempfile
import zlib

from compare_shelf_algorithms import CLAUDE, NativeShelf, metrics, roi_pixels
from evaluate_vision_photos import NativeCore, ROOT, SOURCES, digest, load_image

GROUPS = {
    'empty_normal': 0, 'empty_bright': 0,
    'full_normal': 100, 'full_bright': 100,
    'half_normal': 50, 'full_disordered': 100, 'half_disordered': 50,
}
METHODS = ('core_pct', 'shelf_area_pct', 'shelf_relative_full_pct')


def group_name(name):
    # El orden importa: full_nromal__luz es un subconjunto de full_nromal.
    prefixes = (
        ('ref_vacia_mas_luz', 'empty_bright'),
        ('ref_vacia', 'empty_normal'),
        ('ref_full_nromal__luz', 'full_bright'),
        ('ref_full_nromal', 'full_normal'),
        ('ref_mitad_normal', 'half_normal'),
        ('ref_full_desorden', 'full_disordered'),
        ('mitad_desorden', 'half_disordered'),
    )
    return next((group for prefix, group in prefixes if name.startswith(prefix)), None)


def inventory(session, since):
    """Reconecta alias del usuario con UUID/hora originales por SHA256."""
    index = json.loads((session / 'export_index.json').read_text())
    by_hash = defaultdict(list)
    for sample in index['samples']:
        by_hash[sample['sha256']].append(sample)
    aliases, rows, excluded = {}, [], []
    for path in sorted(session.iterdir()):
        if not path.is_file() or path.name == 'export_index.json':
            continue
        group = group_name(path.name)
        if path.suffix.lower() not in ('.jpg', '.jpeg') and group is None:
            continue
        sha = digest(path.read_bytes())
        matches = by_hash.get(sha, [])
        if len(matches) != 1:
            raise ValueError(f'Alias sin correspondencia única por hash: {path.name}')
        source = matches[0]
        aliases[source['snapshot_id']] = path
        if group is None:
            continue
        row = {'image': path.name, 'original_image': source['image'],
               'snapshot_id': source['snapshot_id'], 'sha256': sha,
               'captured_at': source['captured_at'], 'group': group,
               'expected_pct': GROUPS[group]}
        if datetime.fromisoformat(source['captured_at']) < since:
            excluded.append({**row, 'reason': 'historical_capture_not_this_session'})
        else:
            if any(r['sha256'] == sha for r in rows):
                raise ValueError('Muestra etiquetada duplicada')
            rows.append(row)
    rows.sort(key=lambda r: r['captured_at'])
    return rows, excluded, aliases


def summarize(rows):
    result = {}
    for method in METHODS:
        result[method] = metrics(rows, method)
        result[method]['groups'] = {}
        for group in GROUPS:
            subset = [r for r in rows if r['group'] == group]
            values = [r[method] for r in subset if r[method] is not None]
            result[method]['groups'][group] = {
                **metrics(subset, method),
                'output_median': statistics.median(values) if values else None,
                'output_min': min(values) if values else None,
                'output_max': max(values) if values else None,
            }
    return result


def evaluate_scenario(name, samples, calibration_ids, empty_frames, full_frames,
                      descriptors, roi, frames, core, shelf):
    ctx = shelf.create(empty_frames[0], roi)
    try:
        shelf.calibrate(ctx, empty_frames)
        full_areas = [shelf.measure(ctx, f) for f in full_frames]
        full_area = statistics.median(full_areas)
        rows = []
        for sample in samples:
            if sample['snapshot_id'] in calibration_ids:
                continue
            frame = frames[sample['image']]
            baseline = core.measure(frame, roi, *descriptors, 1)
            area = shelf.measure(ctx, frame)
            rows.append({
                **sample, 'core_status': baseline['status'],
                'core_pct': baseline['occupancy_pct'], 'shelf_area_pct': area,
                'shelf_relative_full_pct': (
                    min(100, int(100 * area / full_area + 0.5))
                    if full_area >= 4 else None),
                'shelf_relative_full_unclamped_pct': (
                    100 * area / full_area if full_area >= 4 else None),
                'shelf_relative_full_clipped': full_area >= 4 and area > full_area,
            })
    finally:
        shelf.destroy(ctx)
    return {
        'name': name, 'calibration_ids_excluded': sorted(calibration_ids),
        'empty_frames_n': len(empty_frames), 'full_frames_n': len(full_frames),
        'descriptor_crcs': [f'{zlib.crc32(d):08x}' for d in descriptors],
        'shelf_full_reference_areas_pct': full_areas,
        'shelf_full_reference_area_median_pct': full_area,
        'relative_full_clipped_n': sum(r['shelf_relative_full_clipped'] for r in rows),
        'metrics': summarize(rows), 'rows': rows,
    }


def analyze(session, context_path, since, directory):
    core, shelf = NativeCore(directory), NativeShelf(directory)
    manifest = json.loads(context_path.read_text())
    samples, excluded, aliases = inventory(session, since)
    if not samples:
        raise ValueError('Sin muestras nuevas etiquetadas')
    frames = {s['image']: load_image(session / s['image']) for s in samples}
    if any(f[1:] != (640, 480) for f in frames.values()):
        raise ValueError('Se requieren fotografías VGA sin reescalar')
    roi = manifest['roi_millionths']
    old_frames, descriptors, reference_evidence = [], [], []
    for kind in ('empty', 'full'):
        ref = manifest['references'][kind]
        frame_path = aliases[ref['image_id']]
        descriptor = bytes.fromhex(ref['descriptor_hex'])
        if (len(descriptor) != 66 or descriptor[:2] != b'\x01\x08'
                or f'{zlib.crc32(descriptor):08x}' != ref['crc32']):
            raise ValueError('Descriptor/CRC persistido inválido')
        if ref.get('context', manifest['context']) != manifest['context']:
            raise ValueError('Referencia de contexto distinto')
        frame = load_image(frame_path)
        if frame[1:] != (640, 480):
            raise ValueError('Referencia de dimensiones distintas')
        old_frames.append(frame)
        descriptors.append(descriptor)
        reference_evidence.append({
            'kind': kind, 'image': frame_path.name,
            'sha256': digest(frame_path.read_bytes()), 'image_id': ref['image_id'],
        })
    old = evaluate_scenario(
        'persisted_pair_no_recalibration', samples, set(),
        old_frames[:1], old_frames[1:], descriptors, roi, frames, core, shelf)

    # Regla fijada antes de medir resultados: tres primeras cronológicas normales.
    selected = [[s for s in samples if s['group'] == group][:3]
                for group in ('empty_normal', 'full_normal')]
    if any(len(group) != 3 for group in selected):
        raise ValueError('Faltan tres referencias normales de cada extremo')
    selected_frames = [[frames[s['image']] for s in group] for group in selected]
    median_descriptors = [
        core.median([core.descriptor(frame, roi) for frame in group])
        for group in selected_frames]
    new = evaluate_scenario(
        'new_host_calibration_first3_normal_NOT_persisted', samples,
        {s['snapshot_id'] for group in selected for s in group},
        *selected_frames, median_descriptors, roi, frames, core, shelf)
    rect = roi_pixels(roi, 640, 480)
    return {
        'schema_version': 1, 'mode': 'exploratory_frozen_parameters',
        'integration_gate': {'approved': False},
        'session': str(session.resolve()), 'since_inclusive': since.isoformat(),
        'labels_source': 'operator filenames; half=50 nominal, NOT area masks',
        'assumed_context_NOT_verified_per_frame': manifest['context'],
        'context_manifest_sha256': digest(context_path.read_bytes()),
        'index_sha256': digest((session / 'export_index.json').read_bytes()),
        'roi_millionths_assumed_unchanged': roi,
        'roi_pixels': [rect.x, rect.y, rect.w, rect.h],
        'new_labelled_n': len(samples), 'excluded_historical_aliases': excluded,
        'unlabelled_index_samples_not_scored': (
            len(json.loads((session / 'export_index.json').read_text())['samples'])
            - len(samples) - len(excluded)),
        'old_reference_evidence': reference_evidence,
        'new_host_reference_samples': selected,
        'warnings': [
            'Geometría config16 asumida: sin evidencia nueva de contexto aplicado.',
            'Etiquetas nominales; mitad de objetos no implica mitad de área.',
            'Un único montaje y tipo de producto; sin generalización demostrada.',
            'No se ajustaron umbrales ni se seleccionaron referencias por error.',
            'Recalibración host no persistida: core usa mediana C de descriptores.',
            'Shelf original usa promedio C de fondos JPEG, no mediana.',
            'La variante relativa a full puede saturar; se registra sin recorte.',
            'Muestras correlacionadas en una sesión; no ensayo independiente de campo.',
            'Sin compensación luminosa, IA, flash ni medición de latencia ESP32.',
        ],
        'compile_commands': [core.command, shelf.command],
        'sources_sha256': {
            str(p): digest(p.read_bytes()) for p in [
                Path(__file__).resolve(), ROOT / 'test/compare_shelf_algorithms.py',
                ROOT / 'test/evaluate_vision_photos.py', *SOURCES,
                ROOT / 'main/modules/vision/vision_core.h',
                ROOT / 'main/modules/calibration/calibration_core.h',
                CLAUDE / 'shelf_detect.c', CLAUDE / 'include/shelf_detect.h']
        },
        'scenarios': [old, new],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--context-manifest', required=True, type=Path)
    parser.add_argument('--since', default='2026-11-13T21:00:00+00:00')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    since = datetime.fromisoformat(args.since)
    if since.tzinfo is None:
        parser.error('--since requiere zona horaria')
    with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
        report = analyze(args.session, args.context_manifest, since, directory)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    for scenario in report['scenarios']:
        print(scenario['name'])
        for method, values in scenario['metrics'].items():
            print(method, 'MAE', values['mae_nominal_points'],
                  'within15', values['within_15_points_n'], '/', values['labelled_n'])
            for group, result in values['groups'].items():
                print(' ', group, result['output_median'],
                      (result['output_min'], result['output_max']),
                      'valid', result['valid_n'], '/', result['labelled_n'])


if __name__ == '__main__':
    main()
