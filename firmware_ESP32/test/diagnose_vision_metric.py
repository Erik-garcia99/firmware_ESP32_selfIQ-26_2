"""Fase 5 E2: diagnóstico de métrica, SOLO HOST y NO desplegado.

Compara el cálculo actual (visión 2: ganancia 32/max(MAD,16) por descriptor)
con un candidato de diagnóstico (solo centrado por media, ganancia común 1)
sobre los mismos manifests, y mide estabilidad ante brillo, contraste y ruido.

No modifica el core, NVS, calibración ni firmware. No sustituye al evaluador:
el evaluador sigue siendo la referencia para aceptación. Aquí la variante
candidata se calcula en Python a partir de descriptores extraídos por el C real;
solo se usa para decidir si merece la pena implementarla en C y probarla.

CLI:
  python3 test/diagnose_vision_metric.py MANIFEST [MANIFEST ...] --output REPORT.json
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import statistics
import tempfile

from PIL import Image, ImageChops, ImageEnhance, __version__ as PILLOW_VERSION

import evaluate_vision_photos as ev

CELLS = 64
MIN_CONTRAST = 16
MAX_MAD = 64
MAX_CLIPPED_PCT = 20
MAX_MEAN_DRIFT = 24
MIN_SEPARATION = 4.0  # grises, candidato centrado; el core usa Q8 >= 4*256
NOISE_SEED = 20261113


def centered(cells):
    mean = sum(cells) / CELLS
    return [value - mean for value in cells]


def mae(a, b):
    return sum(abs(x - y) for x, y in zip(a, b)) / CELLS


def centered_percentage(cells, empty, full):
    """Ocupación por cercanía relativa SIN ganancia por descriptor."""
    de = mae(centered(cells), centered(empty))
    df = mae(centered(cells), centered(full))
    if de + df <= 0:
        return None
    pct = 100.0 * de / (de + df)
    return max(0.0, min(100.0, pct))


def descriptor_stats(cells):
    mean = (sum(cells) + CELLS // 2) // CELLS
    contrast = max(cells) - min(cells)
    mad = sum(abs(value - mean) for value in cells) / CELLS
    return mean, contrast, mad


def reference_gate(empty, full):
    """Puertas del candidato sobre las referencias (las del core, sin ganancia)."""
    reasons = []
    for name, cells in (('empty', empty), ('full', full)):
        _, contrast, mad = descriptor_stats(cells)
        if contrast < MIN_CONTRAST:
            reasons.append(f'{name}_low_contrast')
        if mad > MAX_MAD:
            reasons.append(f'{name}_high_mad')
    separation = mae(centered(empty), centered(full))
    if separation < MIN_SEPARATION:
        reasons.append('references_too_close')
    return {'separation_gray': separation, 'reasons': reasons}


def frame_gate(frame_result, ref_means):
    """Mismas puertas del core que no dependen de la métrica de distancia."""
    if frame_result['status'] != 'ok':
        return frame_result['status']
    low, high = min(ref_means), max(ref_means)
    if not low - MAX_MEAN_DRIFT <= frame_result['mean'] <= high + MAX_MEAN_DRIFT:
        return 'normalization_rejected'
    if frame_result['clipped_pct'] > MAX_CLIPPED_PCT:
        return 'excessive_clipping'
    return None


def summary(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {'n': len(values), 'mean': statistics.mean(values),
            'median': statistics.median(values), 'min': min(values),
            'max': max(values), 'stdev': statistics.pstdev(values)}


def level_table(rows, key):
    levels = defaultdict(list)
    for row in rows:
        if row['expected_pct'] is not None and row.get(key) is not None:
            levels[row['expected_pct']].append(row[key])
    table = {}
    for level in sorted(levels):
        table[str(level)] = summary(levels[level])
    medians = [table[str(l)]['median'] for l in ev.LEVELS if str(l) in table]
    monotonic = (all(a <= b for a, b in zip(medians, medians[1:]))
                 if len(medians) == len(ev.LEVELS) else None)
    return table, monotonic


def error_table(rows, key):
    levels = defaultdict(list)
    for row in rows:
        if row['expected_pct'] is not None and row.get(key) is not None:
            levels[row['expected_pct']].append(abs(row[key] - row['expected_pct']))
    table = {str(l): summary(levels[l]) for l in sorted(levels)}
    errors = [e for values in levels.values() for e in values]
    return table, summary(errors)


def noise_image(size, sigma, rng):
    pixels = bytes(max(0, min(255, int(round(rng.gauss(128, sigma)))))
                   for _ in range(size[0] * size[1]))
    return Image.frombytes('L', size, pixels)


def median_reference_diagnostic(path, roi, core, items):
    """Prueba de hipótesis con referencias derivadas del lote. FUGA DE DATOS.

    No es aceptación ni una pareja calibrada: las referencias salen de fotos
    etiquetadas de la misma sesión. Solo mide si la inestabilidad de una
    referencia de un único frame explica el sesgo observado.
    items: pares (fila, descriptor de 66 B) ya evaluados con el core.
    La mediana la calcula el C real (cal_descriptor_median) vía el puente.
    """
    by_level = defaultdict(list)
    for row, descriptor in items:
        by_level[row['expected_pct']].append((row, descriptor))
    if not by_level.get(0) or not by_level.get(100):
        return {'available': False, 'reason': 'faltan grupos etiquetados 0/100'}
    empty_all = core.median([d for _, d in by_level[0]])
    full_all = core.median([d for _, d in by_level[100]])
    gate = reference_gate(list(empty_all[2:]), list(full_all[2:]))
    if gate['reasons']:
        return {'available': False, 'reason': 'referencias medianas rechazadas',
                'reference_gate': gate}
    measured = defaultdict(list)
    rejections = Counter()
    for level, group in by_level.items():
        for row, descriptor in group:
            if level == 0:
                others = [d for _, d in by_level[0] if d is not descriptor]
                empty = core.median(others) if others else empty_all
                full = full_all
            elif level == 100:
                others = [d for _, d in by_level[100] if d is not descriptor]
                empty = empty_all
                full = core.median(others) if others else full_all
            else:
                empty, full = empty_all, full_all
            pct_centered = centered_percentage(list(descriptor[2:]),
                                               list(empty[2:]), list(full[2:]))
            frame = ev.load_image(path.parent / row['image'])
            result = core.measure(frame, roi, empty, full, 1)
            if result['status'] != 'ok':
                rejections[result['status']] += 1
                continue
            for variant, pct in (('core_v2', float(result['occupancy_pct'])),
                                 ('centered', pct_centered)):
                if pct is not None:
                    measured[variant].append((level, pct))
    variants = {}
    for variant, values in measured.items():
        levels = defaultdict(list)
        for level, pct in values:
            levels[level].append(pct)
        variants[variant] = {
            'by_level': {str(l): summary(v) for l, v in sorted(levels.items())},
            'error_by_level': {str(l): summary([abs(p - l) for p in v])
                               for l, v in sorted(levels.items())},
            'mae_points': summary([abs(p - l) for l, p in values]),
            'medians_nondecreasing': all(
                levels[ev.LEVELS[i]] and levels[ev.LEVELS[i + 1]] and
                statistics.median(levels[ev.LEVELS[i]]) <=
                statistics.median(levels[ev.LEVELS[i + 1]])
                for i in range(len(ev.LEVELS) - 1)) if all(levels.get(l) for l in ev.LEVELS) else None,
            'rejections': dict(rejections)}
    return {'available': True, 'leakage': True,
            'note': 'Referencias = mediana de los grupos etiquetados 0/100 de la misma '
                    'sesión, con leave-one-out en los extremos. Incluye fuga de datos: '
                    'no es una pareja calibrada ni validación independiente.',
            'reference_gate': gate, 'variants': variants}



def perturb(image, case, noise):
    if case == 'brightness_plus15':
        return ImageChops.add(image, Image.new('L', image.size, 15))
    if case == 'brightness_minus15':
        return ImageChops.subtract(image, Image.new('L', image.size, 15))
    if case == 'contrast_1.15':
        return ImageEnhance.Contrast(image).enhance(1.15)
    if case == 'contrast_0.85':
        return ImageEnhance.Contrast(image).enhance(0.85)
    if case == 'noise_sigma4':
        return ImageChops.add(image, noise[4], scale=1.0, offset=-128)
    if case == 'noise_sigma8':
        return ImageChops.add(image, noise[8], scale=1.0, offset=-128)
    raise ValueError(case)


CASES = ('brightness_plus15', 'brightness_minus15', 'contrast_1.15',
         'contrast_0.85', 'noise_sigma4', 'noise_sigma8')


def evaluate_dataset(manifest_path, core, tmpdir):
    path = Path(manifest_path).resolve()
    manifest = json.loads(path.read_text())
    roi = manifest['roi_millionths']
    refs = {kind: bytes.fromhex(manifest['references'][kind]['descriptor_hex'])
            for kind in ('empty', 'full')}
    empty_cells = list(refs['empty'][2:])
    full_cells = list(refs['full'][2:])
    gate = reference_gate(empty_cells, full_cells)
    ref_means = [descriptor_stats(cells)[0] for cells in (empty_cells, full_cells)]

    # Baseline con el core real: fija el subconjunto y los rechazos.
    baseline = ev.evaluate(path, core, repeats=1)
    rows, skipped, items = [], [], []
    for sample in baseline['samples']:
        if sample['exclude_reason']:
            continue
        row = {'image': sample['image'], 'expected_pct': sample['expected_pct'],
               'status': sample['status']}
        if sample['status'] == 'ok':
            frame = ev.load_image(path.parent / sample['image'])
            descriptor = core.descriptor(frame, roi)
            cells = list(descriptor[2:])
            row['core_v2'] = float(sample['occupancy_pct'])
            row['centered'] = centered_percentage(cells, empty_cells, full_cells)
            row['gate'] = frame_gate(sample, ref_means)
            rows.append(row)
            items.append((row, descriptor))
        else:
            row['core_v2'] = None
            row['centered'] = None
            skipped.append(row)
    if gate['reasons']:
        return {'dataset': manifest['dataset_id'], 'manifest': str(path),
                'manifest_sha256': ev.digest(path.read_bytes()),
                'references_rejected': gate, 'rows': rows, 'skipped': skipped,
                'candidate_rejects_all': True}

    controls = defaultdict(lambda: defaultdict(list))
    control_rejections = Counter()
    stable = [row for row in rows if row['status'] == 'ok']
    noise = {}
    rng = random.Random(NOISE_SEED)
    for sigma in (4, 8):
        noise[sigma] = noise_image((640, 480), sigma, rng)
    for row in stable:
        image = Image.open(path.parent / row['image']).convert('L')
        for case in CASES:
            changed = perturb(image, case, noise)
            frame = (changed.tobytes(), changed.width, changed.height)
            result = core.measure(frame, roi, refs['empty'], refs['full'], 1)
            status = frame_gate(result, ref_means)
            cells = list(core.descriptor(frame, roi)[2:])
            centered_pct = centered_percentage(cells, empty_cells, full_cells)
            if status is not None:
                control_rejections[case] += 1
                continue
            controls[case]['core_v2'].append(abs(result['occupancy_pct'] - row['core_v2']))
            controls[case]['centered'].append(abs(centered_pct - row['centered']))
    controls_report = {}
    for case in CASES:
        controls_report[case] = {
            variant: summary(controls[case][variant]) for variant in ('core_v2', 'centered')}
        controls_report[case]['rejections'] = control_rejections[case]

    report = {'dataset': manifest['dataset_id'], 'manifest': str(path),
              'manifest_sha256': ev.digest(path.read_bytes()),
              'references': gate, 'rows': rows, 'skipped': skipped,
              'candidate_rejects_all': False,
              'samples': len(rows) + len(skipped),
              'variants': {}, 'synthetic_controls': controls_report}
    for variant in ('core_v2', 'centered'):
        levels, monotonic = level_table(rows, variant)
        errors, overall = error_table(rows, variant)
        valid = [row[variant] for row in rows if row[variant] is not None]
        report['variants'][variant] = {
            'valid': len(valid), 'rejections': dict(
                Counter(row['status'] for row in skipped)),
            'by_level': levels, 'error_by_level': errors,
            'mae_points': overall, 'medians_nondecreasing': monotonic}
    report['median_reference_leaky_diagnostic'] = median_reference_diagnostic(
        path, roi, core, items)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifests', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        with tempfile.TemporaryDirectory(prefix='selfiq-metric-',
                                         dir=os.environ.get('TMPDIR', '/tmp/opencode')) as tmp:
            core = ev.NativeCore(tmp)
            datasets = [evaluate_dataset(path, core, tmp) for path in args.manifests]
            sources = [*ev.SOURCES,
                       ev.ROOT / 'main/modules/vision/vision_core.h',
                       ev.ROOT / 'main/modules/calibration/calibration_core.h']
            report = {
                'schema_version': 1,
                'kind': 'metric_diagnostic_host_only_not_deployed',
                'algorithm_version_in_core': core.algorithm_version,
                'core_source_sha256': {str(p.relative_to(ev.ROOT)): ev.digest(p.read_bytes())
                                       for p in sources},
                'candidate_definition': {
                    'centered': 'MAE(descriptor - media entera) con ganancia comun 1; '
                                'mismas puertas (contraste, MAD, saturacion, deriva de '
                                'media, separacion >=4 grises). Sin implementar en C.'},
                'environment': {'platform': platform.platform(),
                                'python': platform.python_version(),
                                'pillow': PILLOW_VERSION, 'compiler': core.command[0],
                                'noise_seed': NOISE_SEED},
                'caveats': [
                    'Comparacion sobre las mismas fotos y etiquetas, no validacion independiente.',
                    'La variante centrada se calcula en Python; el core no se modifico.',
                    'Falta reproducir el candidato en C con Q8 y sus propias regresiones.',
                    'Las etiquetas son nominales del operador, no area medida.',
                    'Los tiempos y el ruido sintetico no representan al ESP32 en operacion.'],
                'datasets': datasets,
            }
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False,
                                          allow_nan=False) + '\n')
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(2, f'Diagnóstico abortado: {exc}\n')
    print(f'Diagnóstico: {args.output}')


if __name__ == '__main__':
    main()
