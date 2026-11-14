"""Exploración host: core vigente vs C original de shelf_detect, sin IA.

No selecciona umbrales con etiquetas ni modifica firmware/referencias.
Las etiquetas nominales históricas NO son máscaras de superficie ocupada.
"""
import argparse
import ctypes as ct
import json
import os
from pathlib import Path
import statistics
import subprocess
import tempfile
import zlib

from evaluate_vision_photos import NativeCore, digest, load_image

ROOT = Path(__file__).resolve().parents[1]
CLAUDE = ROOT.parents[1] / 'docs_compartido/algoritmo_claude/firmware/esp32cam/components/shelf_detect'


class Roi(ct.Structure):
    _fields_ = [(name, ct.c_uint16) for name in ('x', 'y', 'w', 'h')]


class Result(ct.Structure):
    _fields_ = [('fill_pct', ct.c_uint8), ('state', ct.c_int),
                ('changed', ct.c_bool)]


def roi_pixels(roi, width, height):
    if (len(roi) != 4 or any(type(v) is not int or v < 0 for v in roi)
            or not roi[2] or not roi[3]
            or roi[0] + roi[2] > 1000000 or roi[1] + roi[3] > 1000000):
        raise ValueError('ROI inválida')
    x, y, w, h = roi
    left, top = x * width // 1000000, y * height // 1000000
    right = ((x + w) * width + 999999) // 1000000
    bottom = ((y + h) * height + 999999) // 1000000
    if right - left < 8 or bottom - top < 8:
        raise ValueError('ROI menor que una celda')
    return Roi(left, top, right - left, bottom - top)


class NativeShelf:
    """Compila el candidato original; esp_err.h se sustituye SOLO en temporal."""
    def __init__(self, directory):
        directory = Path(directory)
        (directory / 'esp_err.h').write_text(
            '#pragma once\n'
            'typedef int esp_err_t;\n#define ESP_OK 0\n'
            '#define ESP_ERR_INVALID_ARG 0x102\n'
            '#define ESP_ERR_INVALID_STATE 0x103\n'
            '#define ESP_ERR_INVALID_SIZE 0x104\n')
        self.command = [os.environ.get('CC', 'cc'), '-std=c11', '-O2',
                        '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
                        '-I', str(directory), '-I', str(CLAUDE / 'include'),
                        str(CLAUDE / 'shelf_detect.c'),
                        '-o', str(directory / 'shelf.so')]
        subprocess.run(self.command, check=True, capture_output=True, text=True)
        self.lib = ct.CDLL(str(directory / 'shelf.so'))
        signatures = {
            'shelf_create': ([ct.c_uint16, ct.c_uint16, ct.POINTER(Roi),
                              ct.c_size_t, ct.c_void_p], ct.c_void_p),
            'shelf_destroy': ([ct.c_void_p], None),
            'shelf_calib_begin': ([ct.c_void_p], None),
            'shelf_calib_add': ([ct.c_void_p, ct.POINTER(ct.c_uint8)], ct.c_int),
            'shelf_calib_end': ([ct.c_void_p], ct.c_int),
            'shelf_process': ([ct.c_void_p, ct.POINTER(ct.c_uint8),
                               ct.POINTER(Result)], ct.c_int),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = args, result
        self.shapes = {}

    def create(self, frame, roi):
        _, width, height = frame
        rect = roi_pixels(roi, width, height)
        ctx = self.lib.shelf_create(width, height, ct.byref(rect), 1, None)
        if not ctx:
            raise ValueError('No se pudo crear shelf_detect')
        self.shapes[ctx] = (width, height)
        return ctx

    def check_frame(self, ctx, frame):
        pixels, width, height = frame
        if (self.shapes.get(ctx) != (width, height)
                or len(pixels) != width * height):
            raise ValueError('Frame incompatible con el contexto shelf_detect')

    def destroy(self, ctx):
        self.lib.shelf_destroy(ctx)
        self.shapes.pop(ctx, None)

    def calibrate(self, ctx, frames):
        self.lib.shelf_calib_begin(ctx)
        for frame in frames:
            self.check_frame(ctx, frame)
            if self.lib.shelf_calib_add(ctx, NativeCore.buffer(frame[0])):
                raise ValueError('Error de calibración shelf_detect')
        if self.lib.shelf_calib_end(ctx):
            raise ValueError('Calibración vacía')

    def measure(self, ctx, frame):
        self.check_frame(ctx, frame)
        result = Result()
        if self.lib.shelf_process(ctx, NativeCore.buffer(frame[0]), ct.byref(result)):
            raise ValueError('Error shelf_process')
        # Solo porcentaje crudo: no evaluar debounce con fotografías espaciadas.
        return int(result.fill_pct)


def metrics(rows, key):
    eligible = [r for r in rows if r['expected_pct'] is not None]
    valid = [r for r in eligible if r[key] is not None]
    errors = [abs(r[key] - r['expected_pct']) for r in valid]
    levels = {}
    for level in sorted({r['expected_pct'] for r in valid}):
        values = [r[key] for r in valid if r['expected_pct'] == level]
        levels[str(level)] = {'n': len(values), 'median': statistics.median(values),
                              'min': min(values), 'max': max(values)}
    medians = [v['median'] for v in levels.values()]
    return {
        'labelled_n': len(eligible), 'valid_n': len(valid),
        'coverage': len(valid) / len(eligible) if eligible else None,
        'mae_nominal_points': statistics.mean(errors) if errors else None,
        'within_15_points_n': sum(e <= 15 for e in errors),
        # Rechazos cuentan como no acierto, nunca como error cero.
        'within_15_points_rate_all_labelled': (
            sum(e <= 15 for e in errors) / len(eligible) if eligible else None),
        'levels': levels,
        'level_medians_nondecreasing': (
            all(a <= b for a, b in zip(medians, medians[1:]))
            if len(medians) >= 2 else None),
    }


def compare(manifest_path, core, shelf):
    path = Path(manifest_path)
    manifest = json.loads(path.read_text())
    index_path = path.parent / 'export_index.json'
    index = json.loads(index_path.read_text())
    inventory = {s['image']: s for s in index['samples']}
    loaded, hashes = {}, {}

    def image(name):
        if name not in loaded:
            raw = (path.parent / name).read_bytes()
            sha = digest(raw)
            if sha != inventory[name]['sha256']:
                raise ValueError(f'Hash incorrecto: {name}')
            loaded[name] = load_image(path.parent / name)
            hashes[name] = sha
        return loaded[name]

    references, descriptors = {}, {}
    for kind in ('empty', 'full'):
        ref = manifest['references'][kind]
        candidates = [s['image'] for s in index['samples']
                      if s['snapshot_id'] == ref['image_id']]
        if len(candidates) != 1:
            raise ValueError(f'JPEG de referencia ambiguo/ausente: {kind}')
        references[kind] = candidates[0]
        descriptors[kind] = bytes.fromhex(ref['descriptor_hex'])
        if f'{zlib.crc32(descriptors[kind]):08x}' != ref['crc32']:
            raise ValueError(f'CRC incorrecto: {kind}')
        if len(descriptors[kind]) != 66 or descriptors[kind][:2] != b'\x01\x08':
            raise ValueError('Descriptor incompatible')
        if ref.get('context', manifest['context']) != manifest['context']:
            raise ValueError('Contexto de referencia distinto')

    empty = image(references['empty'])
    full = image(references['full'])
    if empty[1:] != full[1:]:
        raise ValueError('Dimensiones de referencias distintas')
    roi = manifest['roi_millionths']
    ctx = shelf.create(empty, roi)
    try:
        shelf.calibrate(ctx, [empty])
        full_area = shelf.measure(ctx, full)
        rows = []
        seen = {hashes[name] for name in references.values()}
        for sample in manifest['samples']:
            name = sample['image']
            if sample.get('exclude_reason') or name in references.values():
                continue
            frame = image(name)
            if hashes[name] in seen:
                raise ValueError(f'Imagen duplicada/referencia reutilizada: {name}')
            seen.add(hashes[name])
            if frame[1:] != empty[1:]:
                raise ValueError(f'Dimensiones distintas: {name}')
            expected = sample.get('expected_pct')
            if expected is not None and not manifest.get('labels_operator_confirmed'):
                raise ValueError('No evaluar precisión con etiquetas sin confirmar')
            if expected is not None and (
                    isinstance(expected, bool) or not isinstance(expected, (int, float))
                    or not 0 <= expected <= 100):
                raise ValueError('Etiqueta inválida')
            baseline = core.measure(frame, roi, descriptors['empty'],
                                    descriptors['full'], 1)
            area = shelf.measure(ctx, frame)
            rows.append({
                'image': name, 'sha256': hashes[name],
                'group': sample.get('group'), 'expected_pct': expected,
                'core_status': baseline['status'],
                'core_pct': baseline['occupancy_pct'],
                'shelf_area_pct': area,
                'shelf_relative_full_unclamped_pct': (
                    100 * area / full_area if full_area >= 4 else None),
                'shelf_relative_full_clipped': full_area >= 4 and area > full_area,
                # Variante explícita, sin optimización con etiquetas:
                # relativizar el área al área detectada en la foto full.
                'shelf_relative_full_pct': (
                    min(100, (100 * area + full_area // 2) // full_area)
                    if full_area >= 4 else None),
            })
    finally:
        shelf.destroy(ctx)
    rect = roi_pixels(roi, empty[1], empty[2])
    return {
        'dataset_id': manifest['dataset_id'], 'context': manifest['context'],
        'manifest': str(path.resolve()), 'manifest_sha256': digest(path.read_bytes()),
        'index_sha256': digest(index_path.read_bytes()),
        'reference_images': references, 'image_hashes': hashes,
        'shelf_reference_source': 'JPEG empty, NOT persisted grayscale descriptor',
        'shelf_calibration_frames': 1,
        'shelf_full_reference_area_pct': full_area,
        'shelf_relative_full_min_reference_area_pct': 4,
        'relative_full_clipped_n': sum(
            r['shelf_relative_full_clipped'] for r in rows),
        'roi_pixels': [rect.x, rect.y, rect.w, rect.h],
        'shelf_grid': [rect.w // 8, rect.h // 8],
        'shelf_ignored_right_bottom_pixels': [rect.w % 8, rect.h % 8],
        'label_method': manifest.get('label_method'),
        'labels_operator_confirmed': manifest.get('labels_operator_confirmed', False),
        'metrics': {key: metrics(rows, key) for key in
                    ('core_pct', 'shelf_area_pct', 'shelf_relative_full_pct')},
        'rows': rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifests', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='shelf_compare_',
                                     dir='/tmp/opencode') as directory:
        core, shelf = NativeCore(directory), NativeShelf(directory)
        report = {
            'schema_version': 1, 'mode': 'exploratory_no_hardware',
            'integration_gate': {'approved': False},
            'warnings': [
                'No IA, sin ajuste de parámetros usando etiquetas.',
                'Mismos lotes ya explorados: no validación independiente.',
                'Etiquetas históricas nominales, NO verdad de área visible.',
                'No convertir MAE en accuracy (100-MAE).',
                'Umbrales originales: diff=22, edge=6, sin compensación de luz.',
                'Una referencia JPEG vacía; sin falsa acumulación de copias.',
                'Sin métricas temporales, latencia ESP32 ni estados debounce.',
                'Sin evidencia de generalización a otros objetos/tamaños.',
            ],
            'sources_sha256': {
                str(p.relative_to(ROOT.parents[1])): digest(p.read_bytes())
                for p in [CLAUDE / 'shelf_detect.c',
                          CLAUDE / 'include/shelf_detect.h',
                          Path(__file__), ROOT / 'test/evaluate_vision_photos.py',
                          ROOT / 'test/vision_offline_bridge.c',
                          ROOT / 'main/modules/vision/vision_core.c',
                          ROOT / 'main/modules/vision/vision_core.h',
                          ROOT / 'main/modules/calibration/calibration_core.c',
                          ROOT / 'main/modules/calibration/calibration_core.h']
            },
            'compile_commands': [core.command, shelf.command],
            'datasets': [compare(p, core, shelf) for p in args.manifests],
        }
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    for dataset in report['datasets']:
        print(dataset['dataset_id'])
        for name, values in dataset['metrics'].items():
            print(name, json.dumps(values, ensure_ascii=False))


if __name__ == '__main__':
    main()
