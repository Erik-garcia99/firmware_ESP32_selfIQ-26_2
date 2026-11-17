import ctypes as ct
from pathlib import Path
import tempfile
import unittest

from evaluate_local_pattern import NativeLocal, freeze_split


def image(change=False, offset=0, scale=1, width=64, height=64):
    values = []
    for y in range(height):
        for x in range(width):
            v = 60 if (x // 4 + y // 4) % 2 else 100
            if change and x < width // 2:
                v = 160 - v
            values.append(int(v * scale + offset))
    return bytes(values), width, height


class LocalPatternTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(dir='/tmp/opencode')
        cls.local = NativeLocal(cls.temp.name)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def context(self, frame=None):
        frame = frame or image()
        ctx = self.local.context(frame, [0, 0, 1000000, 1000000])
        self.local.calibrate(ctx, [frame])
        return ctx

    def test_same_empty_and_additive_light(self):
        ctx = self.context()
        for offset in (-30, 0, 30, 80):
            self.assertEqual(self.local.measure(ctx, image(offset=offset))['area_pct'], 0)

    def test_positive_gain_invariance(self):
        ctx = self.context()
        for scale in (0.5, 1.5, 2):
            self.assertEqual(self.local.measure(ctx, image(scale=scale))['area_pct'], 0)

    def test_local_pattern_change_and_relative_full(self):
        ctx = self.context()
        result = self.local.measure(ctx, image(change=True))
        self.assertEqual(result['area_pct'], 50)
        self.assertEqual(result['raw_occupied_pixels'], 2048)
        relative = self.local.measure(ctx, image(change=True), 2048)
        self.assertEqual(relative['relative_pct'], 100)
        self.assertIsNone(result['relative_pct'])
        self.assertIsNone(self.local.measure(ctx, image(), 1)['relative_pct'])

    def test_gain_with_foreground_not_lost(self):
        ctx = self.context()
        self.assertEqual(self.local.measure(
            ctx, image(change=True, scale=1.5, offset=10))['area_pct'], 50)

    def test_partial_tiles_cover_entire_roi(self):
        frame = image(width=67, height=69)
        ctx = self.context(frame)
        result = self.local.measure(ctx, frame)
        self.assertEqual(result['total_pixels'], 67 * 69)
        self.assertEqual(result['area_pct'], 0)

    def test_strides_and_buffer_validation(self):
        frame = image()
        ctx = self.context()
        p = b''.join(frame[0][i * 64:(i + 1) * 64] + b'padding!'
                     for i in range(64))
        result_type = __import__('evaluate_local_pattern').LocalResult
        result = result_type()
        buf = (ct.c_uint8 * len(p)).from_buffer_copy(p)
        self.assertEqual(self.local.lib.lp_measure(
            ctx[0], buf, len(p), 72, 0, ct.byref(result)), 0)
        self.assertEqual(result.area_pct, 0)
        self.assertNotEqual(self.local.lib.lp_measure(
            ctx[0], buf, 1, 72, 0, ct.byref(result)), 0)
        self.assertEqual(result.relative_pct, 255)
        with self.assertRaises(ValueError):
            self.local.measure(ctx, (b'\0', 64, 64))

    def test_calibration_bounds_and_state(self):
        ctx = self.local.context(image(), [0, 0, 1000000, 1000000])
        with self.assertRaises(ValueError):
            self.local.measure(ctx, image())
        self.local.calibrate(ctx, [image()] * 9)
        with self.assertRaises(ValueError):
            self.local.calibrate(ctx, [image()])
        ctx = self.local.context(image(), [0, 0, 1000000, 1000000])
        with self.assertRaises(ValueError):
            self.local.calibrate(ctx, [])

    def test_uniform_background_limitation_is_exposed(self):
        empty = (bytes([80]) * 4096, 64, 64)
        ctx = self.context(empty)
        result = self.local.measure(ctx, (bytes([180]) * 4096, 64, 64))
        self.assertEqual(result['weak_reference_pixels'], 4096)
        self.assertEqual(result['area_pct'], 0)
        # Uniform object vs uniform lighting cannot be distinguished here.

    def test_split_is_disjoint_chronological_and_deterministic(self):
        from analyze_shelf_lighting import GROUPS
        with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
            context = Path(directory) / 'context.json'
            context.write_text('{}')
            samples = [{'snapshot_id': f'{group}_{i}', 'group': group}
                       for group in GROUPS for i in range(7)]
            result = freeze_split(samples, context)
            self.assertEqual(result, freeze_split(samples, context))
            rows = result['samples']
            self.assertEqual(len({s['snapshot_id'] for s in rows}), len(rows))
            self.assertEqual(sum(s['partition'] == 'calibration' for s in rows), 6)
            self.assertEqual(sum(s['partition'] == 'holdout' for s in rows), 14)
            for row in rows:
                if row['partition'] == 'holdout':
                    self.assertIn(row['snapshot_id'][-1], '56')


if __name__ == '__main__':
    unittest.main()
