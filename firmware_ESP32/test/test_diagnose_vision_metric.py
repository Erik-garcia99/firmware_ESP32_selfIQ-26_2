import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import diagnose_vision_metric as diag


class MetricTests(unittest.TestCase):
    def setUp(self):
        # Dos descriptores con la misma forma invertida: separación no nula.
        self.empty = [10 + (i % 8) * 10 for i in range(64)]
        self.full = [250 - value for value in self.empty]

    def test_centered_endpoints_and_midpoint(self):
        self.assertAlmostEqual(diag.centered_percentage(self.empty, self.empty, self.full), 0.0)
        self.assertAlmostEqual(diag.centered_percentage(self.full, self.empty, self.full), 100.0)
        middle = [(a + b) / 2 for a, b in zip(self.empty, self.full)]
        self.assertAlmostEqual(diag.centered_percentage(middle, self.empty, self.full), 50.0)

    def test_centering_ignores_constant_offset_not_scale(self):
        shifted = [value + 17 for value in self.empty]
        self.assertAlmostEqual(diag.centered_percentage(shifted, self.empty, self.full), 0.0)
        scaled = [value * 1.2 for value in self.empty]
        self.assertGreater(diag.centered_percentage(scaled, self.empty, self.full), 0.0)
        self.assertIsNone(diag.centered_percentage(self.empty, self.empty, self.empty))

    def test_reference_gate_reasons(self):
        self.assertEqual(diag.reference_gate(self.empty, self.full)['reasons'], [])
        flat = [40] * 64
        self.assertIn('empty_low_contrast', diag.reference_gate(flat, self.full)['reasons'])
        self.assertIn('references_too_close',
                      diag.reference_gate(self.empty, [v + 1 for v in self.empty])['reasons'])
        noisy = [0 if i % 2 else 255 for i in range(64)]
        self.assertIn('empty_high_mad', diag.reference_gate(noisy, self.full)['reasons'])

    def test_frame_gate_matches_core_limits(self):
        ref_means = [40, 200]
        ok = {'status': 'ok', 'mean': 120, 'clipped_pct': 0}
        self.assertIsNone(diag.frame_gate(ok, ref_means))
        self.assertEqual(diag.frame_gate({'status': 'low_contrast', 'mean': 120,
                                          'clipped_pct': 0}, ref_means),
                         'low_contrast')
        self.assertEqual(diag.frame_gate({'status': 'ok', 'mean': 10,
                                          'clipped_pct': 0}, ref_means),
                         'normalization_rejected')
        self.assertEqual(diag.frame_gate({'status': 'ok', 'mean': 120,
                                          'clipped_pct': diag.MAX_CLIPPED_PCT + 1},
                                         ref_means), 'excessive_clipping')

    def test_summary_and_levels(self):
        rows = [{'expected_pct': pct, 'centered': pct + 5} for pct in diag.ev.LEVELS]
        table, monotonic = diag.level_table(rows, 'centered')
        self.assertTrue(monotonic)
        self.assertEqual(table['0']['median'], 5)
        errors, overall = diag.error_table(rows, 'centered')
        self.assertEqual(overall['mean'], 5)
        self.assertEqual(errors['100']['mean'], 5)


if __name__ == '__main__':
    unittest.main()
