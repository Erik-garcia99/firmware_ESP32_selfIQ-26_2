import random
import tempfile
import unittest

from diagnose_local_regression import RegistrationProbe, reference_guard, translate


class RegistrationDiagnosisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(dir='/tmp/opencode')
        cls.probe = RegistrationProbe(cls.temp.name)
        rng = random.Random(20261113)
        blocks = [[rng.randrange(30, 160) for _ in range(40)] for _ in range(40)]
        cls.frame = (bytes(blocks[y // 4][x // 4]
                           for y in range(160) for x in range(160)), 160, 160)
        cls.roi = [375000, 375000, 250000, 250000]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_identical_reference(self):
        result = self.probe.measure(self.frame, self.frame, self.roi, 1)
        self.assertEqual((result['dx'], result['dy']), (0, 0))
        self.assertAlmostEqual(result['unshifted_correlation'], 1)

    def test_known_shift_with_ring_excluding_roi(self):
        result = self.probe.measure(self.frame, translate(self.frame, 4, 8), self.roi, 1)
        self.assertEqual((result['dx'], result['dy']), (-4, -8))
        self.assertAlmostEqual(result['best_correlation'], 1)
        self.assertEqual(reference_guard(result),
                         'misaligned_reference_pair_recalibration_required')

    def test_affine_brightness_not_interpreted_as_translation(self):
        pixels, w, h = self.frame
        brighter = (bytes(v + 25 for v in pixels), w, h)
        result = self.probe.measure(self.frame, brighter, self.roi, 1)
        self.assertEqual((result['dx'], result['dy']), (0, 0))
        self.assertAlmostEqual(result['best_correlation'], 1)

    def test_product_inside_roi_does_not_drive_ring_registration(self):
        pixels, w, h = self.frame
        modified = bytearray(pixels)
        # Leave >4 px between synthetic product and ring for high-pass kernel.
        for y in range(68, 92):
            for x in range(68, 92):
                modified[y * w + x] = 250
        result = self.probe.measure(self.frame, (bytes(modified), w, h), self.roi, 1)
        self.assertEqual((result['dx'], result['dy']), (0, 0))
        self.assertAlmostEqual(result['best_correlation'], 1)

    def test_weak_match_is_never_autocorrected(self):
        result = {'dx': 20, 'dy': -12, 'points': 1000,
                  'unshifted_correlation': 0.12, 'best_correlation': 0.14}
        self.assertEqual(reference_guard(result), 'inconclusive_do_not_autocorrect')
        result['best_correlation'] = float('nan')
        self.assertEqual(reference_guard(result), 'inconclusive_do_not_autocorrect')

    def test_uniform_texture_is_inconclusive(self):
        frame = (bytes([80]) * 25600, 160, 160)
        result = self.probe.measure(frame, frame, self.roi, 1)
        self.assertEqual(reference_guard(result), 'inconclusive_do_not_autocorrect')

    def test_invalid_frame_shape(self):
        with self.assertRaises(ValueError):
            self.probe.measure(self.frame, (b'\0', 160, 160), self.roi, 1)


if __name__ == '__main__':
    unittest.main()
