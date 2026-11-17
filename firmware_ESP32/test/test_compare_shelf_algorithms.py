import tempfile
import unittest

from compare_shelf_algorithms import NativeShelf, metrics, roi_pixels


class ShelfComparisonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(dir='/tmp/opencode')
        cls.shelf = NativeShelf(cls.temp.name)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    @staticmethod
    def frame(occupied_columns=0, base=80):
        return (bytes(150 if x < occupied_columns else base
                      for y in range(32) for x in range(32)), 32, 32)

    def setUp(self):
        self.ctx = self.shelf.create(self.frame(), [0, 0, 1000000, 1000000])
        self.shelf.calibrate(self.ctx, [self.frame()])

    def tearDown(self):
        self.shelf.destroy(self.ctx)

    def test_empty_and_local_area(self):
        self.assertEqual(self.shelf.measure(self.ctx, self.frame()), 0)
        for columns, expected in [(8, 25), (16, 50), (24, 75), (32, 100)]:
            self.assertEqual(self.shelf.measure(self.ctx, self.frame(columns)), expected)

    def test_original_calibration_averages_multiple_frames(self):
        self.shelf.calibrate(self.ctx, [self.frame(base=60), self.frame(base=100)])
        self.assertEqual(self.shelf.measure(self.ctx, self.frame(base=80)), 0)

    def test_threshold_and_light_limitation_are_visible(self):
        self.assertEqual(self.shelf.measure(self.ctx, self.frame(base=102)), 0)
        # Un cambio global de brillo engaña al candidato sin illum_roi.
        self.assertEqual(self.shelf.measure(self.ctx, self.frame(base=103)), 100)

    def test_wrong_dimensions_or_short_buffer_rejected_before_c(self):
        for frame in [(b'\0', 32, 32), (bytes(1024), 16, 64)]:
            with self.assertRaises(ValueError):
                self.shelf.measure(self.ctx, frame)
            with self.assertRaises(ValueError):
                self.shelf.calibrate(self.ctx, [frame])

    def test_empty_calibration_rejected(self):
        with self.assertRaises(ValueError):
            self.shelf.calibrate(self.ctx, [])

    def test_roi_rounding_matches_core_geometry(self):
        rect = roi_pixels([172794, 436928, 520221, 411765], 640, 480)
        self.assertEqual([rect.x, rect.y, rect.w, rect.h], [110, 209, 334, 199])
        for roi in ([0, 0, 0, 1], [0, 0, 1000001, 1], [-1, 0, 10, 10],
                    [0, 0, 1, 1]):
            with self.assertRaises(ValueError):
                roi_pixels(roi, 640, 480)

    def test_metrics_rejections_are_not_successes(self):
        result = metrics([
            {'expected_pct': 0, 'x': 10},
            {'expected_pct': 100, 'x': None},
            {'expected_pct': None, 'x': 50},
        ], 'x')
        self.assertEqual(result['mae_nominal_points'], 10)
        self.assertEqual(result['coverage'], 0.5)
        self.assertEqual(result['within_15_points_rate_all_labelled'], 0.5)

    def test_unlabelled_has_no_accuracy(self):
        result = metrics([{'expected_pct': None, 'x': 12}], 'x')
        self.assertIsNone(result['mae_nominal_points'])
        self.assertIsNone(result['within_15_points_rate_all_labelled'])


if __name__ == '__main__':
    unittest.main()
