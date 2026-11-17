from datetime import datetime
import json
from pathlib import Path
import tempfile
import unittest

from analyze_shelf_lighting import evaluate_scenario, group_name, inventory, summarize
from evaluate_vision_photos import digest


class LightingAnalysisTests(unittest.TestCase):
    def test_filename_groups_and_typo_extension(self):
        self.assertEqual(group_name('ref_full_nromal__luz_6.jpg'), 'full_bright')
        self.assertEqual(group_name('ref_full_nromal_1.jpg'), 'full_normal')
        self.assertEqual(group_name('ref_vacia_mas_luz_v3.jpg'), 'empty_bright')
        self.assertEqual(group_name('mitad_desorden_3jpg'), 'half_disordered')
        self.assertIsNone(group_name('20261113_original.jpg'))

    def test_hash_mapping_not_filename_or_sequence(self):
        with tempfile.TemporaryDirectory(dir='/tmp/opencode') as directory:
            path = Path(directory)
            entries = []
            for name, raw, hour in [
                ('ref_vacia.jpg', b'historical', '04'),
                ('mitad_desorden_3jpg', b'new', '21'),
            ]:
                (path / name).write_bytes(raw)
                entries.append({
                    'sha256': digest(raw), 'snapshot_id': name,
                    'image': 'original_' + name,
                    'captured_at': f'2026-11-13T{hour}:30:00+00:00',
                })
            (path / 'export_index.json').write_text(json.dumps({'samples': entries}))
            rows, excluded, aliases = inventory(
                path, datetime.fromisoformat('2026-11-13T21:00:00+00:00'))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['expected_pct'], 50)
            self.assertEqual(rows[0]['original_image'], 'original_mitad_desorden_3jpg')
            self.assertEqual(len(excluded), 1)
            self.assertEqual(len(aliases), 2)
            (path / 'mitad_desorden_3jpg').write_bytes(b'changed')
            with self.assertRaises(ValueError):
                inventory(path, datetime.fromisoformat('2026-11-13T21:00:00+00:00'))

    def test_rejections_do_not_disappear_from_group_denominator(self):
        row = {'group': 'empty_normal', 'expected_pct': 0,
               'core_pct': None, 'shelf_area_pct': 0, 'shelf_relative_full_pct': 0}
        result = summarize([row])
        group = result['core_pct']['groups']['empty_normal']
        self.assertEqual(group['within_15_points_rate_all_labelled'], 0)
        self.assertIsNone(group['mae_nominal_points'])
        self.assertIsNone(group['output_median'])

    def test_no_samples_no_accuracy(self):
        result = summarize([])
        self.assertIsNone(result['core_pct']['within_15_points_rate_all_labelled'])

    def test_calibration_samples_excluded_and_context_destroyed(self):
        class Core:
            def measure(self, *args):
                return {'status': 'ok', 'occupancy_pct': 50}

        class Shelf:
            destroyed = False

            def create(self, *args):
                return 1

            def calibrate(self, *args):
                pass

            def measure(self, *args):
                return 50

            def destroy(self, *args):
                self.destroyed = True

        shelf = Shelf()
        frame = (bytes(64), 8, 8)
        samples = [{'image': str(i), 'snapshot_id': str(i),
                    'group': 'half_normal', 'expected_pct': 50} for i in range(2)]
        result = evaluate_scenario(
            'test', samples, {'0'}, [frame], [frame], [b'a', b'b'],
            [0, 0, 1000000, 1000000], {'0': frame, '1': frame},
            Core(), shelf)
        self.assertEqual(len(result['rows']), 1)
        self.assertEqual(result['rows'][0]['snapshot_id'], '1')
        self.assertTrue(shelf.destroyed)


if __name__ == '__main__':
    unittest.main()
