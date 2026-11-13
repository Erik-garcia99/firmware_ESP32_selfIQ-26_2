"""Tests del evaluador: fixtures sintéticos, no evidencia del piloto."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import zlib

from PIL import Image
from evaluate_vision_photos import NativeCore, digest, evaluate, load_image, summary, sweep_references


class EvaluationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native_dir = tempfile.TemporaryDirectory(dir='/tmp/opencode')
        cls.core = NativeCore(cls.native_dir.name)

    @classmethod
    def tearDownClass(cls):
        cls.native_dir.cleanup()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir='/tmp/opencode')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = self.root / 'manifest.json'
        for pct in (0, 25, 50, 75, 100):
            self.image(f'{pct}.png', pct, shift=1)
        self.image('empty.png', 0)
        self.image('full.png', 100)
        self.m = {'schema_version': 1, 'dataset_id': 'synthetic-test-NOT-pilot',
                  'mode': 'exploratory', 'context': None,
                  'roi_millionths': [0, 0, 1000000, 1000000],
                  'references': {'empty': {'image': 'empty.png'}, 'full': {'image': 'full.png'}},
                  'samples': [{'image': f'{p}.png', 'expected_pct': p, 'group': str(p)}
                              for p in (0, 25, 50, 75, 100)]}

    def image(self, name, pct, shift=0, size=16):
        im = Image.new('L', (size, size))
        im.putdata([128+shift+(32 if (x*8//size)%2 else -32)
                    +(16 if (y*8//size)%2 else -16)*(pct-50)//50
                    for y in range(size) for x in range(size)])
        im.save(self.root / name)

    def run_manifest(self):
        self.path.write_text(json.dumps(self.m))
        return evaluate(self.path, self.core, repeats=2)

    def acceptance(self):
        self.m['mode'] = 'acceptance'
        context = {'roi_id': 'f71aaeca-d168-43e4-9eee-1001aa524f2f',
                   'camera_id': '00000000-0000-0000-0000-000000000002',
                   'context_hash': 'a'*64, 'config_version': 9, 'calibration_version': 2}
        self.m['context'] = context
        for kind in ('empty', 'full'):
            frame = load_image(self.root / f'{kind}.png')
            data = self.core.descriptor(frame, self.m['roi_millionths'])
            self.m['references'][kind] = {'descriptor_hex': data.hex(),
                                          'crc32': f'{zlib.crc32(data):08x}',
                                          'context': copy.deepcopy(context)}
        for sample in self.m['samples']:
            pct = sample['expected_pct']
            self.image(sample['image'], pct, shift=1, size=640)
            # Construir VGA sin reescalar textura: usar recorte de la fixture.
            with Image.open(self.root / sample['image']) as im:
                im.crop((0, 0, 640, 480)).save(self.root / sample['image'])
            sample['sha256'] = digest((self.root / sample['image']).read_bytes())
            sample['context'] = copy.deepcopy(context)

    def test_levels_errors_and_timing(self):
        report = self.run_manifest()
        for level in (0, 25, 50, 75, 100):
            group = report['groups'][str(level)]
            self.assertEqual(group['occupancy_pct']['mean'], level)
            self.assertEqual(group['absolute_error_pct_points']['max'], 0)
            self.assertGreaterEqual(group['host_core_ms']['min'], 0)
        self.assertFalse(report['integration_gate']['approved'])
        self.assertEqual(len(report['source_sha256']), 5)
        self.assertTrue(report['labelled_level_medians_nondecreasing'])
        self.assertEqual(report['algorithm_version'], 2)

    def test_nonmonotonic_labels_are_explicit_blocker(self):
        for sample in self.m['samples']:
            sample['expected_pct'] = 100 - sample['expected_pct']
        report = self.run_manifest()
        self.assertFalse(report['labelled_level_medians_nondecreasing'])
        self.assertIn('labelled_level_medians_not_monotonic', report['integration_gate']['blockers'])

    def test_unknown_labels_do_not_invent_error(self):
        for sample in self.m['samples']:
            sample['expected_pct'] = None
        report = self.run_manifest()
        self.assertTrue(all(g['absolute_error_pct_points'] is None for g in report['groups'].values()))
        self.assertIsNone(report['labelled_level_medians_nondecreasing'])

    def test_exclude_reference_duplicate_and_hand(self):
        self.m['samples'][0]['image'] = 'empty.png'
        self.m['samples'].append(dict(self.m['samples'][1]))
        self.m['samples'][2]['exclude_reason'] = 'hand_in_roi'
        rows = self.run_manifest()['samples']
        self.assertEqual(rows[0]['exclude_reason'], 'reference_image_not_independent')
        self.assertEqual(rows[-1]['exclude_reason'], 'duplicate_image_not_independent')
        self.assertNotIn('status', rows[2])

    def test_rejections_not_zero_and_counted(self):
        Image.new('L', (16, 16), 0).save(self.root / '50.png')
        report = self.run_manifest()
        self.assertIsNone(report['samples'][2]['occupancy_pct'])
        self.assertEqual(report['groups']['50']['rejections'], {'excessive_clipping': 1})
        self.assertIsNone(report['groups']['50']['absolute_error_pct_points'])

    def test_rgb_rejected(self):
        Image.new('RGB', (16, 16)).save(self.root / '25.png')
        with self.assertRaisesRegex(ValueError, 'gris L'):
            self.run_manifest()

    def test_invalid_labels(self):
        for value in (True, float('nan'), float('inf'), -1, 101):
            self.m['samples'][0]['expected_pct'] = value
            with self.assertRaisesRegex(ValueError, 'expected_pct'):
                self.run_manifest()

    def test_invalid_roi(self):
        for value in ([0, 0, 0, 10], [0, 0, 1.5, 10], [999999, 0, 2, 20]):
            self.m['roi_millionths'] = value
            with self.assertRaisesRegex(ValueError, 'ROI'):
                self.run_manifest()

    def test_empty_dataset(self):
        self.m['samples'] = []
        with self.assertRaisesRegex(ValueError, 'sin muestras'):
            self.run_manifest()

    def test_acceptance_missing_context(self):
        self.m['mode'] = 'acceptance'
        with self.assertRaisesRegex(ValueError, 'contexto real'):
            self.run_manifest()

    def test_acceptance_vga_still_never_autoapproves(self):
        self.acceptance()
        report = self.run_manifest()
        self.assertFalse(report['integration_gate']['approved'])
        self.assertEqual(report['references']['empty']['source'], 'persisted_descriptor')

    def test_acceptance_rejects_context_mismatch(self):
        self.acceptance()
        self.m['samples'][0]['context']['config_version'] = 10
        with self.assertRaisesRegex(ValueError, 'Contexto de muestra'):
            self.run_manifest()

    def test_acceptance_crc_mismatch(self):
        self.acceptance()
        self.m['references']['empty']['crc32'] = '00000000'
        with self.assertRaisesRegex(ValueError, 'CRC incorrecto'):
            self.run_manifest()

    def test_acceptance_sha_mismatch(self):
        self.acceptance()
        self.m['samples'][0]['sha256'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            self.run_manifest()

    def test_acceptance_resolution_mismatch(self):
        self.acceptance()
        self.image('0.png', 0)
        self.m['samples'][0]['sha256'] = digest((self.root / '0.png').read_bytes())
        with self.assertRaisesRegex(ValueError, 'VGA'):
            self.run_manifest()

    def test_acceptance_rejects_reconstructed_reference(self):
        self.acceptance()
        self.m['references']['empty']['image'] = 'empty.png'
        with self.assertRaisesRegex(ValueError, 'persistidos'):
            self.run_manifest()

    def test_native_median_reference(self):
        descriptors = []
        for shift in (0, 1, 2):
            self.image(f'median{shift}.png', 50, shift=shift)
            descriptors.append(self.core.descriptor(
                load_image(self.root / f'median{shift}.png'),
                self.m['roi_millionths']))
        merged = self.core.median(descriptors)
        self.assertIsInstance(merged, bytes)
        self.assertEqual(len(merged), 66)
        self.assertEqual(merged[:2], bytes([1, 8]))
        # Un corrimiento uniforme de +1 hace que la mediana sea la toma central.
        self.assertEqual(list(merged[2:]), list(descriptors[1][2:]))
        with self.assertRaisesRegex(ValueError, 'entre 1 y 9'):
            self.core.median(descriptors * 4)
        with self.assertRaisesRegex(ValueError, '66 bytes'):
            self.core.median([b'corto'])

    def test_summary(self):
        self.assertIsNone(summary([]))
        stats = summary([1, 2, 3])
        self.assertEqual(stats['median'], 2)
        self.assertEqual(stats['mean'], 2)
        self.assertEqual(stats['max'], 3)

    def test_reference_sweep(self):
        self.run_manifest()
        report = sweep_references(self.path, self.core, '0', '100')
        self.assertEqual(report['pairs'], 1)
        self.assertEqual(report['status_counts_across_overlapping_folds']['ok'], 3)
        self.assertEqual(report['pair_results'][0]['group_medians']['50'], 50)

    def test_sweep_forbidden_for_acceptance(self):
        self.acceptance()
        self.run_manifest()
        with self.assertRaisesRegex(ValueError, 'solo admite modo exploratory'):
            sweep_references(self.path, self.core, '0', '100')


if __name__ == '__main__':
    unittest.main()
