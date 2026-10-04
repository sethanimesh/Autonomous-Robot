"""Check configured support surfaces against the checkpoint's semantic labels."""
import unittest

from robot.mac.route_perception import RoutePerceptionEngine, validate_floor_label_ids


class FloorLabelMappingTests(unittest.TestCase):
    # Extracted from the pinned ADE20K checkpoint config, not inferred from code.
    ADE_LABELS = {3: 'floor', 21: 'water', 28: 'rug'}

    def test_default_floor_mask_excludes_water(self):
        ids = RoutePerceptionEngine(device='cpu').floor_ids
        self.assertNotIn(21, ids)
        self.assertEqual({self.ADE_LABELS[value] for value in ids}, {'floor', 'rug'})
        validate_floor_label_ids(self.ADE_LABELS, ids)

    def test_explicit_water_configuration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'water'):
            validate_floor_label_ids(self.ADE_LABELS, (3, 21, 28))

    def test_matching_semantics_can_use_different_indices(self):
        validate_floor_label_ids({'10': 'floor', '12': 'rug', '21': 'water'}, (10, 12))

    def test_unknown_class_cannot_be_assumed_floor(self):
        with self.assertRaisesRegex(ValueError, 'not floor/rug'):
            validate_floor_label_ids(self.ADE_LABELS, (3, 99))

    def test_empty_or_duplicate_configuration_is_rejected(self):
        for ids in ((), (3, 3)):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                validate_floor_label_ids(self.ADE_LABELS, ids)


if __name__ == '__main__':
    unittest.main()
