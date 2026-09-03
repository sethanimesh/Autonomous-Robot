import unittest

from robot.jetson.perception.build_engine import fixed_batch_shape


class FixedBatchShapeTests(unittest.TestCase):
    def test_dynamic_batch_is_pinned(self):
        self.assertEqual((1, 3, 112, 112), fixed_batch_shape((-1, 3, 112, 112)))

    def test_existing_fixed_shape_is_unchanged(self):
        self.assertEqual((1, 3, 640, 640), fixed_batch_shape((1, 3, 640, 640)))

    def test_dynamic_spatial_axis_is_rejected(self):
        with self.assertRaises(ValueError):
            fixed_batch_shape((-1, 3, -1, 112))
