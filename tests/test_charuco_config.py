import math
import unittest

from robot.jetson.camera.charuco_config import CharucoConfigError
from robot.jetson.camera.charuco_config import descriptor_distance
from robot.jetson.camera.charuco_config import dataset_geometry_reasons
from robot.jetson.camera.charuco_config import is_novel_view
from robot.jetson.camera.charuco_config import validate_board
from robot.jetson.camera.charuco_config import validate_result
from robot.jetson.camera.charuco_config import view_descriptor


class BoardTests(unittest.TestCase):
    def test_default_shape_is_valid(self):
        self.assertEqual((5, 7, 0.025, 0.018), validate_board(5, 7, 0.025, 0.018))

    def test_marker_must_be_smaller_than_square(self):
        with self.assertRaises(CharucoConfigError):
            validate_board(5, 7, 0.025, 0.025)


class ViewTests(unittest.TestCase):
    def test_descriptor_normalizes_center_and_coverage(self):
        descriptor = view_descriptor([(160, 120), (480, 120), (160, 360), (480, 360)], 640, 480)
        self.assertAlmostEqual(0.5, descriptor[0])
        self.assertAlmostEqual(0.5, descriptor[1])
        self.assertAlmostEqual(math.sqrt(0.25), descriptor[2])

    def test_nested_opencv_style_points_are_accepted(self):
        descriptor = view_descriptor([[[10, 20]], [[30, 40]]], 100, 100)
        self.assertEqual(4, len(descriptor))

    def test_duplicate_and_novel_views(self):
        first = (0.5, 0.5, 0.3, 0.1)
        self.assertFalse(is_novel_view((0.51, 0.51, 0.3, 0.1), [first], 0.08))
        self.assertTrue(is_novel_view((0.2, 0.2, 0.3, 0.1), [first], 0.08))

    def test_angle_wrap_does_not_make_same_pose_novel(self):
        self.assertLess(descriptor_distance((0, 0, 0, -0.99), (0, 0, 0, 0.99)), 0.02)


class ResultTests(unittest.TestCase):
    def good_result(self):
        return validate_result(
            640, 480, 0.35,
            [[500.0, 0.0, 320.0], [0.0, 505.0, 240.0], [0.0, 0.0, 1.0]],
            [0.1, -0.2, 0.0, 0.0, 0.03], [0.2] * 20, 20,
        )

    def test_good_result_is_accepted(self):
        self.assertEqual([], self.good_result())

    def test_high_reprojection_errors_are_rejected(self):
        reasons = validate_result(
            640, 480, 1.1,
            [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            [0, 0, 0, 0, 0], [0.2] * 19 + [1.6], 20,
        )
        self.assertTrue(any("RMS" in reason for reason in reasons))
        self.assertTrue(any("worst view" in reason for reason in reasons))

    def test_implausible_intrinsics_are_rejected(self):
        reasons = validate_result(
            640, 480, 0.2,
            [[10, 0, 700], [0, 10, 240], [0, 0, 1]],
            [0, 0, 0, 0, 0], [0.2] * 20, 20,
        )
        self.assertTrue(any("focal" in reason for reason in reasons))
        self.assertTrue(any("principal" in reason for reason in reasons))

    def test_destructive_rectification_is_rejected(self):
        reasons = validate_result(
            640, 480, 0.2,
            [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            [0, 0, 0, 0, 0], [0.2] * 20, 20,
            valid_roi=[0, 0, 244, 198], intrinsic_stddev=[1.0, 1.0],
        )
        self.assertTrue(any("valid pixels" in reason for reason in reasons))

    def test_high_focal_uncertainty_is_rejected(self):
        reasons = validate_result(
            640, 480, 0.2,
            [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            [0, 0, 0, 0, 0], [0.2] * 20, 20,
            valid_roi=[0, 0, 620, 460], intrinsic_stddev=[30.0, 30.0],
        )
        self.assertTrue(any("uncertainty" in reason for reason in reasons))

    def test_zero_focal_length_with_uncertainty_does_not_divide_by_zero(self):
        reasons = validate_result(
            640, 480, 0.2,
            [[0, 0, 320], [0, 0, 240], [0, 0, 1]],
            [0, 0, 0, 0, 0], [0.2] * 20, 20,
            valid_roi=[0, 0, 620, 460], intrinsic_stddev=[1.0, 1.0],
        )
        self.assertTrue(any("uncertainty" in reason for reason in reasons))


class DatasetGeometryTests(unittest.TestCase):
    def test_full_sensor_coverage_is_accepted(self):
        points = []
        for row in range(3):
            for column in range(3):
                x = 20 + column * 290
                y = 20 + row * 210
                points.append([(x, y), (x + 140, y), (x, y + 130), (x + 140, y + 130)])
        points.extend([[(100, 100), (500, 100), (100, 350), (500, 350)]] * 6)
        self.assertEqual([], dataset_geometry_reasons(points, 640, 480))

    def test_central_small_views_are_rejected(self):
        points = [[(280, 200), (360, 200), (280, 280), (360, 280)]] * 30
        reasons = dataset_geometry_reasons(points, 640, 480)
        self.assertTrue(any("left and right" in reason for reason in reasons))
        self.assertTrue(any("top and bottom" in reason for reason in reasons))
        self.assertTrue(any("close views" in reason for reason in reasons))
        self.assertTrue(any("3x3" in reason for reason in reasons))


if __name__ == "__main__":
    unittest.main()
