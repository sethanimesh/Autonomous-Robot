import math
import unittest

from robot.jetson.navigation.ground_plane import expected_floor_z_m
from robot.jetson.navigation.ground_plane import fit_ground_plane
from robot.jetson.navigation.ground_plane import floor_depth_ratio


class GroundPlaneTests(unittest.TestCase):
    focal_y = 413.608
    center_y = 235.303
    height = 0.15

    def synthetic_samples(self, pitch=27.0, scale=0.0675):
        samples = []
        for pixel_y in range(285, 451, 3):
            expected = expected_floor_z_m(
                pixel_y, self.focal_y, self.center_y, pitch, self.height
            )
            samples.append((pixel_y, expected / scale))
        return samples

    def test_recovers_pitch_and_scale_from_relative_depth(self):
        fit = fit_ground_plane(
            self.synthetic_samples(), self.focal_y, self.center_y, self.height
        )
        self.assertAlmostEqual(27.0, fit.pitch_degrees, delta=0.25)
        self.assertAlmostEqual(0.0675, fit.depth_scale, places=4)
        self.assertLess(fit.median_log_error, 0.001)

    def test_fit_acceptance_requires_enough_consistent_samples(self):
        fit = fit_ground_plane(
            self.synthetic_samples(), self.focal_y, self.center_y, self.height
        )
        self.assertFalse(fit.accepted(minimum_samples=100))
        self.assertTrue(fit.accepted(minimum_samples=50))

    def test_depth_ratio_is_one_for_floor_and_lower_for_obstacle(self):
        fit = fit_ground_plane(
            self.synthetic_samples(), self.focal_y, self.center_y, self.height
        )
        pixel_y = 390
        expected = expected_floor_z_m(
            pixel_y, self.focal_y, self.center_y, fit.pitch_degrees, self.height
        )
        raw_floor = expected / fit.depth_scale
        self.assertAlmostEqual(
            1.0,
            floor_depth_ratio(
                pixel_y,
                raw_floor,
                fit,
                self.focal_y,
                self.center_y,
                self.height,
            ),
            places=5,
        )
        self.assertLess(
            floor_depth_ratio(
                pixel_y,
                raw_floor * 0.5,
                fit,
                self.focal_y,
                self.center_y,
                self.height,
            ),
            0.6,
        )

    def test_invalid_samples_are_rejected(self):
        with self.assertRaises(ValueError):
            fit_ground_plane(
                [(300, math.nan), (320, -1), (340, 0)],
                self.focal_y,
                self.center_y,
                self.height,
            )


if __name__ == "__main__":
    unittest.main()

