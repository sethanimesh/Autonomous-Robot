import unittest

from robot.jetson.perception.camera_controls import CAMERA_COARSE_JOG_DEGREES
from robot.jetson.perception.camera_controls import CAMERA_FINE_JOG_DEGREES
from robot.jetson.perception.camera_controls import jog_degrees
from robot.jetson.perception.camera_controls import validate_camera_jog


def status(position=-20):
    return {
        "homed": True,
        "homing": False,
        "moving": False,
        "position": position,
        "minimum_position": -180,
        "maximum_position": 0,
    }


class CameraControlTests(unittest.TestCase):
    def test_up_and_down_steps_are_bounded(self):
        self.assertEqual(-15, validate_camera_jog(status(), 0.1, 5))
        self.assertEqual(-25, validate_camera_jog(status(), 0.1, -5))

    def test_only_the_published_step_sizes_are_allowed(self):
        for degrees in (10, 1, 20, 0):
            with self.assertRaises(ValueError):
                validate_camera_jog(status(), 0.1, degrees)

    def test_coarse_steps_are_accepted(self):
        self.assertEqual(-5, validate_camera_jog(status(), 0.1, 15))
        self.assertEqual(-35, validate_camera_jog(status(), 0.1, -15))

    def test_stale_or_unhomed_status_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_camera_jog(status(), 2.1, -5)
        unhomed = status()
        unhomed["homed"] = False
        with self.assertRaises(ValueError):
            validate_camera_jog(unhomed, 0.1, -5)

    def test_moving_head_is_rejected(self):
        moving = status()
        moving["moving"] = True
        with self.assertRaises(ValueError):
            validate_camera_jog(moving, 0.1, -5)

    def test_a_step_past_a_limit_is_trimmed_to_that_limit(self):
        # The head drifts off round numbers, so the last few degrees have to
        # stay reachable instead of refusing the whole step.
        self.assertEqual(0, validate_camera_jog(status(-3), 0.1, 5))
        self.assertEqual(0, validate_camera_jog(status(-4), 0.1, 15))
        self.assertEqual(-180, validate_camera_jog(status(-178), 0.1, -5))
        self.assertEqual(-180, validate_camera_jog(status(-170), 0.1, -15))

    def test_head_already_on_a_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_camera_jog(status(0), 0.1, 5)
        with self.assertRaises(ValueError):
            validate_camera_jog(status(-180), 0.1, -5)


class TiltDirectionTests(unittest.TestCase):
    """Encoder degrees increase downward; see camera_controls for the evidence."""

    def test_down_is_positive_and_up_is_negative(self):
        self.assertEqual(5, jog_degrees("down"))
        self.assertEqual(-5, jog_degrees("up"))
        self.assertEqual(15, jog_degrees("down", CAMERA_COARSE_JOG_DEGREES))
        self.assertEqual(-15, jog_degrees("up", CAMERA_COARSE_JOG_DEGREES))

    def test_a_down_step_moves_toward_the_maximum(self):
        # The bottom direction is numeric positive, so "down" approaches maximum.
        self.assertEqual(-15, validate_camera_jog(status(-20), 0.1, jog_degrees("down")))
        self.assertEqual(-25, validate_camera_jog(status(-20), 0.1, jog_degrees("up")))

    def test_unknown_direction_or_magnitude_is_rejected(self):
        for direction in ("left", "", None):
            with self.assertRaises(ValueError):
                jog_degrees(direction)
        with self.assertRaises(ValueError):
            jog_degrees("down", 7)

    def test_fine_step_is_the_default_magnitude(self):
        self.assertEqual(CAMERA_FINE_JOG_DEGREES, abs(jog_degrees("down")))


if __name__ == "__main__":
    unittest.main()
