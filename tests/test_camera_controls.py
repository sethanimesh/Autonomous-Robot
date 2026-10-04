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
    def test_manual_buttons_ignore_calibration_and_saved_limits(self):
        h=dict(status(-12),homed=False,require_approved_reference=True,
               reference_id="new",approved_reference_id="old",minimum_position=-54,
               maximum_position=-21,maximum_target_position=-24)
        self.assertEqual(-7,validate_camera_jog(h,.1,5))
        self.assertEqual(3,validate_camera_jog(h,.1,15))
        self.assertEqual(-27,validate_camera_jog(h,.1,-15))
        h.update(homed=True,reference_id="old")
        self.assertEqual(3,validate_camera_jog(h,.1,15))

    def test_manual_buttons_require_a_current_encoder_reading(self):
        for h,age in [(None,.1),(status(),2.1),({},.1)]:
            with self.assertRaises(ValueError):validate_camera_jog(h,age,5)
        for step in [0,1,10,20,True]:
            with self.assertRaises(ValueError):validate_camera_jog(status(),.1,step)

    def test_repeated_same_direction_keeps_target_and_opposite_takes_over(self):
        h=dict(status(-20),moving=True,target_position=-35)
        self.assertEqual(-35,validate_camera_jog(h,.1,-5))
        self.assertEqual(-15,validate_camera_jog(h,.1,5))


class TiltDirectionTests(unittest.TestCase):
    """Encoder degrees decrease while lifting; see camera_controls for evidence."""

    def test_up_is_negative_and_down_is_positive(self):
        self.assertEqual(5, jog_degrees("down"))
        self.assertEqual(-5, jog_degrees("up"))
        self.assertEqual(15, jog_degrees("down", CAMERA_COARSE_JOG_DEGREES))
        self.assertEqual(-15, jog_degrees("up", CAMERA_COARSE_JOG_DEGREES))

    def test_a_down_step_moves_toward_the_maximum(self):
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
