import unittest

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

    def test_only_five_degree_steps_are_allowed(self):
        with self.assertRaises(ValueError):
            validate_camera_jog(status(), 0.1, 10)

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

    def test_software_limits_are_enforced(self):
        with self.assertRaises(ValueError):
            validate_camera_jog(status(0), 0.1, 5)
        with self.assertRaises(ValueError):
            validate_camera_jog(status(-180), 0.1, -5)


if __name__ == "__main__":
    unittest.main()
