import math
import unittest

from robot.jetson.perception.manual_drive import ManualDriveController
from robot.jetson.perception.manual_drive import ManualDriveError
from robot.jetson.perception.manual_drive import yaw_from_quaternion


class FakeClock(object):
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class ManualDriveTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.drive = ManualDriveController(
            clock=self.clock,
            token_factory=lambda: "safe-token",
        )
        self.drive.update_odometry(0.5)

    def enable(self):
        return self.drive.enable(True, True, True, False, False)

    def command(self, direction, sequence=0):
        return self.drive.command(
            direction, "safe-token", sequence, True, True, False
        )

    def test_enable_requires_neutral_fresh_camera_robot_and_odometry(self):
        invalid = (
            (False, True, True, False, False),
            (True, False, True, False, False),
            (True, True, False, False, False),
            (True, True, True, True, False),
            (True, True, True, False, True),
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ManualDriveError):
                self.drive.enable(*values)

        self.clock.advance(2.0)
        with self.assertRaisesRegex(ManualDriveError, "odometry"):
            self.enable()

    def test_existing_session_cannot_be_rebased_as_a_new_neutral(self):
        self.enable()
        self.drive.update_odometry(0.25)

        with self.assertRaisesRegex(ManualDriveError, "existing"):
            self.enable()

    def test_forward_and_reverse_use_low_distinct_speeds(self):
        self.enable()

        forward = self.command("forward")
        self.command("stop", 1)
        reverse = self.command("back", 2)

        self.assertEqual((0.03, 0.0), (forward["linear"], forward["angular"]))
        self.assertEqual((-0.02, 0.0), (reverse["linear"], reverse["angular"]))

    def test_release_stop_clears_active_motion(self):
        self.enable()
        self.command("left")

        result = self.command("stop", 1)

        self.assertFalse(result["active"])
        self.assertEqual((0.0, 0.0), (result["linear"], result["angular"]))

    def test_stale_or_wrong_session_command_is_rejected(self):
        self.enable()
        self.command("forward", 3)

        with self.assertRaisesRegex(ManualDriveError, "stale"):
            self.command("forward", 2)
        with self.assertRaisesRegex(ManualDriveError, "session"):
            self.drive.command("forward", "wrong", 4, True, True, False)

    def test_direction_change_requires_a_stop(self):
        self.enable()
        self.command("forward", 0)

        with self.assertRaisesRegex(ManualDriveError, "Release"):
            self.command("left", 1)

        self.assertFalse(self.drive.status()["active"])

    def test_deadman_watchdog_stops_when_browser_heartbeats_end(self):
        self.enable()
        self.command("forward", 0)
        self.clock.advance(0.26)

        self.assertTrue(self.drive.watchdog(True, True, False))
        self.assertFalse(self.drive.status()["active"])

    def test_turns_are_blocked_before_tether_envelope(self):
        self.enable()
        self.drive.update_odometry(0.5 - math.radians(78.0))

        with self.assertRaisesRegex(ManualDriveError, "Right turn blocked"):
            self.command("right")

        inward = self.command("left", 1)
        self.assertGreater(inward["angular"], 0.0)

    def test_camera_or_ev3_loss_stops_active_drive(self):
        self.enable()
        self.command("back")

        self.assertTrue(self.drive.watchdog(False, True, False))
        self.assertFalse(self.drive.status()["active"])

    def test_disable_invalidates_delayed_commands(self):
        self.enable()
        self.drive.stop(disable=True)

        with self.assertRaisesRegex(ManualDriveError, "Enable"):
            self.command("forward")

    def test_quaternion_yaw_helper(self):
        yaw = math.radians(45.0)

        result = yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

        self.assertAlmostEqual(yaw, result)


if __name__ == "__main__":
    unittest.main()
