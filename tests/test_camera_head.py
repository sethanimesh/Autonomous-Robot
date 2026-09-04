import unittest

from robot.jetson.ev3_bridge.camera_head import CameraHeadController
from robot.jetson.ev3_bridge.camera_head import CameraHeadError


class FakeClient(object):
    def __init__(self, position=0):
        self.position = position
        self.commands = []

    def status(self):
        return {
            "motors": {"tool": {"position": self.position}},
            "tool_motion_active": False,
        }

    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        self.position = position
        return {"applied": {"position": position, "speed": speed}}

    def zero_tool(self):
        self.commands.append(("zero",))
        self.position = 0
        return {"position": 0}

    def home_tool(self, speed=25):
        self.commands.append(("home", speed))
        return {"applied": {"direction": 1, "speed": speed}}

    def acknowledge_tool_position(self):
        self.commands.append(("acknowledge_position",))
        return {"position": self.position}

    def stop(self):
        self.commands.append(("stop",))
        return {"stopped": True}


class StalledClient(FakeClient):
    def move_tool(self, position, speed):
        self.commands.append(("move", position, speed))
        return {"applied": {"position": position, "speed": speed}}


class MovingClient(FakeClient):
    def status(self):
        return {
            "motors": {"tool": {"position": self.position}},
            "tool_motion_active": True,
        }


class FakeClock(object):
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class CameraHeadControllerTests(unittest.TestCase):
    def test_uncalibrated_head_allows_only_bounded_jog(self):
        client = FakeClient(position=-20)
        head = CameraHeadController(client)

        head.execute('{"action":"jog","degrees":-10}')

        self.assertEqual([("move", -30, 40)], client.commands)
        with self.assertRaises(CameraHeadError):
            head.execute("look_forward")
        with self.assertRaises(CameraHeadError):
            head.execute('{"action":"jog","degrees":16}')

    def test_jog_is_trimmed_to_the_software_limit(self):
        client = FakeClient(position=-175)
        head = CameraHeadController(client)

        head.jog(-10)

        self.assertEqual([("move", -180, 40)], client.commands)

    def test_jog_is_rejected_once_the_head_sits_on_the_limit(self):
        client = FakeClient(position=-180)
        head = CameraHeadController(client)

        with self.assertRaises(CameraHeadError):
            head.jog(-10)

        self.assertEqual([], client.commands)

    def test_calibrated_named_positions(self):
        client = FakeClient(position=0)
        head = CameraHeadController(
            client, calibrated=True, forward_position=0, down_position=-75
        )

        head.execute("look_down")
        head.execute("look_forward")

        self.assertEqual(
            [
                ("move", -15, 40),
                ("move", -30, 40),
                ("move", -45, 40),
                ("move", -60, 40),
                ("move", -75, 40),
                ("move", -60, 40),
                ("move", -45, 40),
                ("move", -30, 40),
                ("move", -15, 40),
                ("move", 0, 40),
            ],
            client.commands,
        )

    def test_zero_is_for_calibration_only(self):
        client = FakeClient(position=50)
        uncalibrated = CameraHeadController(client)
        calibrated = CameraHeadController(client, calibrated=True)

        uncalibrated.execute("zero")
        with self.assertRaises(CameraHeadError):
            calibrated.execute("zero")

    def test_home_is_available_after_reboot(self):
        client = FakeClient(position=50)
        head = CameraHeadController(client, calibrated=True)

        head.execute("home")

        self.assertEqual([("home", 60)], client.commands)

    def test_vision_calibration_can_replace_named_positions_for_this_boot(self):
        client = FakeClient(position=0)
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-30,
            down_position=0,
            minimum_position=-150,
            maximum_position=0,
        )

        result = head.execute(
            {"action": "set_runtime_positions", "forward": -75, "down": 0}
        )
        head.execute("look_forward")

        self.assertEqual(-75, result["forward_position"])
        self.assertEqual(
            [
                ("move", -15, 40),
                ("move", -30, 40),
                ("move", -45, 40),
                ("move", -60, 40),
                ("move", -75, 40),
            ],
            client.commands,
        )

    def test_runtime_positions_remain_within_mechanical_limits(self):
        head = CameraHeadController(
            FakeClient(),
            minimum_position=-150,
            maximum_position=0,
        )

        with self.assertRaises(CameraHeadError):
            head.set_runtime_positions(-151, 0)
        with self.assertRaises(CameraHeadError):
            head.set_runtime_positions(-30, -40)

    def test_named_move_fails_immediately_when_loaded_head_does_not_progress(self):
        head = CameraHeadController(
            StalledClient(position=0),
            calibrated=True,
            forward_position=-30,
            down_position=0,
            minimum_position=-150,
            maximum_position=0,
        )

        with self.assertRaisesRegex(CameraHeadError, "stalled"):
            head.execute("look_forward")

    def test_named_move_stops_motor_when_running_status_never_clears(self):
        clock = FakeClock()
        client = MovingClient(position=0)
        head = CameraHeadController(
            client,
            calibrated=True,
            forward_position=-30,
            down_position=0,
            minimum_position=-150,
            maximum_position=0,
            step_timeout_seconds=0.25,
            clock=clock,
            sleeper=clock.sleep,
        )

        with self.assertRaisesRegex(CameraHeadError, "timed out"):
            head.execute("look_forward")

        self.assertEqual(("stop",), client.commands[-1])

    def test_retained_position_can_be_acknowledged_after_reboot(self):
        client = FakeClient(position=63)
        head = CameraHeadController(client, calibrated=True)

        head.execute("acknowledge_position")

        self.assertEqual([("acknowledge_position",)], client.commands)

    def test_description_reports_named_position(self):
        client = FakeClient(position=-74)
        head = CameraHeadController(
            client, calibrated=True, forward_position=0, down_position=-75
        )

        description = head.describe(client.status())

        self.assertEqual("down", description["named_position"])
        self.assertEqual(-74, description["position"])


if __name__ == "__main__":
    unittest.main()
