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
        return {"applied": {"position": position, "speed": speed}}

    def zero_tool(self):
        self.commands.append(("zero",))
        self.position = 0
        return {"position": 0}

    def home_tool(self):
        self.commands.append(("home",))
        return {"applied": {"direction": 1, "speed": 25}}

    def stop(self):
        self.commands.append(("stop",))
        return {"stopped": True}


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

    def test_jog_cannot_cross_software_limit(self):
        client = FakeClient(position=-175)
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

        self.assertEqual([("move", -75, 40), ("move", 0, 40)], client.commands)

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

        self.assertEqual([("home",)], client.commands)

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
