import json
import os
import shutil
import tempfile
import unittest

from robot.ev3.server.ev3_server import HardwareError
from robot.ev3.server.ev3_server import MotorController
from robot.ev3.server.ev3_server import ProtocolError
from robot.ev3.server.ev3_server import SysfsMotor
from robot.ev3.server.ev3_server import decode_request
from robot.ev3.server.ev3_server import encode_response
from robot.ev3.server.ev3_server import handle_request
from robot.ev3.server.ev3_server import process_request_line


class FakeClock(object):
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeMotor(object):
    def __init__(self, port, fail_on_speed=False):
        self.port = port
        self.fail_on_speed = fail_on_speed
        self.position = 0
        self.speed = 0
        self.stop_count = 0
        self.target = None

    def set_speed(self, speed):
        if self.fail_on_speed:
            raise HardwareError("simulated write failure")
        self.speed = speed

    def stop(self):
        self.speed = 0
        self.stop_count += 1

    def move_to(self, position, speed):
        if self.fail_on_speed:
            raise HardwareError("simulated write failure")
        self.target = position
        self.speed = abs(speed) if position > self.position else -abs(speed)

    def set_position(self, position):
        self.position = position

    def snapshot(self):
        return {
            "port": self.port,
            "position": self.position,
            "speed": self.speed,
            "state": ["running"] if self.speed else [],
        }


def make_controller(clock=None, failing_role=None):
    motors = {
        "left": FakeMotor("outB", fail_on_speed=failing_role == "left"),
        "right": FakeMotor("outC", fail_on_speed=failing_role == "right"),
        "tool": FakeMotor("outA", fail_on_speed=failing_role == "tool"),
    }
    controller = MotorController(motors, clock=clock)
    return controller, motors


class SysfsMotorTests(unittest.TestCase):
    def test_re_resolves_port_after_driver_reenumeration(self):
        root = tempfile.mkdtemp()
        try:
            first = os.path.join(root, "motor0")
            os.mkdir(first)
            with open(os.path.join(first, "address"), "w") as handle:
                handle.write("ev3-ports:outA")
            motor = SysfsMotor("outA", root)

            shutil.rmtree(first)
            replacement = os.path.join(root, "motor3")
            os.mkdir(replacement)
            with open(os.path.join(replacement, "address"), "w") as handle:
                handle.write("ev3-ports:outA")

            self.assertEqual("ev3-ports:outA", motor._read("address"))
            self.assertEqual(replacement, motor.path)
        finally:
            shutil.rmtree(root)


class MotorControllerTests(unittest.TestCase):
    def test_startup_stops_every_motor(self):
        controller, motors = make_controller()

        self.assertFalse(controller.motion_active)
        self.assertEqual("startup", controller.last_stop_reason)
        self.assertEqual([1, 1, 1], [motors[name].stop_count for name in motors])

    def test_drive_applies_confirmed_roles_and_speed_limits(self):
        controller, motors = make_controller()

        applied = controller.drive(999, -999, 2000)

        self.assertEqual({"left": 250, "right": -250, "tool": 300}, applied)
        self.assertEqual(250, motors["left"].speed)
        self.assertEqual(-250, motors["right"].speed)
        self.assertEqual(300, motors["tool"].speed)
        self.assertTrue(controller.motion_active)

    def test_zero_drive_stops_all_motors(self):
        controller, motors = make_controller()
        controller.drive(10, 10, 0)

        applied = controller.drive(0, 0, 0)

        self.assertEqual({"left": 0, "right": 0, "tool": 0}, applied)
        self.assertFalse(controller.motion_active)
        self.assertEqual("zero-command", controller.last_stop_reason)
        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])

    def test_watchdog_stops_only_after_timeout(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        controller.drive(100, 100, 0)

        clock.advance(0.49)
        self.assertFalse(controller.enforce_watchdog())
        self.assertEqual(100, motors["left"].speed)

        clock.advance(0.02)
        self.assertTrue(controller.enforce_watchdog())
        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])
        self.assertEqual("watchdog", controller.last_stop_reason)

    def test_partial_motor_write_failure_stops_everything(self):
        controller, motors = make_controller(failing_role="right")

        with self.assertRaises(HardwareError):
            controller.drive(100, 100, 0)

        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])
        self.assertEqual("motor-write-failure", controller.last_stop_reason)


class ProtocolTests(unittest.TestCase):
    def test_ping_and_status(self):
        controller, motors = make_controller()
        motors["left"].position = 123

        ping = handle_request(controller, {"command": "ping"})
        status = handle_request(controller, {"command": "status"})

        self.assertEqual("pong", ping["message"])
        self.assertEqual(3, ping["protocol_version"])
        self.assertEqual(123, status["motors"]["left"]["position"])
        self.assertEqual(500, status["watchdog_timeout_ms"])

    def test_drive_requires_both_track_speeds(self):
        controller, _ = make_controller()

        with self.assertRaises(ProtocolError):
            handle_request(controller, {"command": "drive", "left": 10})

    def test_invalid_speed_is_rejected_without_movement(self):
        controller, motors = make_controller()

        with self.assertRaises(ProtocolError):
            handle_request(
                controller, {"command": "drive", "left": True, "right": 10}
            )

        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])

    def test_position_move_locks_tracks_and_is_bounded(self):
        controller, motors = make_controller()
        controller.zero_tool()

        response = handle_request(
            controller, {"command": "tool_move", "position": 30, "speed": 40}
        )

        self.assertEqual({"position": 30, "speed": 40}, response["applied"])
        self.assertEqual(30, motors["tool"].target)
        self.assertEqual([0, 0], [motors["left"].speed, motors["right"].speed])
        self.assertTrue(controller.tool_motion_active)
        with self.assertRaises(ProtocolError):
            controller.drive(30, 30)
        with self.assertRaises(ProtocolError):
            controller.move_tool(721, 40)

    def test_tool_move_has_independent_timeout(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        controller.zero_tool()
        controller.move_tool(-20, 30)

        clock.advance(3.99)
        self.assertFalse(controller.enforce_watchdog())
        clock.advance(0.02)
        self.assertTrue(controller.enforce_watchdog())

        self.assertEqual(0, motors["tool"].speed)
        self.assertEqual("tool-move-timeout", controller.last_stop_reason)

    def test_tool_must_be_homed_before_position_move(self):
        controller, _ = make_controller()

        with self.assertRaises(ProtocolError):
            controller.move_tool(10, 30)

    def test_home_stops_and_zeroes_after_no_encoder_progress(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        motors["tool"].position = 75

        applied = controller.home_tool(25)
        clock.advance(0.39)
        self.assertFalse(controller.enforce_watchdog())
        clock.advance(0.02)
        self.assertFalse(controller.enforce_watchdog())

        self.assertEqual({"direction": 1, "speed": 25}, applied)
        self.assertTrue(controller.tool_homed)
        self.assertFalse(controller.tool_homing)
        self.assertEqual(0, motors["tool"].position)
        self.assertEqual(0, motors["tool"].speed)
        self.assertEqual("tool-home-complete", controller.last_stop_reason)

    def test_tool_zero_requires_all_motion_stopped(self):
        controller, motors = make_controller()
        motors["tool"].position = 123

        self.assertEqual(0, controller.zero_tool())
        self.assertEqual(0, motors["tool"].position)
        controller.drive(30, 30)
        with self.assertRaises(ProtocolError):
            controller.zero_tool()

    def test_json_line_round_trip(self):
        request = decode_request(b'{"command":"ping"}')
        response = encode_response({"status": "ok"})

        self.assertEqual({"command": "ping"}, request)
        self.assertTrue(response.endswith(b"\n"))
        self.assertEqual({"status": "ok"}, json.loads(response.decode("utf-8")))

    def test_invalid_json_is_rejected(self):
        with self.assertRaises(ProtocolError):
            decode_request(b"not-json")

    def test_invalid_request_stops_active_motion_immediately(self):
        controller, motors = make_controller()
        controller.drive(100, 100, 0)

        response = process_request_line(controller, b"not-json")

        self.assertEqual("error", response["status"])
        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])
        self.assertEqual("protocol-error", controller.last_stop_reason)


if __name__ == "__main__":
    unittest.main()
