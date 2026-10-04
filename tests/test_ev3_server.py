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
from robot.ev3.server.ev3_server import parse_args


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
        self.hold_count = 0
        self.target = None
        self.move_history = []
        self.state = None

    def set_speed(self, speed):
        if self.fail_on_speed:
            raise HardwareError("simulated write failure")
        self.speed = speed

    def stop(self):
        self.speed = 0
        self.stop_count += 1

    def hold(self):
        self.speed = 0
        self.hold_count += 1

    def move_to(self, position, speed):
        if self.fail_on_speed:
            raise HardwareError("simulated write failure")
        self.target = position
        self.speed = abs(speed) if position > self.position else -abs(speed)
        self.move_history.append((position, abs(speed)))

    def set_position(self, position):
        self.position = position

    def snapshot(self):
        return {
            "port": self.port,
            "position": self.position,
            "speed": self.speed,
            "state": self.state if self.state is not None else (["running"] if self.speed else []),
        }


def make_controller(clock=None, failing_role=None):
    motors = {
        "left": FakeMotor("outB", fail_on_speed=failing_role == "left"),
        "right": FakeMotor("outC", fail_on_speed=failing_role == "right"),
        "tool": FakeMotor("outA", fail_on_speed=failing_role == "tool"),
    }
    controller = MotorController(motors, clock=clock)
    return controller, motors


class CameraReferenceEpochTests(unittest.TestCase):
    def test_restart_and_rezero_cannot_reuse_a_calibration_reference(self):
        first, _ = make_controller()
        second, _ = make_controller()
        self.assertNotEqual(first.status()["tool_reference_id"], second.status()["tool_reference_id"])
        before = first.status()["tool_reference_id"]
        first.zero_tool()
        after = first.status()["tool_reference_id"]
        self.assertNotEqual(before, after)
        self.assertTrue(first.status()["tool_homed"])
        first.zero_tool()
        self.assertNotEqual(after, first.status()["tool_reference_id"])

    def test_reconnected_motor_loses_home_even_when_encoder_number_is_unchanged(self):
        controller, motors = make_controller()
        controller.zero_tool()
        before = controller.status()["tool_reference_id"]
        original_snapshot = motors["tool"].snapshot
        motors["tool"].snapshot = lambda: dict(original_snapshot(), generation="new-device")
        state = controller.status()
        self.assertFalse(state["tool_homed"])
        self.assertNotEqual(before, state["tool_reference_id"])
        with self.assertRaises(ProtocolError):
            controller.move_tool(-15, 300)


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
    def test_brick_defaults_enforce_the_loaded_head_step_limit(self):
        args = parse_args([])

        self.assertEqual(180, args.tool_position_limit)
        self.assertEqual(15, args.tool_step_limit)
        self.assertEqual(1500, args.tool_speed_limit)

    def test_startup_stops_every_motor(self):
        controller, motors = make_controller()

        self.assertFalse(controller.motion_active)
        self.assertEqual("startup", controller.last_stop_reason)
        self.assertEqual(1, motors["left"].stop_count)
        self.assertEqual(1, motors["right"].stop_count)
        self.assertEqual(0, motors["tool"].stop_count)
        self.assertEqual(1, motors["tool"].hold_count)

    def test_drive_applies_confirmed_roles_and_speed_limits(self):
        controller, motors = make_controller()

        applied = controller.drive(999, -999)

        self.assertEqual({"left": 250, "right": -250}, applied)
        self.assertEqual(250, motors["left"].speed)
        self.assertEqual(-250, motors["right"].speed)
        self.assertEqual(0, motors["tool"].speed)
        self.assertTrue(controller.motion_active)

    def test_zero_drive_stops_all_motors(self):
        controller, motors = make_controller()
        controller.drive(10, 10)

        applied = controller.drive(0, 0)

        self.assertEqual({"left": 0, "right": 0}, applied)
        self.assertFalse(controller.motion_active)
        self.assertEqual("zero-command", controller.last_stop_reason)
        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])

    def test_watchdog_stops_only_after_timeout(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        controller.drive(100, 100)

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
            controller.drive(100, 100)

        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])
        self.assertEqual("motor-write-failure", controller.last_stop_reason)


class ToolSettlementTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.controller, motors = make_controller(clock=self.clock)
        self.tool = motors["tool"]
        self.controller.zero_tool()

    def holding_at(self, position, speed=0):
        self.tool.position = position
        self.tool.speed = speed
        self.tool.state = ["holding"]
        return self.controller.status()

    def test_transient_holding_before_travel_does_not_complete_move(self):
        self.controller.move_tool(15, 300)
        self.assertTrue(self.holding_at(0)["tool_motion_active"])
        self.clock.advance(0.41)
        self.assertTrue(self.controller.status()["tool_motion_active"])
        with self.assertRaisesRegex(ProtocolError, "camera head is moving"):
            self.controller.drive(30, 30)

        self.assertTrue(self.holding_at(15)["tool_motion_active"])
        self.clock.advance(0.41)
        self.assertFalse(self.controller.status()["tool_motion_active"])

    def test_stable_completion_allows_four_counts_and_small_encoder_jitter(self):
        self.controller.move_tool(15, 300)
        self.assertTrue(self.holding_at(18)["tool_motion_active"])
        self.clock.advance(0.2)
        self.assertTrue(self.holding_at(19)["tool_motion_active"])
        self.clock.advance(0.21)
        status = self.controller.status()
        self.assertFalse(status["tool_motion_active"])
        self.assertEqual(0, status["tool_stall_retry_count"])
        self.assertEqual([(15, 300)], self.tool.move_history)

    def test_later_drift_restarts_stability_window(self):
        self.controller.move_tool(15, 300)
        self.holding_at(15)
        self.clock.advance(0.3)
        self.assertTrue(self.holding_at(18)["tool_motion_active"])
        self.clock.advance(0.2)
        self.assertTrue(self.controller.status()["tool_motion_active"])
        self.clock.advance(0.21)
        self.assertFalse(self.controller.status()["tool_motion_active"])

    def test_holding_state_with_speed_is_not_settled(self):
        self.controller.move_tool(15, 300)
        self.holding_at(15, speed=20)
        self.clock.advance(0.41)
        self.assertTrue(self.controller.status()["tool_motion_active"])
        self.assertTrue(self.holding_at(15)["tool_motion_active"])
        self.clock.advance(0.41)
        self.assertFalse(self.controller.status()["tool_motion_active"])

    def test_running_or_stalled_state_cannot_complete_even_at_target(self):
        for state in (["running"], ["holding", "stalled"]):
            with self.subTest(state=state):
                self.controller.stop_all("test-reset")
                self.tool.position = 0
                self.controller.move_tool(15, 300)
                self.tool.position = 15
                self.tool.speed = 0
                self.tool.state = state
                self.controller.status()
                self.clock.advance(0.41)
                self.assertTrue(self.controller.status()["tool_motion_active"])

    def test_off_target_stopped_motor_retries_once_then_reports_stall(self):
        self.controller.move_tool(-15, 300)
        self.holding_at(0)
        started_at = self.controller.tool_move_started_at
        self.clock.advance(1.01)
        self.controller.enforce_watchdog()
        self.assertTrue(self.controller.tool_motion_active)
        self.assertEqual([(-15, 300), (-15, 1500)], self.tool.move_history)
        self.assertEqual(started_at, self.controller.tool_move_started_at)
        self.holding_at(0)
        self.clock.advance(1.01)
        self.controller.enforce_watchdog()
        self.assertFalse(self.controller.tool_motion_active)
        self.assertEqual("tool-stalled-after-retry", self.controller.last_stop_reason)
        self.assertEqual(2, len(self.tool.move_history))

    def test_retry_restarts_settlement_without_extending_deadline(self):
        self.controller.move_tool(15, 300)
        started_at = self.controller.tool_move_started_at
        self.holding_at(12)
        self.clock.advance(0.2)
        self.controller.move_tool(15, 300)
        self.assertEqual(started_at, self.controller.tool_move_started_at)
        self.holding_at(12)
        self.clock.advance(0.21)
        self.assertTrue(self.controller.status()["tool_motion_active"])
        self.clock.advance(0.2)
        self.assertFalse(self.controller.status()["tool_motion_active"])
        self.assertEqual(1, self.controller.tool_stall_retry_count)

    def test_stop_and_new_move_cannot_reuse_previous_settlement_window(self):
        self.controller.move_tool(15, 300)
        self.holding_at(12)
        self.clock.advance(0.2)
        self.controller.stop_all("operator-stop")
        self.controller.move_tool(15, 300)
        self.holding_at(12)
        self.clock.advance(0.21)
        self.assertTrue(self.controller.status()["tool_motion_active"])
        self.clock.advance(0.2)
        self.assertFalse(self.controller.status()["tool_motion_active"])

    def test_move_to_current_position_still_waits_for_stable_hold(self):
        self.controller.move_tool(0, 300)
        self.assertTrue(self.holding_at(0)["tool_motion_active"])
        self.clock.advance(0.41)
        self.assertFalse(self.controller.status()["tool_motion_active"])
        self.assertEqual([], self.tool.move_history)

    def test_continuing_settlement_cannot_extend_absolute_watchdog(self):
        self.controller.move_tool(15, 300)
        # The position remains near target but changes enough to prevent a
        # stable interval. The absolute move deadline still ends the attempt.
        for index in range(39):
            self.clock.advance(0.2)
            self.holding_at(15 if index % 2 else 18)
            self.controller.enforce_watchdog()
            self.assertTrue(self.controller.tool_motion_active)
        self.clock.advance(0.21)
        self.assertTrue(self.controller.enforce_watchdog())
        self.assertEqual("tool-move-timeout", self.controller.last_stop_reason)
        self.assertFalse(self.controller.tool_motion_active)


class ProtocolTests(unittest.TestCase):
    def test_ping_and_status(self):
        controller, motors = make_controller()
        motors["left"].position = 123

        ping = handle_request(controller, {"command": "ping"})
        status = handle_request(controller, {"command": "status"})

        self.assertEqual("pong", ping["message"])
        self.assertEqual(6, ping["protocol_version"])
        self.assertEqual(123, status["motors"]["left"]["position"])
        self.assertEqual(500, status["watchdog_timeout_ms"])

    def test_drive_requires_both_track_speeds(self):
        controller, _ = make_controller()

        with self.assertRaises(ProtocolError):
            handle_request(controller, {"command": "drive", "left": 10})

    def test_raw_camera_speed_is_rejected(self):
        controller, motors = make_controller()

        with self.assertRaisesRegex(ProtocolError, "raw tool speed"):
            handle_request(
                controller,
                {"command": "drive", "left": 0, "right": 0, "tool": 1},
            )

        self.assertEqual(0, motors["tool"].speed)

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
            controller, {"command": "tool_move", "position": 15, "speed": 40}
        )

        self.assertEqual({"position": 15, "speed": 40}, response["applied"])
        self.assertEqual(15, motors["tool"].target)
        self.assertEqual([0, 0], [motors["left"].speed, motors["right"].speed])
        self.assertTrue(controller.tool_motion_active)
        with self.assertRaises(ProtocolError):
            controller.drive(30, 30)
        with self.assertRaises(ProtocolError):
            controller.move_tool(181, 40)
        with self.assertRaisesRegex(ProtocolError, "no more than 15"):
            controller.move_tool(16, 40)

    def test_tool_move_has_independent_timeout(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        controller.zero_tool()
        controller.move_tool(-15, 30)

        clock.advance(7.99)
        self.assertFalse(controller.enforce_watchdog())
        clock.advance(0.02)
        self.assertTrue(controller.enforce_watchdog())

        self.assertEqual(0, motors["tool"].speed)
        self.assertEqual(4, motors["tool"].hold_count)
        self.assertEqual("tool-move-timeout", controller.last_stop_reason)

    def test_lifting_stall_retries_same_target_at_speed_limit_then_holds(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        controller.zero_tool()
        controller.move_tool(-15, 300)

        clock.advance(0.99)
        controller.enforce_watchdog()
        self.assertEqual([(-15, 300)], motors["tool"].move_history)

        clock.advance(0.02)
        controller.enforce_watchdog()
        self.assertEqual([(-15, 300), (-15, 1500)], motors["tool"].move_history)
        self.assertEqual(1, controller.tool_stall_retry_count)
        self.assertTrue(controller.tool_motion_active)

        clock.advance(1.01)
        controller.enforce_watchdog()
        self.assertFalse(controller.tool_motion_active)
        self.assertEqual("tool-stalled-after-retry", controller.last_stop_reason)
        self.assertEqual(0, motors["tool"].speed)

    def test_downward_stall_retry_does_not_use_gravity_assisted_boost(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        motors["tool"].position = -15
        controller.zero_tool()
        motors["tool"].position = -15
        controller.move_tool(0, 300)

        clock.advance(1.01)
        controller.enforce_watchdog()

        self.assertEqual([(0, 300), (0, 300)], motors["tool"].move_history)

    def test_same_target_manual_retry_does_not_extend_absolute_timeout(self):
        clock = FakeClock()
        controller, motors = make_controller(clock=clock)
        controller.zero_tool()
        controller.move_tool(-15, 300)
        started_at = controller.tool_move_started_at

        clock.advance(0.5)
        controller.move_tool(-15, 1500)

        self.assertEqual(started_at, controller.tool_move_started_at)
        self.assertEqual(1, controller.tool_stall_retry_count)
        self.assertEqual([(-15, 300), (-15, 1500)], motors["tool"].move_history)

    def test_new_target_cannot_stack_while_tool_is_moving(self):
        controller, _ = make_controller()
        controller.zero_tool()
        controller.move_tool(-15, 300)

        with self.assertRaisesRegex(ProtocolError, "current target"):
            controller.move_tool(-10, 300)

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
        self.assertEqual(3, motors["tool"].hold_count)
        self.assertEqual("tool-home-complete", controller.last_stop_reason)

    def test_loaded_camera_can_home_at_calibration_speed(self):
        controller, motors = make_controller()

        self.assertEqual(
            {"direction": 1, "speed": 60}, controller.home_tool(60)
        )
        self.assertEqual(60, motors["tool"].speed)
        self.assertEqual(
            {"direction": 1, "speed": 101}, controller.home_tool(101)
        )

    def test_tool_zero_requires_all_motion_stopped(self):
        controller, motors = make_controller()
        motors["tool"].position = 123

        self.assertEqual(0, controller.zero_tool())
        self.assertEqual(0, motors["tool"].position)
        self.assertEqual(3, motors["tool"].hold_count)
        controller.drive(30, 30)
        with self.assertRaises(ProtocolError):
            controller.zero_tool()

    def test_tool_position_can_be_acknowledged_after_restart_without_rezeroing(self):
        controller, motors = make_controller()
        motors["tool"].position = 63

        response = handle_request(
            controller, {"command": "tool_acknowledge_position"}
        )

        self.assertEqual(63, response["position"])
        self.assertEqual(63, motors["tool"].position)
        self.assertTrue(controller.tool_homed)
        self.assertEqual("tool-position-acknowledged", controller.last_stop_reason)

    def test_tool_position_acknowledgement_requires_stopped_motors(self):
        controller, _ = make_controller()
        controller.drive(30, 30)

        with self.assertRaises(ProtocolError):
            controller.acknowledge_tool_position()

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
        controller.drive(100, 100)

        response = process_request_line(controller, b"not-json")

        self.assertEqual("error", response["status"])
        self.assertEqual([0, 0, 0], [motors[name].speed for name in motors])
        self.assertEqual("protocol-error", controller.last_stop_reason)


if __name__ == "__main__":
    unittest.main()
