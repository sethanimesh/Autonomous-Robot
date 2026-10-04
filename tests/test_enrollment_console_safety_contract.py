import ast
import copy
from pathlib import Path
import threading
import unittest


class EnrollmentConsoleSafetyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            Path(__file__).resolve().parents[1]
            / "robot"
            / "jetson"
            / "perception"
            / "enrollment_console.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(cls.source)
        node_class = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "EnrollmentNode"
        )
        cls.methods = {
            node.name: node
            for node in node_class.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

    def compile_harness(self, *method_names):
        definition = ast.ClassDef(
            name="Harness",
            bases=[],
            keywords=[],
            body=[copy.deepcopy(self.methods[name]) for name in method_names],
            decorator_list=[],
        )
        module = ast.Module(body=[definition], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {"ManualDriveError": RuntimeError}
        exec(compile(module, "<enrollment-console-harness>", "exec"), namespace)
        return namespace["Harness"]

    def test_motion_transactions_start_under_the_shared_action_lock(self):
        methods = (
            "on_manual_drive_watchdog",
            "publish_emergency_stop",
            "enable_manual_drive",
            "command_manual_drive",
            "disable_manual_drive",
            "start_find_mission",
            "stop_find_mission",
            "jog_camera",
        )
        for name in methods:
            with self.subTest(method=name):
                statements = self.methods[name].body
                if (
                    statements
                    and isinstance(statements[0], ast.Expr)
                    and isinstance(statements[0].value, ast.Constant)
                    and isinstance(statements[0].value.value, str)
                ):
                    statements = statements[1:]
                self.assertTrue(statements)
                self.assertIsInstance(statements[0], ast.With)
                context = statements[0].items[0].context_expr
                self.assertIsInstance(context, ast.Attribute)
                self.assertIsInstance(context.value, ast.Name)
                self.assertEqual("self", context.value.id)
                self.assertEqual("action_lock", context.attr)

    def test_server_mission_preflight_requires_a_ready_stopped_ev3(self):
        method_source = ast.get_source_segment(
            self.source, self.methods["start_find_mission"]
        )
        self.assertIn("camera_ready, robot_ready, robot_moving", method_source)
        self.assertIn("if not robot_ready", method_source)
        self.assertIn("if robot_moving", method_source)
        self.assertIn("Fresh EV3 status is required", method_source)

    def test_global_stop_uses_locked_emergency_publication(self):
        method_source = ast.get_source_segment(
            self.source, self.methods["_stop_find_mission_locked"]
        )
        self.assertIn("self.manual_drive.stop(disable=True)", method_source)
        self.assertIn("self._publish_emergency_stop_locked()", method_source)
        self.assertNotIn("self.publish_emergency_stop()", method_source)

    def test_global_stop_cannot_finish_before_an_inflight_drive_publish(self):
        harness_type = self.compile_harness(
            "command_manual_drive",
            "_command_manual_drive_locked",
            "stop_find_mission",
            "_stop_find_mission_locked",
        )
        command_entered = threading.Event()
        release_command = threading.Event()
        stop_attempted = threading.Event()
        stop_finished = threading.Event()
        events = []
        errors = []

        class Drive(object):
            def command(self, *_args):
                command_entered.set()
                if not release_command.wait(1.0):
                    raise RuntimeError("test command release timed out")
                return {"linear": 0.03, "angular": 0.0}

            def stop(self, disable=False, message=None):
                events.append("drive_disabled" if disable else "drive_stopped")
                return {"enabled": not disable, "message": message}

        class Mission(object):
            def status(self):
                return {"running": False}

            def stop(self):
                events.append("mission_stopped")
                return {"state": "stopping"}

        node = harness_type()
        node.action_lock = threading.Lock()
        node.manual_drive = Drive()
        node.mission = Mission()
        node.runtime_safety = lambda: (True, True, False)
        node.publish_chassis_velocity = (
            lambda linear=0.0, angular=0.0: events.append(
                ("velocity", linear, angular)
            )
        )
        node.publish_chassis_stop = lambda repeats=1: events.append(
            ("chassis_stop", repeats)
        )
        node._publish_emergency_stop_locked = lambda: events.append(
            "emergency_stop"
        )

        def run_command():
            try:
                node.command_manual_drive(
                    {"direction": "forward", "control_token": "token", "sequence": 1}
                )
            except BaseException as exc:  # pragma: no cover - assertion aid
                errors.append(exc)

        def run_stop():
            stop_attempted.set()
            try:
                node.stop_find_mission()
            except BaseException as exc:  # pragma: no cover - assertion aid
                errors.append(exc)
            finally:
                stop_finished.set()

        command_thread = threading.Thread(target=run_command)
        stop_thread = threading.Thread(target=run_stop)
        command_thread.start()
        self.assertTrue(command_entered.wait(1.0))
        stop_thread.start()
        self.assertTrue(stop_attempted.wait(1.0))
        self.assertFalse(stop_finished.wait(0.05))
        release_command.set()
        command_thread.join(1.0)
        stop_thread.join(1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual([], errors)
        self.assertEqual("emergency_stop", events[-1])

    def test_mission_completion_stop_serializes_after_drive_publication(self):
        harness_type = self.compile_harness(
            "publish_emergency_stop",
            "command_manual_drive",
            "_command_manual_drive_locked",
        )
        command_entered = threading.Event()
        release_command = threading.Event()
        stop_finished = threading.Event()
        events = []

        class Drive(object):
            def command(self, *_args):
                command_entered.set()
                if not release_command.wait(1.0):
                    raise RuntimeError("test command release timed out")
                return {"linear": 0.03, "angular": 0.0}

            def stop(self, disable=False, message=None):
                return {"enabled": not disable, "message": message}

        class Mission(object):
            def status(self):
                return {"running": False}

        node = harness_type()
        node.action_lock = threading.Lock()
        node.manual_drive = Drive()
        node.mission = Mission()
        node.runtime_safety = lambda: (True, True, False)
        node.publish_chassis_velocity = (
            lambda linear=0.0, angular=0.0: events.append(
                ("velocity", linear, angular)
            )
        )
        node.publish_chassis_stop = lambda repeats=1: None
        node._publish_emergency_stop_locked = lambda: events.append(
            "completion_stop"
        )

        command_thread = threading.Thread(
            target=lambda: node.command_manual_drive(
                {"direction": "forward", "control_token": "token", "sequence": 1}
            )
        )
        stop_thread = threading.Thread(
            target=lambda: (node.publish_emergency_stop(), stop_finished.set())
        )
        command_thread.start()
        self.assertTrue(command_entered.wait(1.0))
        stop_thread.start()
        self.assertFalse(stop_finished.wait(0.05))
        release_command.set()
        command_thread.join(1.0)
        stop_thread.join(1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual("completion_stop", events[-1])


if __name__ == "__main__":
    unittest.main()
