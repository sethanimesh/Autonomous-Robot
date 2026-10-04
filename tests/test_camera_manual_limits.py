import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from robot.jetson.ev3_bridge.camera_head import CameraHeadController, CameraHeadError
from tests.test_camera_head import FakeClient


class ManualLimitTests(unittest.TestCase):
    def setUp(self):
        self.client = FakeClient(position=0)
        self.head = self.make_head()

    def make_head(self):
        return CameraHeadController(self.client, minimum_position=-50, maximum_position=3,
                                    forward_position=-20, down_position=0, up_position=-47,
                                    lower_target_margin=3, upper_target_margin=3,
                                    require_approved_reference=True,
                                    approved_reference_id='test-reference', settle_tolerance=3)

    def save(self, kind, position, **overrides):
        self.client.position = position
        request = dict(action='save_manual_limit', limit=kind, expected_position=position,
                       expected_reference_id='test-reference', request_id='save-'+kind)
        request.update(overrides)
        return self.head.execute(json.dumps(request))

    def test_two_saves_never_move_or_zero_and_keep_margin(self):
        self.save('lower', 0)
        with self.assertRaises(CameraHeadError):
            self.head.jog_to(-5)
        self.save('upper', -40)
        self.assertEqual([], self.client.commands)
        self.assertEqual((-37, -3), (self.head.minimum_target_position, self.head.maximum_target_position))
        self.assertEqual(-20, self.head.forward_position)
        self.assertTrue(self.head.calibrated)
        self.assertIsNone(self.head.manual_reference_id)
        self.assertTrue(self.head.describe(self.client.status())['limit_save_result']['ok'])

    def test_normal_search_and_manual_moves_preserve_operator_setup(self):
        self.save('lower', 0)
        self.save('upper', -40)
        self.head.jog_to(-35)
        self.head.observe_status(self.client.status())
        self.head.manual_jog('test-reference', -35, -30)
        self.assertTrue(self.head.calibrated)
        self.assertEqual('operator_limits', self.head.calibration_source)
        self.head.invalidate_calibration()
        self.head.execute({'action': 'use_saved_limits'})
        self.assertTrue(self.head.calibrated)
        self.assertEqual([('move', -35, self.head.speed), ('move', -30, self.head.speed)], self.client.commands)

    def test_live_lower_endpoint_allows_settling_inside_saved_boundary(self):
        self.head.lower_target_margin = 6
        self.head.settle_tolerance = 4
        self.save('lower', 39)
        self.save('upper', -5)
        self.assertEqual(33, self.head.down_position)
        for position in (33, 37, 39):
            self.client.position = position
            self.assertIsNone(self.head.observe_status(self.client.status()))
            self.assertTrue(self.head.calibrated)
        self.client.position = 40
        self.assertIn('lower physical boundary', self.head.observe_status(self.client.status()))
        self.assertFalse(self.head.calibrated)
        self.assertEqual(39, self.head.saved_limits['lower'])

    def test_saved_limits_cannot_restore_changed_reference_or_unapproved_pose(self):
        self.save('lower', 0)
        self.save('upper', -40)
        original = self.client.status()
        for change in (dict(tool_reference_id='new'), dict(tool_homed=False),
                       dict(tool_motion_active=True), dict(motors={'tool': {'position': 4}})):
            with self.subTest(change=change), patch.object(self.client, 'status', return_value=dict(original, **change)):
                with self.assertRaises(CameraHeadError):
                    self.head.use_saved_limits()
        self.assertEqual([], self.client.commands)

    def test_upper_requires_lower_and_rejects_reversed_or_tiny_range(self):
        with self.assertRaisesRegex(CameraHeadError, 'lower limit first'):
            self.save('upper', -40)
        self.save('lower', 0)
        for position in (5, -5, -15):
            with self.assertRaises(CameraHeadError):
                self.save('upper', position)
        self.assertIsNone(self.head.saved_limits['upper'])
        self.assertFalse(self.head.limit_save_result['ok'])

    def test_new_lower_discards_previous_upper(self):
        self.save('lower', 0)
        self.save('upper', -40)
        self.save('lower', 2)
        self.assertEqual(2, self.head.saved_limits['lower'])
        self.assertIsNone(self.head.saved_limits['upper'])
        self.assertIsNone(self.head.approved_reference_id)

    def test_manual_buttons_stay_unrestricted_and_return_inside_restores_setup(self):
        self.save('lower', 0)
        self.save('upper', -40)
        self.head.manual_jog('test-reference', -40, -45)
        self.assertIsNotNone(self.head.manual_reference_id)
        self.head.manual_jog('test-reference', -45, -35)
        self.assertIsNone(self.head.manual_reference_id)
        self.assertEqual(-40, self.head.saved_limits['upper'])
        self.assertTrue(self.head.calibrated)

    def test_limits_survive_service_restart_but_wrong_boot_cannot_move(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'limits.json'
            self.head.load_manual_limits(path)
            self.save('lower', 0)
            self.save('upper', -40)
            restored = self.make_head()
            restored.load_manual_limits(path)
            self.assertEqual(self.head.saved_limits, restored.saved_limits)
            self.assertEqual(-37, restored.minimum_target_position)
            status = dict(self.client.status(), tool_reference_id='new-boot')
            with patch.object(self.client, 'status', return_value=status):
                with self.assertRaises(CameraHeadError):
                    restored.jog_to(-35)
            self.assertEqual([], self.client.commands)

    def test_stale_reference_position_or_moving_state_does_not_save(self):
        for change in (dict(expected_reference_id='old'), dict(expected_position=99)):
            with self.assertRaises(CameraHeadError):
                self.save('lower', 0, **change)
        with patch.object(self.client, 'status', return_value=dict(self.client.status(), tool_motion_active=True)):
            with self.assertRaises(CameraHeadError):
                self.save('lower', 0)
        self.assertIsNone(self.head.saved_limits)

    def test_disk_failure_does_not_report_success_or_change_approved_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            self.head.load_manual_limits(Path(folder) / 'limits.json')
            with patch('robot.jetson.ev3_bridge.camera_head.os.replace', side_effect=OSError('disk full')):
                with self.assertRaisesRegex(CameraHeadError, 'disk full'):
                    self.save('lower', 0)
            self.assertIsNone(self.head.saved_limits)
            self.assertFalse(self.head.limit_save_result['ok'])
            self.assertEqual('test-reference', self.head.approved_reference_id)

class ManualLimitEndpointTests(unittest.TestCase):
    def test_ui_save_waits_for_controller_ack_and_surfaces_rejection(self):
        import ast
        import contextlib
        import threading
        import time
        import types
        import uuid
        from unittest.mock import Mock
        source = Path('robot/jetson/perception/enrollment_console.py').read_text()
        tree = ast.parse(source)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'EnrollmentNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'save_camera_limit')
        namespace = dict(time=time, uuid=uuid, json=json, String=types.SimpleNamespace,
                         camera_control_lease=lambda **kw: contextlib.nullcontext(),
                         CameraControlBusy=type('CameraControlBusy', (RuntimeError,), {}))
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<endpoint>', 'exec'), namespace)
        client = FakeClient(0)
        head = CameraHeadController(client, settle_tolerance=3, lower_target_margin=3,
                                    upper_target_margin=3)
        ui = types.SimpleNamespace(action_lock=threading.Lock(), lock=threading.Lock(),
            mission=types.SimpleNamespace(stop=Mock()), manual_drive=types.SimpleNamespace(stop=Mock()),
            publish_chassis_stop=Mock(), runtime_safety=lambda: (True, True, False),
            camera_head_status_time=time.monotonic(), camera_head_status=head.describe(client.status()),
            status=lambda: {'saved': True})
        callbacks = []
        def deliver(message):
            # Bridge saves immediately, but console acknowledgement must run
            # behind its watchdog on the single-threaded ROS executor.
            try:
                head.execute(message.data)
            except CameraHeadError:
                pass
            result = head.describe(client.status())
            def ros_callbacks():
                with ui.action_lock:  # watchdog executes before head status
                    pass
                with ui.lock:
                    ui.camera_head_status = result
            callback = threading.Thread(target=ros_callbacks, daemon=True)
            callbacks.append(callback)
            callback.start()
        ui.camera_head_command_publisher = types.SimpleNamespace(publish=deliver)
        save = namespace['save_camera_limit']
        with self.assertRaisesRegex(ValueError, 'lower limit first'):
            save(ui, {'limit': 'upper'})
        self.assertEqual({'saved': True}, save(ui, {'limit': 'lower'}))
        self.assertEqual('Lower camera limit saved.', ui.last_message)
        client.position = -40
        ui.camera_head_status = head.describe(client.status())
        self.assertEqual({'saved': True}, save(ui, {'limit': 'upper'}))
        self.assertEqual(-40, head.saved_limits['upper'])
        self.assertEqual([], client.commands)
        for callback in callbacks:
            callback.join(timeout=1)
            self.assertFalse(callback.is_alive())


if __name__ == '__main__':
    unittest.main()
