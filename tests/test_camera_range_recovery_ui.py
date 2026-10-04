"""One-point reboot recovery UI and acknowledgement flow, without ROS."""

import ast
import contextlib
import json
from pathlib import Path
import shutil
import subprocess
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path('robot/jetson/perception/enrollment_console.py').read_text()
PAGE = SOURCE.split('PAGE = r"""', 1)[1].split('""".encode("utf-8")', 1)[0]


class CameraRangeRecoveryEndpointTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.published, self.waited = [], []
        self.acknowledge = True
        self.rejection = None
        tree = ast.parse(SOURCE)
        node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'EnrollmentNode')
        methods = [method for method in node.body if getattr(method, 'name', None)
                   in ('save_camera_limit', 'restore_saved_camera_range')]
        definition = ast.ClassDef(name='Console', bases=[], keywords=[], body=methods, decorator_list=[])
        module = ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[]))
        scope = dict(time=SimpleNamespace(monotonic=lambda: self.now, sleep=self.sleep),
                     json=json, String=SimpleNamespace,
                     uuid=SimpleNamespace(uuid4=lambda: SimpleNamespace(hex='new-request')),
                     camera_control_lease=lambda **kwargs: contextlib.nullcontext(),
                     CameraControlBusy=type('CameraControlBusy', (RuntimeError,), {}))
        exec(compile(module, '<camera-recovery-ui>', 'exec'), scope)
        self.node = scope['Console']()
        self.node.action_lock, self.node.lock = threading.Lock(), threading.Lock()
        self.node.mission = SimpleNamespace(stop=Mock())
        self.node.manual_drive = SimpleNamespace(stop=Mock())
        self.node.publish_chassis_stop = Mock()
        self.node.runtime_safety = lambda: (True, True, False)
        self.node.camera_head_status_time = 99.9
        self.node.camera_head_status = {
            'position': 5, 'reference_id': 'new-reference', 'moving': False, 'homing': False,
            'saved_limits': {'lower': 43, 'upper': -17, 'reference_id': 'old-reference'},
            'limit_save_result': {'request_id': 'previous-request', 'ok': True},
        }
        self.node.camera_head_command_publisher = SimpleNamespace(
            publish=lambda message: self.published.append(json.loads(message.data)))
        self.node.status = lambda: {'result': self.node.last_message}

    def sleep(self, seconds):
        self.waited.append(seconds)
        self.now += seconds
        # Status delivery must remain possible behind the ROS watchdog while
        # the HTTP action waits for this particular controller acknowledgement.
        self.assertTrue(self.node.action_lock.acquire(blocking=False))
        self.node.action_lock.release()
        if self.acknowledge:
            self.node.camera_head_status['limit_save_result'] = {
                'request_id': 'new-request', 'ok': self.rejection is None,
                'error': self.rejection,
            }

    def test_restore_uses_fresh_server_position_and_waits_for_matching_ack(self):
        result = self.node.restore_saved_camera_range({
            'operator_confirmed': True, 'expected_reference_id': 'spoofed',
            'expected_position': -999,
        })
        self.assertEqual([{
            'action': 'restore_saved_range_from_lower', 'operator_confirmed': True,
            'expected_reference_id': 'new-reference', 'expected_position': 5,
            'request_id': 'new-request',
        }], self.published)
        self.assertTrue(self.waited)
        self.assertEqual('Saved camera range restored from this lower view.', result['result'])

    def test_restore_requires_explicit_lower_view_confirmation(self):
        for value in (None, False, 1, 'true'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'same saved lower view'):
                self.node.restore_saved_camera_range({'operator_confirmed': value})
        self.assertFalse(self.published)
        self.node.mission.stop.assert_not_called()

    def test_restore_rejects_missing_upper_limit_or_unchanged_reference(self):
        for changes in ({'upper': None}, {'lower': True}, {'upper': 44}, {'reference_id': 'new-reference'}):
            with self.subTest(changes=changes):
                self.node.camera_head_status['saved_limits'] = dict(
                    lower=43, upper=-17, reference_id='old-reference')
                self.node.camera_head_status['saved_limits'].update(changes)
                with self.assertRaises(ValueError):
                    self.node.restore_saved_camera_range({'operator_confirmed': True})
        self.assertFalse(self.published)

    def test_stale_or_moving_feedback_cannot_reanchor_the_range(self):
        self.node.camera_head_status_time = 97.
        with self.assertRaisesRegex(ValueError, 'live camera and EV3'):
            self.node.restore_saved_camera_range({'operator_confirmed': True})
        self.node.camera_head_status_time = 99.9
        self.node.camera_head_status['moving'] = True
        with self.assertRaisesRegex(ValueError, 'camera and tracks to stop'):
            self.node.restore_saved_camera_range({'operator_confirmed': True})
        self.assertFalse(self.published)

    def test_controller_rejection_is_returned_instead_of_success(self):
        self.rejection = 'Camera moved while confirming the lower view.'
        with self.assertRaisesRegex(ValueError, self.rejection):
            self.node.restore_saved_camera_range({'operator_confirmed': True})
        self.assertTrue(self.waited)

    def test_unrelated_acknowledgement_never_counts_as_restore_success(self):
        self.acknowledge = False
        with self.assertRaisesRegex(ValueError, 'confirmation has not arrived'):
            self.node.restore_saved_camera_range({'operator_confirmed': True})
        self.assertEqual(1, len(self.published))
        self.assertGreaterEqual(self.now, 105.)


class CameraRangeRecoveryPageTests(unittest.TestCase):
    def test_action_label_and_route_explicitly_confirm_the_lower_view(self):
        self.assertIn('Use saved range from this lower view', PAGE)
        self.assertIn('same lowest view you saved', PAGE)
        self.assertIn("post('/api/camera/restore-range',JSON.stringify({operator_confirmed:true}))", PAGE)
        self.assertIn('"/api/camera/restore-range": lambda: self.node.restore_saved_camera_range(request)', SOURCE)

    @unittest.skipUnless(shutil.which('node'), 'JavaScript runtime unavailable')
    def test_recovery_visibility_and_enablement_do_not_restrict_manual_buttons(self):
        function = 'function updateCameraLimits(h,s){' + PAGE.split(
            'function updateCameraLimits(h,s){', 1)[1].split('\nasync function saveCameraLimit', 1)[0]
        script = """
const elements = {};
const document = {getElementById: id => elements[id] ||= {disabled:false}};
function setText(id, value) {document.getElementById(id).textContent=value;}
let savingCameraLimit = false;
""" + function + """
const ready = {robot_ready:true, camera_ready:true};
const head = {available:true, reference_id:'new', moving:false, homing:false,
              saved_limits:{reference_id:'old', lower:43, upper:-17}};
const states = [];
for (const change of [{}, {saved_limits:{reference_id:'old',lower:43,upper:null}},
                      {reference_id:'old'}, {moving:true}, {available:false}]) {
    updateCameraLimits({...head,...change}, ready);
    states.push({hidden:elements.cameraRangeRecovery.hidden,
                 disabled:elements.restoreCameraRange.disabled,
                 manualDisabled:document.getElementById('tiltDown').disabled});
}
console.log(JSON.stringify(states));
"""
        result = subprocess.run([shutil.which('node'), '-e', script], text=True,
                                capture_output=True, check=True, timeout=5)
        self.assertEqual([
            {'hidden': False, 'disabled': False, 'manualDisabled': False},
            {'hidden': True, 'disabled': True, 'manualDisabled': False},
            {'hidden': True, 'disabled': True, 'manualDisabled': False},
            {'hidden': False, 'disabled': True, 'manualDisabled': False},
            {'hidden': False, 'disabled': True, 'manualDisabled': False},
        ], json.loads(result.stdout))


if __name__ == '__main__':
    unittest.main()
