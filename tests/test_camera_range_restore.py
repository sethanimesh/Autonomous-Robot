import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot.jetson.ev3_bridge.camera_head import CameraHeadController, CameraHeadError
from tests.test_camera_head import FakeClient


class RebootedClient(FakeClient):
    def __init__(self):
        super().__init__(position=13)
        self.reference = 'new-boot'
        self.homed = True

    def status(self):
        result = super().status()
        result.update(tool_reference_id=self.reference, tool_homed=self.homed,
                      motion_active=False, track_motion_active=False)
        result['motors']['tool'].update(speed=0, state=['holding'])
        result['motors']['left'] = dict(position=0, speed=0, state=[])
        result['motors']['right'] = dict(position=0, speed=0, state=[])
        return result

    def acknowledge_tool_position(self):
        result = super().acknowledge_tool_position()
        self.reference = 'acknowledged-reference'
        self.homed = True
        return result


class RestoreSavedRangeTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.path = Path(folder.name) / 'camera-limits.json'
        self.previous = dict(version=1, reference_id='previous-boot', lower=39, upper=-5)
        self.path.write_text(json.dumps(self.previous))
        self.client = RebootedClient()
        self.head = CameraHeadController(
            self.client, minimum_position=-5, maximum_position=39,
            forward_position=16, down_position=33, up_position=-2,
            lower_target_margin=6, upper_target_margin=3,
            require_approved_reference=True, settle_tolerance=4,
            sleeper=lambda seconds: None)
        self.head.load_manual_limits(self.path)

    def request(self, **overrides):
        request = dict(action='restore_saved_range_from_lower',
                       expected_reference_id=self.client.reference,
                       expected_position=self.client.position,
                       request_id='restore-1', operator_confirmed=True)
        request.update(overrides)
        return request

    def restore(self, **overrides):
        return self.head.execute(self.request(**overrides))

    def assert_previous_limits_preserved(self):
        self.assertEqual(self.previous, self.head.saved_limits)
        self.assertEqual(self.previous, json.loads(self.path.read_text()))

    def test_confirmed_lower_translates_saved_span_and_preserves_current_margins(self):
        result = self.restore()
        self.assertEqual(dict(request_id='restore-1', ok=True,
                              limit='restore_saved_range_from_lower'), result)
        expected = dict(version=1, reference_id='new-boot', lower=13, upper=-31)
        self.assertEqual(expected, self.head.saved_limits)
        self.assertEqual(expected, json.loads(self.path.read_text()))
        self.assertEqual((6, 3), (self.head.lower_target_margin, self.head.upper_target_margin))
        self.assertEqual((7, -28), (self.head.down_position, self.head.up_position))
        self.assertTrue(self.head.calibrated)
        self.assertEqual('operator_limits', self.head.calibration_source)
        self.assertIsNone(self.head.manual_reference_id)
        self.assertEqual(result, self.head.describe(self.client.status())['limit_save_result'])
        self.assertEqual([], self.client.commands)

    def test_unhomed_encoder_is_acknowledged_without_movement_or_zero(self):
        self.client.homed = False
        self.restore()
        self.assertEqual([('acknowledge_position',)], self.client.commands)
        self.assertEqual(13, self.client.position)
        self.assertEqual('acknowledged-reference', self.head.saved_limits['reference_id'])
        self.assertEqual(44, self.head.maximum_position - self.head.minimum_position)
        self.assertTrue(self.head.calibrated)

    def test_loading_old_range_does_not_automatically_bind_new_boot(self):
        self.head.describe(self.client.status())
        self.assert_previous_limits_preserved()
        self.assertFalse(self.head.calibrated)
        self.assertEqual([], self.client.commands)

    def test_idle_disconnect_recovers_only_after_matching_stopped_feedback(self):
        self.client.reference = 'previous-boot'
        self.head.suspend_for_status_disconnect()
        self.head.suspend_for_status_disconnect()
        self.assertFalse(self.head.calibrated)
        moving = self.client.status()
        moving['tool_motion_active'] = True
        self.head.observe_status(moving)
        self.assertFalse(self.head.calibrated)
        self.head.observe_status(self.client.status())
        self.assertTrue(self.head.calibrated)
        self.assertEqual('operator_limits', self.head.calibration_source)
        self.assert_previous_limits_preserved()
        self.assertEqual([], self.client.commands)

    def test_disconnect_recovery_cannot_survive_changed_reference_or_boundary_violation(self):
        for change in ('reference', 'boundary', 'explicit_invalidation'):
            with self.subTest(change=change):
                self.client.reference = 'previous-boot'
                self.client.position = 13
                self.head._apply_manual_limits(self.previous)
                self.head.suspend_for_status_disconnect()
                if change == 'reference':
                    self.client.reference = 'new-boot'
                elif change == 'boundary':
                    self.client.position = 40
                else:
                    self.head.invalidate_calibration()
                self.head.observe_status(self.client.status())
                self.assertFalse(self.head.calibrated)
                self.client.reference = 'previous-boot'
                self.client.position = 13
                self.head.observe_status(self.client.status())
                self.assertFalse(self.head.calibrated)

    def test_disconnect_does_not_restore_explicitly_invalidated_limits(self):
        self.client.reference = 'previous-boot'
        self.head.invalidate_calibration()
        self.head.suspend_for_status_disconnect()
        self.head.observe_status(self.client.status())
        self.assertFalse(self.head.calibrated)

    def test_confirmation_and_request_identity_are_required(self):
        for overrides in (dict(operator_confirmed=False), dict(operator_confirmed=1),
                          dict(operator_confirmed='true'), dict(request_id=None),
                          dict(expected_reference_id='')):
            with self.subTest(overrides=overrides), self.assertRaises(CameraHeadError):
                self.restore(**overrides)
            self.assert_previous_limits_preserved()
            self.assertFalse(self.head.limit_save_result['ok'])
        self.assertEqual([], self.client.commands)

    def test_prior_complete_valid_range_is_required(self):
        for limits in (None, dict(self.previous, upper=None),
                       dict(self.previous, upper=35), dict(self.previous, version=0)):
            self.head.saved_limits = limits
            with self.subTest(limits=limits), self.assertRaises(CameraHeadError):
                self.restore()
            self.assertEqual(limits, self.head.saved_limits)
            self.assertEqual(self.previous, json.loads(self.path.read_text()))
        self.assertEqual([], self.client.commands)

    def test_stale_expected_reference_or_position_cannot_restore(self):
        for overrides in (dict(expected_reference_id='stale'), dict(expected_position=12)):
            with self.subTest(overrides=overrides), self.assertRaises(CameraHeadError):
                self.restore(**overrides)
            self.assert_previous_limits_preserved()
        self.assertEqual([], self.client.commands)

    def test_translation_cannot_shrink_span_to_fit_brick_position_limit(self):
        self.client.homed = False
        status = dict(self.client.status(), tool_position_limit=30)
        with patch.object(self.client, 'status', return_value=status):
            with self.assertRaisesRegex(CameraHeadError, 'supported encoder range'):
                self.restore()
        self.assert_previous_limits_preserved()
        self.assertEqual([], self.client.commands)

    def test_changed_reference_or_position_between_checks_is_rejected(self):
        request = self.request()
        for change in ('reference', 'position', 'motion'):
            before = self.client.status()
            after = self.client.status()
            if change == 'reference':
                after['tool_reference_id'] = 'another-boot'
            elif change == 'position':
                after['motors']['tool']['position'] += 1
            else:
                after['track_motion_active'] = True
            with self.subTest(change=change), patch.object(
                    self.client, 'status', side_effect=[before, after]):
                with self.assertRaises(CameraHeadError):
                    self.head.execute(request)
            self.assert_previous_limits_preserved()
        self.assertEqual([], self.client.commands)

    def test_motion_flags_or_actual_motor_motion_reject_restoration(self):
        for key in ('tool_motion_active', 'tool_homing', 'track_motion_active', 'motion_active'):
            status = self.client.status()
            status[key] = True
            with self.subTest(key=key), patch.object(self.client, 'status', return_value=status):
                with self.assertRaises(CameraHeadError):
                    self.restore()
        for role in ('tool', 'left', 'right'):
            for change in (dict(speed=1), dict(state=['running'])):
                status = self.client.status()
                status['motors'][role].update(change)
                with self.subTest(role=role, change=change), patch.object(
                        self.client, 'status', return_value=status):
                    with self.assertRaises(CameraHeadError):
                        self.restore()
        self.assert_previous_limits_preserved()
        self.assertEqual([], self.client.commands)

    def test_encoder_change_during_acknowledgement_does_not_rebind(self):
        self.client.homed = False
        def moved_acknowledgement():
            result = RebootedClient.acknowledge_tool_position(self.client)
            self.client.position += 1
            return result
        with patch.object(self.client, 'acknowledge_tool_position', side_effect=moved_acknowledgement):
            with self.assertRaisesRegex(CameraHeadError, 'position or reference changed'):
                self.restore()
        self.assert_previous_limits_preserved()
        self.assertFalse(self.head.calibrated)

    def test_reference_change_after_acknowledgement_does_not_rebind(self):
        self.client.homed = False
        before = self.client.status()
        after = self.client.status()
        after.update(tool_homed=True, tool_reference_id='acknowledged-reference')
        changed = dict(after, tool_reference_id='another-reference')
        with patch.object(self.client, 'status', side_effect=[before, before, after, changed]):
            with self.assertRaisesRegex(CameraHeadError, 'position or reference changed'):
                self.restore()
        self.assert_previous_limits_preserved()
        self.assertFalse(self.head.calibrated)

    def test_failed_acknowledgement_cannot_enable_range(self):
        self.client.homed = False
        with patch.object(self.client, 'acknowledge_tool_position', side_effect=RuntimeError('link lost')):
            with self.assertRaisesRegex(CameraHeadError, 'link lost'):
                self.restore()
        self.assert_previous_limits_preserved()
        self.assertFalse(self.head.calibrated)
        self.assertEqual(dict(request_id='restore-1', ok=False, error='link lost'),
                         self.head.limit_save_result)

    def test_disk_failure_after_acknowledgement_preserves_span_for_fresh_retry(self):
        self.client.homed = False
        with patch('robot.jetson.ev3_bridge.camera_head.os.replace', side_effect=OSError('disk full')):
            with self.assertRaisesRegex(CameraHeadError, 'disk full'):
                self.restore()
        self.assert_previous_limits_preserved()
        self.assertEqual('previous-boot', self.head.approved_reference_id)
        self.assertFalse(self.head.calibrated)
        self.assertFalse(self.head.limit_save_result['ok'])
        self.assertEqual('acknowledged-reference', self.client.reference)
        self.restore(request_id='restore-2')
        self.assertEqual([('acknowledge_position',)], self.client.commands)
        self.assertEqual(dict(version=1, reference_id='acknowledged-reference', lower=13, upper=-31),
                         json.loads(self.path.read_text()))
        self.assertTrue(self.head.calibrated)

    def test_disk_failure_without_reference_change_preserves_existing_setup(self):
        self.client.reference = 'previous-boot'
        with patch('robot.jetson.ev3_bridge.camera_head.os.replace', side_effect=OSError('disk full')):
            with self.assertRaisesRegex(CameraHeadError, 'disk full'):
                self.restore()
        self.assert_previous_limits_preserved()
        self.assertTrue(self.head.calibrated)
        self.assertEqual([], self.client.commands)


if __name__ == '__main__':
    unittest.main()
