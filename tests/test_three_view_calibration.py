import unittest
from unittest.mock import Mock
from robot.jetson.mission.camera_head_calibration import (
    CameraCalibrationError, use_operator_limits, calibrate_three_views, recover_cloud_room, three_view_targets,
)


class SimulatedHead:
    def __init__(self, offsets=(1, -2, -1), overhead_middle=False):
        self.head = dict(minimum_target_position=-41, forward_position=-18)
        self.floor_position = -3
        self.moves = []
        self.offsets = iter(offsets)
        self.cloud_calls = 0
        self.overhead_middle = overhead_middle

    def move_to(self, target):
        self.moves.append(target)
        self.position = target + next(self.offsets)

    def observe_scene(self):
        return dict(position=self.position, target_position=self.last_view_target,
                    view_usable=True, frame_sha256=str(self.position),
                    cloud_view_role='floor', cloud_advice_id='old-template')

    def query(self, samples):
        self.cloud_calls += 1
        middle = 'ceiling' if self.overhead_middle and self.cloud_calls == 1 else 'room'
        return dict(ok=True, provider='groq', advice_id='new-advice', reference_id='test',
                    frame_sha256=[s['frame_sha256'] for s in samples],
                    positions=[s['position'] for s in samples], upper_verified=middle == 'room',
                    observations=[dict(view='floor_room', floor_visible='yes', ceiling_visible='no'),
                                  dict(view=middle, floor_visible='no', ceiling_visible='yes'),
                                  dict(view='ceiling', floor_visible='no', ceiling_visible='yes')])

    def recover_room(self, samples, context):
        def capture(target):
            self.move_to(target)
            self.last_view_target = target
            return self.observe_scene()
        return recover_cloud_room(samples, 'test', self.query, capture, -41, -3, [])


class ThreeViewCalibrationTests(unittest.TestCase):
    def test_operator_limits_need_no_cloud_or_motor_move(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from tests.test_camera_head import FakeClient
        from robot.jetson.ev3_bridge.camera_head import CameraHeadController
        client = FakeClient(0)
        head = CameraHeadController(client)
        for kind, position in [('lower', 0), ('upper', -40)]:
            client.position = position
            head.save_manual_limit(dict(limit=kind, expected_position=position,
                                        expected_reference_id='test-reference'))
        node = SimpleNamespace(head=head.describe(client.status()), head_at=1,
                               frame_fingerprint=object())
        def command(value):
            head.execute(value)
            node.head = head.describe(client.status())
            node.head_at += 1
        node.command_head = command
        node.spin_until = lambda predicate, *args: self.assertTrue(predicate())
        report = {}
        # A bright plain wall is enough to reuse physical operator endpoints.
        with patch('robot.jetson.mission.camera_head_calibration.fingerprint_quality',
                   return_value={'usable': False, 'mean_luma': 140}):
            self.assertTrue(use_operator_limits(node, report))
            self.assertEqual('operator_limits', report['calibration_method'])
            self.assertEqual(0, report['cloud_calls'])
            self.assertFalse(report['route_verified'])
            self.assertEqual([], client.commands)
            node.head['saved_limits'] = dict(node.head['saved_limits'], reference_id='old')
            with self.assertRaises(CameraCalibrationError):
                use_operator_limits(node, {})

    def test_three_captures_one_cloud_call_accept_settling_and_replace_old_labels(self):
        node = SimulatedHead()
        report = {}
        result = calibrate_three_views(node, -48, report)
        self.assertEqual([-3, -18, -41], node.moves)
        self.assertEqual(1, node.cloud_calls)
        self.assertEqual(-18, result['forward_position'])
        self.assertEqual(3, len(report['samples']))
        self.assertTrue(report['calibration_readiness']['verified'])

    def test_one_middle_adjustment_does_not_repeat_endpoints(self):
        node = SimulatedHead(offsets=(1, 0, -1, 1), overhead_middle=True)
        result = calibrate_three_views(node, -48, {})
        self.assertEqual([-3, -18, -41, -13], node.moves)
        self.assertEqual(2, node.cloud_calls)
        self.assertEqual(-13, result['forward_position'])

    def test_failure_does_not_continue_to_upper_or_cloud(self):
        node = SimulatedHead()
        node.move_to = Mock(side_effect=CameraCalibrationError('motor stalled'))
        with self.assertRaisesRegex(CameraCalibrationError, 'stalled'):
            calibrate_three_views(node, -48, {})
        node.move_to.assert_called_once_with(-3)
        self.assertEqual(0, node.cloud_calls)

    def test_outside_settling_margin_rejected_before_cloud(self):
        node = SimulatedHead(offsets=(4,))
        with self.assertRaises(CameraCalibrationError):
            calibrate_three_views(node, -48, {})
        self.assertEqual(0, node.cloud_calls)

    def test_bad_saved_middle_uses_interior_and_upper_never_widens(self):
        self.assertEqual([-3, -22, -41], three_view_targets(
            dict(minimum_target_position=-41, forward_position=100), -3, -100))
        with self.assertRaises(CameraCalibrationError):
            three_view_targets(dict(minimum_target_position=-8), -3, -100)

    def test_default_upper_uses_shifted_encoder_reference_after_reboot(self):
        self.assertEqual([-109, -117, -125], three_view_targets(
            dict(minimum_target_position=-125, forward_position=-117), -109, None))


if __name__ == '__main__':
    unittest.main()
