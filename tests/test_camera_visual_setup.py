import base64
import hashlib
import io
import json
import unittest
from unittest.mock import Mock, patch

from robot.jetson.mission.camera_visual_setup import (
    SetupStateChanged, SetupUnavailable, approved_setup_range, request_setup_view,
    run_visual_setup, setup_tolerance)
from robot.jetson.mission.camera_head_calibration import parse_args, dry_run_report
from robot.mac.camera_setup_advisor import setup_view_decision
from robot.mac.route_perception import RouteService


def view(scene='floor_room', quality='clear'):
    lower = scene == 'floor_room' and quality == 'clear'
    upper = scene == 'room' and quality == 'clear'
    return dict(frame_index=1, scene=scene, quality=quality,
                near_floor_visible='yes' if lower else 'no',
                room_context_visible='yes' if lower or upper else 'no',
                lower_suitable='yes' if lower else 'no',
                upper_suitable='yes' if upper else 'no', evidence='Test view.')


class FakeSetup:
    def __init__(self, responses=None):
        self.head = dict(reference_id='ref', approved_reference_id='ref',
            require_approved_reference=True, homed=True, available=True,
            minimum_target_position=3, maximum_target_position=34,
            minimum_position=0, maximum_position=40, position=20,
            down_position=34, forward_position=18, up_position=3)
        self.responses = iter(responses) if responses is not None else None
        self.moves = []
        self.observations = 0
        self.commits = []
        self.retained = False

    def setup_ready(self):
        if self.head.get('moving') or self.head.get('homing'):
            raise SetupStateChanged('Wait for camera movement to finish')
        approved_setup_range(self.head)

    def setup_move(self, target):
        _, low, high = approved_setup_range(self.head)
        assert low <= target <= high
        self.moves.append(target)
        self.head['position'] = target

    def setup_observe(self):
        self.observations += 1
        observation = next(self.responses) if self.responses else view(
            'floor_room' if self.head['position'] >= 20 else 'room')
        if isinstance(observation, Exception):
            raise observation
        return dict(reference_id=self.head['reference_id'], position=self.head['position'],
                    observations=[observation], frame_sha256=['frame-' + str(self.observations)],
                    decisions={goal: setup_view_decision(observation, goal)
                               for goal in ('lower', 'upper')})

    def setup_commit(self, calibration):
        self.commits.append(calibration)
        self.head.update(calibration, calibrated=True)

    def setup_retain(self):
        self.retained = True


class VisualSetupTests(unittest.TestCase):
    def test_motion_wait_is_separate_from_reference_validation(self):
        node = FakeSetup()
        expected = approved_setup_range(node.head)
        node.head['moving'] = True
        self.assertEqual(expected, approved_setup_range(node.head))
        with self.assertRaisesRegex(SetupStateChanged, 'movement'):
            run_visual_setup(node, {})
        self.assertEqual([], node.moves)

    def test_accepted_settling_error_at_bound_does_not_trigger_an_extra_jog(self):
        node = FakeSetup([view('room')])
        node.head['settle_tolerance'] = 4
        original = node.setup_observe
        def offset():
            result = original()
            result['position'] -= 4
            return result
        node.setup_observe = offset
        report = run_visual_setup(node, {})
        self.assertEqual([34], node.moves)
        self.assertEqual(1, node.observations)
        self.assertEqual('deferred', report['visual_setup_outcome'])

    def test_raw_ros_status_does_not_require_ui_only_availability_field(self):
        node = FakeSetup()
        node.head.pop('available')
        report = run_visual_setup(node, {})
        self.assertEqual('selected', report['visual_setup_outcome'])
        self.assertEqual(2, node.observations)

    def test_saved_endpoints_take_two_calls_and_do_not_require_ceiling(self):
        node = FakeSetup()
        report = run_visual_setup(node, {})
        self.assertEqual(2, node.observations)
        self.assertEqual([34, 3, 34], node.moves)
        self.assertEqual('selected', report['visual_setup_outcome'])
        self.assertEqual(1, len(node.commits))
        self.assertFalse(report['physical_limits_changed'])
        self.assertFalse(report['route_verified'])

    def test_one_dark_frame_recovers_at_the_same_pose(self):
        node = FakeSetup([view('unknown', 'dark'), view(), view('room')])
        report = run_visual_setup(node, {})
        self.assertEqual('selected', report['visual_setup_outcome'])
        self.assertEqual([34, 3, 34], node.moves)
        self.assertEqual(3, node.observations)

    def test_persistent_dark_or_cloud_failure_retains_limits_without_forced_tilt(self):
        for responses in ([view('unknown', 'dark')] * 2, [SetupUnavailable('quota')]):
            with self.subTest(responses=responses):
                node = FakeSetup(responses)
                report = run_visual_setup(node, {})
                self.assertEqual('calibrated', report['outcome'])
                self.assertEqual('deferred', report['visual_setup_outcome'])
                self.assertTrue(node.retained)
                self.assertEqual([], node.commits)
                self.assertEqual([34], node.moves)

    def test_floor_only_adjusts_up_inside_range(self):
        node = FakeSetup([view('floor_only'), view(), view('room')])
        report = run_visual_setup(node, {})
        self.assertEqual([34, 29, 3, 29], node.moves)
        self.assertEqual(29, report['calibration']['down_position'])

    def test_wrong_view_at_upper_bound_does_not_move_past_it(self):
        node = FakeSetup([view(), view()])
        report = run_visual_setup(node, {})
        self.assertEqual('deferred', report['visual_setup_outcome'])
        self.assertEqual([34, 3], node.moves)
        self.assertTrue(node.retained)

    def test_reboot_cannot_rebind_or_move_using_old_limit_numbers(self):
        node = FakeSetup()
        node.head['reference_id'] = 'new-boot'
        with self.assertRaises(SetupStateChanged):
            run_visual_setup(node, {})
        self.assertEqual([], node.moves)
        self.assertEqual(0, node.observations)
        self.assertFalse(node.retained)

    def test_reference_change_during_cloud_failure_does_not_restore_old_limits(self):
        node = FakeSetup()
        def changed():
            node.head['reference_id'] = 'new-boot'
            raise SetupUnavailable('cloud timeout')
        node.setup_observe = changed
        with self.assertRaises(SetupStateChanged):
            run_visual_setup(node, {})
        self.assertEqual([34], node.moves)
        self.assertFalse(node.retained)
        self.assertEqual([], node.commits)

    def test_reference_change_after_success_cannot_commit(self):
        node = FakeSetup()
        original = node.setup_observe
        def changed():
            result = original()
            if node.observations == 2:
                node.head['reference_id'] = 'new-boot'
            return result
        node.setup_observe = changed
        with self.assertRaises(SetupStateChanged):
            run_visual_setup(node, {})
        self.assertEqual([], node.commits)
        self.assertFalse(node.retained)

    def test_small_settling_error_does_not_make_named_targets_uncommandable(self):
        node = FakeSetup()
        original = node.setup_observe
        def drift():
            result = original()
            result['position'] += 2 if node.observations == 1 else -2
            return result
        node.setup_observe = drift
        report = run_visual_setup(node, {})
        self.assertEqual(34, report['calibration']['down_position'])
        self.assertEqual(3, report['calibration']['up_position'])
        self.assertEqual(36, report['visual_setup_samples'][0]['position'])

    def test_four_count_bridge_allowance_is_used_without_changing_command_bounds(self):
        node = FakeSetup()
        node.head['settle_tolerance'] = 4
        original = node.setup_observe
        def drift():
            result = original()
            result['position'] += 4 if node.observations == 1 else 0
            return result
        node.setup_observe = drift
        report = run_visual_setup(node, {})
        self.assertEqual('selected', report['visual_setup_outcome'])
        self.assertEqual(34, report['calibration']['down_position'])
        self.assertEqual(38, report['visual_setup_samples'][0]['position'])
        self.assertEqual(3, setup_tolerance({}))
        self.assertEqual(4, setup_tolerance({'settle_tolerance': 50}))

    def test_opt_in_does_not_change_default_startup_or_dry_run_move_motors(self):
        self.assertFalse(parse_args([]).visual_setup)
        result = dry_run_report(parse_args(['--visual-setup']))
        self.assertEqual('dry_run_success', result['outcome'])
        self.assertFalse(result['mechanical_homing_attempted'])

    def test_mission_passes_visual_setup_only_when_explicitly_selected(self):
        from robot.jetson.mission.autonomous_find import calibration_command, parse_args as mission_args
        normal = calibration_command(mission_args([]), '/tmp/report.json')
        self.assertNotIn('--visual-setup', normal)
        selected = calibration_command(mission_args(['--visual-camera-setup']), '/tmp/report.json')
        self.assertIn('--visual-setup', selected)
        self.assertEqual('http://127.0.0.1:18091/camera-setup',
                         selected[selected.index('--setup-advice-url') + 1])

    def test_request_checks_image_digest_and_unique_request_id(self):
        jpeg = b'\xff\xd8' + b'x' * 100
        def response(request, timeout):
            payload = json.loads(request.data)
            self.assertEqual({'request_id', 'jpeg_base64'}, set(payload))
            self.assertEqual(jpeg, base64.b64decode(payload['jpeg_base64']))
            return io.BytesIO(json.dumps(dict(request_id=payload['request_id'],
                frame_sha256=[hashlib.sha256(jpeg).hexdigest()],
                provider='groq', advisory_only=True)).encode())
        with patch('robot.jetson.mission.camera_visual_setup.urlopen', side_effect=response):
            self.assertEqual('groq', request_setup_view(jpeg, 'http://mac/camera-setup')['provider'])
        with patch('robot.jetson.mission.camera_visual_setup.urlopen',
                   return_value=io.BytesIO(b'{"request_id":"old"}')):
            with self.assertRaises(SetupUnavailable):
                request_setup_view(jpeg, 'http://mac/camera-setup')

    def test_server_uses_only_supplied_jpeg_and_returns_advice_without_motor_interface(self):
        jpeg = b'\xff\xd8' + b'x' * 100
        request = dict(request_id='a' * 32, jpeg_base64=base64.b64encode(jpeg).decode())
        service = RouteService(Mock())
        with patch('robot.mac.camera_setup_pool.CameraSetupPool') as advisor:
            advisor.return_value.interpret.return_value = dict(observations=[view()], advisory_only=True)
            with patch('robot.mac.route_perception.fetch_snapshot') as fetch:
                result = service.setup_advice(request)
            fetch.assert_not_called()
            advisor.return_value.interpret.assert_called_once_with([jpeg])
            self.assertEqual('candidate_view', result['decisions']['lower']['action'])
            self.assertEqual(request['request_id'], result['request_id'])
            with self.assertRaises(ValueError):
                service.setup_advice(dict(request, motor_force=100))
            with self.assertRaises(ValueError):
                service.setup_advice(dict(request, jpeg_base64='invalid!'))


if __name__ == '__main__':
    unittest.main()
