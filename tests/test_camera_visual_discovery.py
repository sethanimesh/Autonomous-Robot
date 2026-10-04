"""Automatic setup scenarios; the same coordinator is used on the live EV3."""
import io
import json
import unittest
from urllib.error import HTTPError
from unittest.mock import patch

from robot.jetson.mission.camera_visual_setup import (
    discovery_setup_range, run_visual_discovery, request_setup_view, SetupStateChanged,
    SetupUnavailable, SetupRateLimited, SetupRetryLater)
from robot.jetson.mission.camera_head_calibration import parse_args, dry_run_report
from robot.mac.camera_setup_advisor import setup_view_decision
from tests.test_camera_visual_setup import view
from tests.test_camera_manual_limits import ManualLimitTests


class DiscoveryCamera:
    def __init__(self, start=5, settling=0):
        self.head = dict(position=start, reference_id='new-boot', homed=False,
                         lower_target_margin=6, upper_target_margin=3, settle_tolerance=4)
        self.origin = ('new-boot', start)
        self.moves, self.saved = [], []
        self.settling = settling
        self.override = None

    def setup_range(self):
        return discovery_setup_range(self.head, self.origin)

    def setup_prepare_discovery(self):
        self.head['homed'] = True

    def setup_ready(self):
        self.setup_range()

    def setup_move(self, target):
        _, low, high = self.setup_range()
        assert low <= target <= high
        position = self.head['position']
        self.moves.append(target)
        self.head['position'] = target - (self.settling if target > position else -self.settling)
        self.head['homed'] = True

    def setup_observe(self):
        position = self.head['position']
        obs = self.override() if self.override else view(
            scene='floor_only' if position > 65 else 'ceiling_only' if position < 0 else
                  'floor_room' if position >= 30 else 'room')
        return dict(position=position, reference_id=self.head['reference_id'],
                    decisions={goal: setup_view_decision(obs, goal) for goal in ('lower', 'upper')})

    def setup_save_discovery(self, chosen):
        self.saved.append(chosen)
        self.head.update(down_position=chosen['lower'] - 6, up_position=chosen['upper'] + 3,
                         forward_position=(chosen['lower'] + chosen['upper']) // 2)


class DiscoveryScenarios(unittest.TestCase):
    def test_new_boot_upper_floor_and_ceiling_start_automate_both_endpoints(self):
        for start in (5, 45, -5, 75):
            with self.subTest(start=start):
                camera = DiscoveryCamera(start)
                report = run_visual_discovery(camera, {})
                self.assertEqual('calibrated', report['outcome'])
                self.assertTrue(report['return_views_verified'])
                self.assertFalse(report['physical_limits_changed'])
                self.assertEqual(1, len(camera.saved))

    def test_four_count_settling_and_incorrect_saved_range_do_not_block_discovery(self):
        camera = DiscoveryCamera(settling=4)
        camera.head.update(minimum_position=-43, maximum_position=0,
                           manual_override=True, approved_reference_id='old-boot')
        report = run_visual_discovery(camera, {})
        self.assertEqual('calibrated', report['outcome'])
        self.assertGreater(report['observed_endpoints']['lower'], 0)

    def test_transient_cloud_failure_retries_automatically(self):
        camera = DiscoveryCamera()
        original = camera.setup_observe
        failures = [True]
        def flaky():
            if failures:
                failures.pop()
                raise SetupUnavailable('quota or timeout')
            return original()
        camera.setup_observe = flaky
        self.assertEqual('calibrated', run_visual_discovery(camera, {})['outcome'])

    def test_rate_limit_waits_and_resumes_at_the_same_pose(self):
        camera = DiscoveryCamera()
        original = camera.setup_observe
        calls, pauses = [], []
        def limited():
            calls.append(camera.head['position'])
            if len(calls) == 1:
                raise SetupRateLimited({'x-ratelimit-reset-tokens': '1.5s'})
            return original()
        camera.setup_observe = limited
        camera.setup_pause = lambda seconds: pauses.append((seconds, camera.head['position']))
        report = run_visual_discovery(camera, {})
        self.assertEqual('calibrated', report['outcome'])
        self.assertEqual([(2.5, 5)], pauses)
        self.assertEqual([5, 5], calls[:2])

    def test_rate_limit_duration_accepts_provider_formats(self):
        self.assertEqual(60, SetupRateLimited({'x-ratelimit-reset-tokens': '1m2s'}).retry_after)
        self.assertEqual(4, SetupRateLimited({'retry-after': '3'}).retry_after)

    def test_service_cooldown_retains_discovered_views_and_retries_same_pose(self):
        camera = DiscoveryCamera()
        original = camera.setup_observe
        calls, pauses = [], []
        failed = False
        def flaky_return():
            nonlocal failed
            calls.append(camera.head['position'])
            if camera.head['position'] == 29 and not failed:
                failed = True
                raise SetupRetryLater({'retry-after': '15'})
            return original()
        camera.setup_observe = flaky_return
        camera.setup_pause = lambda seconds: pauses.append((seconds, camera.head['position']))
        report = run_visual_discovery(camera, {})
        self.assertEqual('calibrated', report['outcome'])
        self.assertEqual([(16, 29)], pauses)
        retry_index = calls.index(29)
        self.assertEqual([29, 29], calls[retry_index:retry_index+2])
        self.assertEqual(1, len(camera.saved))

    def test_service_failure_preserves_provider_retry_delay(self):
        for status in (502, 503, 504):
            with self.subTest(status=status):
                error = HTTPError('http://localhost/camera-setup', status, 'Unavailable', {},
                                  io.BytesIO(json.dumps({'quota': {'retry-after': '15'}}).encode()))
                with patch('robot.jetson.mission.camera_visual_setup.urlopen', side_effect=error):
                    with self.assertRaises(SetupRetryLater) as raised:
                        request_setup_view(b'jpeg', 'http://localhost/camera-setup')
                self.assertNotIsInstance(raised.exception, SetupRateLimited)
                self.assertEqual(16, raised.exception.retry_after)

    def test_covered_camera_does_not_guess_direction_or_erase_limits(self):
        camera = DiscoveryCamera()
        camera.override = lambda: view(quality='dark', scene='unknown')
        with self.assertRaises(SetupUnavailable):
            run_visual_discovery(camera, {})
        self.assertEqual([], camera.moves)
        self.assertEqual([], camera.saved)

    def test_reference_change_during_cloud_response_never_commits(self):
        camera = DiscoveryCamera()
        original = camera.setup_observe
        def changed():
            result = original()
            camera.head['reference_id'] = 'reboot'
            return result
        camera.setup_observe = changed
        with self.assertRaises(SetupStateChanged):
            run_visual_discovery(camera, {})
        self.assertEqual([], camera.saved)

    def test_no_floor_stops_at_search_budget_without_repeating_manual_checks(self):
        camera = DiscoveryCamera()
        camera.override = lambda: view(scene='room')
        with self.assertRaises(SetupUnavailable):
            run_visual_discovery(camera, {})
        self.assertLessEqual(max(camera.moves), 65)
        self.assertEqual([], camera.saved)

    def test_cli_needs_no_restore_option_and_preview_never_moves(self):
        result = dry_run_report(parse_args(['--discover-views']))
        self.assertTrue(result['discovery'])
        self.assertEqual(10, result['step_counts'])

    def test_normal_mission_uses_automatic_setup(self):
        from robot.jetson.mission.autonomous_find import calibration_command, parse_args as mission_args
        self.assertIn('--auto-setup', calibration_command(mission_args([]), '/tmp/report.json'))
        self.assertTrue(parse_args(['--auto-setup']).visual_setup)


class VisualLimitBridgeTests(ManualLimitTests):
    def test_new_boot_acknowledges_without_zeroing_or_movement(self):
        self.client.position = 7
        original = self.client.status
        state = dict(tool_homed=False, tool_reference_id='boot')
        self.client.status = lambda: dict(original(), **state)
        def acknowledge():
            self.client.commands.append(('acknowledge_position',))
            state.update(tool_homed=True, tool_reference_id='acknowledged')
            return {'position': 7}
        self.client.acknowledge_tool_position = acknowledge
        result = self.head.execute(dict(action='begin_visual_setup', request_id='start',
                                        expected_reference_id='boot', expected_position=7))
        self.assertEqual('acknowledged', result['reference_id'])
        self.assertEqual('boot', result['previous_reference_id'])
        self.assertEqual([('acknowledge_position',)], self.client.commands)
        self.assertEqual(7, self.client.position)

    def test_visual_pair_persists_atomically_with_truthful_source(self):
        self.client.position = 34
        request = dict(action='save_visual_limits', request_id='visual',
                       expected_reference_id='test-reference', expected_position=34,
                       lower=40, upper=0)
        self.head.execute(request)
        self.assertEqual('cloud_useful_views', self.head.calibration_source)
        self.assertEqual([], self.client.commands)
        old = dict(self.head.saved_limits)
        with patch.object(self.head, '_persist_manual_limits', side_effect=OSError('disk full')):
            with self.assertRaises(Exception):
                self.head.execute(dict(request, lower=45))
        self.assertEqual(old, self.head.saved_limits)

    def test_setup_step_uses_normal_speed_outside_obsolete_limits(self):
        self.client.position = 4
        self.head.execute(dict(action='setup_jog', expected_reference_id='test-reference',
                               expected_position=4, target=14))
        self.assertEqual([('move', 14, self.head.speed)], self.client.commands)
        with self.assertRaises(Exception):
            self.head.execute(dict(action='setup_jog', expected_reference_id='old-boot',
                                   expected_position=14, target=24))

    def test_discovery_does_not_reactivate_obsolete_limits_when_passing_inside_them(self):
        self.save('lower', 0)
        self.save('upper', -40)
        self.client.position = -20
        self.head.execute(dict(action='setup_jog', expected_reference_id='test-reference',
                               expected_position=-20, target=-10))
        self.head.describe(self.client.status())
        self.assertFalse(self.head.calibrated)


if __name__ == '__main__':
    unittest.main()
