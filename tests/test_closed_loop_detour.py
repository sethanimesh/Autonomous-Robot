import io
import unittest
from contextlib import redirect_stderr

from robot.jetson.navigation.closed_loop_detour import DetourError
from robot.jetson.navigation.closed_loop_detour import center_floor_fraction
from robot.jetson.navigation.closed_loop_detour import parse_args, straight_step_distance
from robot.jetson.navigation.closed_loop_detour import validate_route_result
from robot.jetson.navigation.closed_loop_detour import recheck_uncertain_floor
from unittest.mock import Mock
from unittest.mock import patch
from urllib.error import HTTPError
from robot.jetson.navigation.closed_loop_detour import fetch_route
from robot.jetson.navigation.closed_loop_detour import reusable_straight_route
from robot.jetson.navigation.closed_loop_detour import validate_route_reasoning
from scripts.diagnostics.simulate_navigation_reasoning import route_advice


class ClosedLoopDetourTests(unittest.TestCase):
    def combined_result(self):
        value = self.valid_result()
        value['frame_sha256'] = 'fresh'
        value['evidence'][1]['floor_fraction'] = .99
        value['route_reasoning'] = dict(provider='gemini', advisory_only=True,
            source_frame_sha256='previous', fresh_frame_sha256='fresh',
            approved_headings=[-30], interpretation=route_advice(), scene_recheck=dict(stable=True))
        return value

    def test_side_approval_cannot_authorize_straight_drive_after_turn(self):
        value = self.combined_result()
        validate_route_reasoning(value)
        with self.assertRaisesRegex(DetourError, 'straight corridor'):
            straight_step_distance(value, 1., .95, .05)
        value['route_reasoning']['approved_headings'].append(0)
        self.assertEqual(.05, straight_step_distance(value, 1., .95, .05))

    def test_mixed_deployment_unbound_and_changed_scene_results_cannot_move(self):
        with self.assertRaises(DetourError):validate_route_reasoning(self.valid_result())
        for field, value in (('fresh_frame_sha256', 'wrong'), ('source_frame_sha256', 'fresh'),
                             ('scene_recheck', dict(stable=False)), ('advisory_only', False),
                             ('approved_headings', [0]), ('interpretation', {})):
            result = self.combined_result();result['route_reasoning'][field] = value
            with self.subTest(field=field), self.assertRaises(DetourError):validate_route_reasoning(result)

    def test_straight_route_reuse_requires_fresh_unchanged_view_and_valid_result(self):
        value = self.valid_result()
        value['camera_received_at_unix'] = 100.
        result = reusable_straight_route(value, 0., 0., 1., 100.4)
        self.assertAlmostEqual(.4, result['result_age_seconds'])
        self.assertEqual(.02, value['result_age_seconds'])
        self.assertIsNone(reusable_straight_route(value, 15., 15., 1., 100.4))
        self.assertIsNone(reusable_straight_route(value, 0., 0., 1., 101.1))
        value['camera_head']['moving'] = True
        self.assertIsNone(reusable_straight_route(value, 0., 0., 1., 100.4))

    def test_transient_service_error_gets_one_fresh_request_and_preserves_reason(self):
        error = HTTPError('http://route', 503, 'unavailable', {},
                          io.BytesIO(b'{"error":"camera head changed during inference"}'))
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.read.return_value = b'{"ok":true}'
        with patch('robot.jetson.navigation.closed_loop_detour.urlopen', side_effect=[error,response]) as request:
            result = fetch_route('http://route')
        self.assertEqual(2, request.call_count)
        self.assertEqual('camera head changed during inference', result['service_recheck']['error'])

    def test_repeated_service_error_stops_with_the_actual_reason(self):
        errors = [HTTPError('http://route',503,'unavailable',{},io.BytesIO(b'{"error":"camera unavailable"}')) for _ in range(2)]
        with patch('robot.jetson.navigation.closed_loop_detour.urlopen', side_effect=errors) as request:
            with self.assertRaisesRegex(DetourError,'camera unavailable'):
                fetch_route('http://route')
        self.assertEqual(2,request.call_count)

    def test_floor_feedback_timeout_reports_actual_head_failure(self):
        import ast
        import math
        from pathlib import Path
        from types import MethodType, SimpleNamespace
        tree = ast.parse(Path('robot/jetson/navigation/closed_loop_detour.py').read_text())
        methods = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                   and n.name in ('spin_until', 'motion_reason')]
        now = [0.0]
        def spin(node, timeout_sec):
            now[0] += timeout_sec
        scope = dict(time=SimpleNamespace(monotonic=lambda: now[0]), math=math,
                     rclpy=SimpleNamespace(spin_once=spin), DetourError=DetourError)
        exec(compile(ast.Module(body=methods, type_ignores=[]), '<floor-feedback>', 'exec'), scope)
        node = SimpleNamespace(robot_status_at=0., head_status_at=0., odom_at=0.,
                               frame_at=0., frame_usable=True,
                               head_status={'homed': True, 'calibrated': False,
                                            'calibration_error': 'Camera head crossed the approved lower physical boundary'})
        node.motion_reason = MethodType(scope['motion_reason'], node)
        with self.assertRaisesRegex(DetourError, 'crossed the approved lower physical boundary'):
            scope['spin_until'](node, lambda: node.motion_reason() is None, .15, node.motion_reason)

    def valid_result(self):
        return {
            "ok": True,
            "result_age_seconds": 0.02,
            "decision": {
                "blocked": False,
                "heading_degrees": -30,
                "distance_m": 0.10,
            },
            "camera_head": {
                "available": True,
                "homed": True,
                "calibrated": True,
                "moving": False,
                "homing": False,
                "position": 19,
                "down_position": 17,
                "reference_id": "test",
                "approved_reference_id": "test",
            },
            "evidence": [
                {"heading_degrees": -30, "floor_fraction": 0.97},
                {"heading_degrees": 0, "floor_fraction": 0.93, "known_fraction": 0.99, "sample_count": 100},
            ],
        }

    def test_valid_bounded_decision(self):
        self.assertEqual(validate_route_result(self.valid_result()), (-30.0, 0.1))

    def uncertain_result(self):
        value = self.valid_result()
        value['frame_sha256'] = 'first'
        value['decision'].update(blocked=True, reason='no proven footprint-wide route')
        value['evidence'][1].update(floor_fraction=1., known_fraction=.70)
        return value

    def test_single_fresh_floor_recheck_can_recover_uncertainty(self):
        second = self.valid_result()
        second['frame_sha256'] = 'second'
        fetch = Mock(side_effect=[self.uncertain_result(), second])
        self.assertIs(second, recheck_uncertain_floor(fetch, 1.))
        self.assertEqual(2, fetch.call_count)
        self.assertTrue(second['uncertainty_recheck']['decision']['blocked'])

    def test_recheck_rejects_duplicate_stale_and_still_blocked_results(self):
        for failure in ('duplicate', 'stale', 'blocked'):
            second = self.valid_result()
            second['frame_sha256'] = 'first' if failure == 'duplicate' else 'second'
            if failure == 'stale': second['result_age_seconds'] = 2.
            if failure == 'blocked': second['decision']['blocked'] = True
            fetch = Mock(side_effect=[self.uncertain_result(), second])
            with self.subTest(failure=failure), self.assertRaises(DetourError):
                recheck_uncertain_floor(fetch, 1.)
            self.assertEqual(2, fetch.call_count)

    def test_visible_obstacle_is_not_retried(self):
        first = self.uncertain_result()
        first['evidence'][1]['floor_fraction'] = .7
        fetch = Mock(return_value=first)
        self.assertIs(first, recheck_uncertain_floor(fetch, 1.))
        self.assertEqual(1, fetch.call_count)

    def test_relaxed_coverage_still_requires_floor_and_caps_short_step(self):
        value = self.valid_result()
        value['evidence'][1].update(floor_fraction=1., known_fraction=.7636)
        self.assertEqual(.05, straight_step_distance(value, 1., .95, .05))
        value['evidence'][1]['known_fraction'] = .749
        with self.assertRaises(DetourError):
            straight_step_distance(value, 1., .95, .05)

    def test_stale_result_is_rejected(self):
        value = self.valid_result()
        value["result_age_seconds"] = 2.0
        with self.assertRaisesRegex(DetourError, "stale"):
            validate_route_result(value)

    def test_blocked_result_is_rejected(self):
        value = self.valid_result()
        value["decision"]["blocked"] = True
        with self.assertRaisesRegex(DetourError, "blocked"):
            validate_route_result(value)

    def test_overlong_motion_is_rejected(self):
        value = self.valid_result()
        value["decision"]["distance_m"] = 0.20
        with self.assertRaisesRegex(DetourError, "distance"):
            validate_route_result(value)

    def test_unstable_camera_head_is_rejected(self):
        value = self.valid_result()
        value["camera_head"]["moving"] = True
        with self.assertRaisesRegex(DetourError, "camera head"):
            validate_route_result(value)

    def test_center_fraction(self):
        self.assertEqual(center_floor_fraction(self.valid_result()), 0.93)

    def test_forward_step_requires_known_center_and_is_capped(self):
        value = self.valid_result()
        value['evidence'][1]['floor_fraction'] = .99
        self.assertEqual(.05, straight_step_distance(value, 1, .95, .05))
        for field, bad in [('floor_fraction', float('nan')), ('known_fraction', .1), ('sample_count', 0)]:
            with self.subTest(field=field):
                invalid = self.valid_result()
                invalid['evidence'][1]['floor_fraction'] = .99
                invalid['evidence'][1][field] = bad
                with self.assertRaises(DetourError):
                    straight_step_distance(invalid, 1, .95, .05)

    def test_unapproved_reference_and_negative_age_are_rejected(self):
        value = self.valid_result()
        value['camera_head']['approved_reference_id'] = 'another'
        with self.assertRaises(DetourError):validate_route_result(value)
        value = self.valid_result();value['result_age_seconds'] = -.5
        with self.assertRaises(DetourError):validate_route_result(value)

    def test_execute_requires_physical_cable_neutral_confirmation(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--route-url", "http://route", "--execute"])
        args = parse_args(
            [
                "--route-url",
                "http://route",
                "--execute",
                "--cable-zero-confirmed",
            ]
        )
        self.assertTrue(args.execute)


if __name__ == "__main__":
    unittest.main()
