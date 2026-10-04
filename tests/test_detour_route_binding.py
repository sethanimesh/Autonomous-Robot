"""Replay the actual route request method without ROS or motors."""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from robot.jetson.navigation.closed_loop_detour import (
    DetourError, angle_delta, validate_route_reasoning,
)
from robot.jetson.mission.bounded_target_scan import chassis_motion_active
from robot.jetson.mission.recovery_policy import same_motor_references
from scripts.diagnostics.simulate_navigation_reasoning import route_advice


class DetourRouteBindingTests(unittest.TestCase):
    def run_request(self, fault=None):
        method = next(n for n in ast.walk(ast.parse(Path(
            'robot/jetson/navigation/closed_loop_detour.py').read_text()))
            if isinstance(n, ast.FunctionDef) and n.name == 'checked_route_once')
        stops = []
        status = dict(track_motion_active=False, motors={name: dict(generation=name, position=0, speed=0, commanded_speed=0)
                             for name in ('left', 'right', 'tool')})
        node = NS(x=0., y=0., yaw=0., robot_status=status, head_status=dict(reference_id='ref'),
                  wait_route_feedback=lambda: None, publish_velocity=lambda: stops.append(True))
        value = dict(frame_sha256='fresh', camera_received_at_unix=100., result_age_seconds=.1,
            camera_head=dict(reference_id='ref'), decision=dict(blocked=False, heading_degrees=0),
            route_reasoning=dict(provider='gemini', advisory_only=True, source_frame_sha256='old',
                fresh_frame_sha256='fresh', approved_headings=[0], interpretation=route_advice(),
                scene_recheck=dict(stable=True)))
        def answer():
            if fault == 'translation':node.x = .02
            if fault == 'rotation':node.yaw = math.radians(5)
            if fault == 'moving':node.robot_status = dict(track_motion_active=True)
            if fault == 'reference':
                node.robot_status = copy.deepcopy(status)
                node.robot_status['motors']['left']['generation'] = 'new'
            if fault == 'head':node.head_status['reference_id'] = 'new'
            if fault == 'late':value['camera_received_at_unix'] = 98.
            if fault == 'future':value['camera_received_at_unix'] = 102.
            if fault == 'nan':value['result_age_seconds'] = float('nan')
            if fault == 'old_service':value.pop('route_reasoning')
            return value
        class Pool:
            def __init__(self, **kwargs):pass
            def __enter__(self):return self
            def __exit__(self, *args):return False
            def submit(self, *args):return NS(done=lambda: True, result=answer)
        scope = dict(ThreadPoolExecutor=Pool, args=NS(route_url='unused', maximum_result_age=1.),
            fetch_route=lambda *args: None, time=NS(time=lambda: 100.5), math=math,
            rclpy=NS(spin_once=lambda *args, **kwargs: None), DetourError=DetourError,
            same_motor_references=same_motor_references, chassis_motion_active=chassis_motion_active,
            angle_delta=angle_delta, validate_route_reasoning=validate_route_reasoning)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<route-request>', 'exec'), scope)
        return scope['checked_route_once'](node), stops

    def test_delivery_age_is_added_to_still_fresh_route(self):
        value, stops = self.run_request()
        self.assertEqual(.5, value['result_age_seconds'])
        self.assertEqual([], stops)

    def test_robot_change_late_results_and_old_server_cannot_authorize_motion(self):
        for fault in ('translation', 'rotation', 'moving', 'reference', 'head', 'late', 'future', 'nan', 'old_service'):
            with self.subTest(fault=fault), self.assertRaises(DetourError):self.run_request(fault)

    def test_stopped_feedback_gap_recovers_but_outage_and_motion_do_not(self):
        method = next(n for n in ast.walk(ast.parse(Path(
            'robot/jetson/navigation/closed_loop_detour.py').read_text()))
            if isinstance(n, ast.FunctionDef) and n.name == 'wait_route_feedback')
        for fault in (None, 'outage', 'motion', 'head'):
            clock, stops = [0.], []
            def reason():
                if clock[0] < 3. or fault == 'outage':return 'robot feedback is stale'
                return 'camera left its floor view' if fault == 'head' else None
            node = NS(motion_reason=reason, robot_status=dict(track_motion_active=fault == 'motion'),
                      publish_velocity=lambda: stops.append(clock[0]))
            def spin(*args, **kwargs):clock[0] += .1
            scope = dict(time=NS(monotonic=lambda: clock[0]), rclpy=NS(spin_once=spin),
                         DetourError=DetourError, chassis_motion_active=chassis_motion_active)
            exec(compile(ast.Module(body=[method], type_ignores=[]), '<feedback-recovery>', 'exec'), scope)
            with self.subTest(fault=fault):
                if fault:
                    with self.assertRaises(DetourError):scope['wait_route_feedback'](node)
                else:
                    scope['wait_route_feedback'](node)
                    self.assertLess(clock[0], 3.2)
                self.assertTrue(stops)
                self.assertLess(clock[0], 8.2)


if __name__ == '__main__':unittest.main()
