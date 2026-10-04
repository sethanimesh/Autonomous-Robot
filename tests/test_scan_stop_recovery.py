"""Replay delayed stop confirmation through the actual scan turn handler."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from robot.jetson.mission.bounded_target_scan import ScanError
from robot.jetson.mission.recovery_policy import same_motor_references
from robot.jetson.navigation.cable_guard import project_cable_turn, record_cable_turn, validate_measured_cable_heading


class ScanStopRecoveryTests(unittest.TestCase):
    def replay(self, error='stopped status was not confirmed', changed=False, stop_fails=False):
        tree = ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        guard = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'guarded_turn')
        wrapper = ast.parse('def replay():\n cable_heading = -12.82319932952447\n pass\n guarded_turn(-22.796890834095656)\n return cable_heading\n').body[0]
        wrapper.body[1] = guard
        report = {'turns': [], 'observed_headings': []}
        robot = {'motors': {'left': {'generation': 'L'}, 'right': {'generation': 'R'}}}
        node = NS(robot=robot, yaw=0., yaw_at=0.)
        calls = []
        def turn(*args, **kwargs):
            calls.append('turn')
            raise ScanError(error)
        def stop():
            calls.append('stop')
            if stop_fails: raise ScanError('stopped status was not confirmed')
        def wait(predicate, timeout, reason):
            node.yaw = math.radians(29.76248733272349)
            node.yaw_at = 101.
            if changed: node.robot = {'motors': {'left': {'generation': 'new'}, 'right': {'generation': 'R'}}}
            self.assertTrue(predicate())
        node.turn_relative, node.stop, node.spin_until = turn, stop, wait
        scope = dict(node=node, report=report, math=math,
                     time=NS(monotonic=lambda: 100.),
                     args=NS(cable_limit_degrees=90., cable_margin_degrees=10., maximum_turn_error_degrees=25.),
                     same_motor_references=same_motor_references,
                     project_cable_turn=project_cable_turn, record_cable_turn=record_cable_turn,
                     validate_measured_cable_heading=validate_measured_cable_heading)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper], type_ignores=[])), '<stop-recovery>', 'exec'), scope)
        heading = scope['replay']()
        return heading, report, calls

    def test_delayed_stop_confirmation_continues_same_scan_once(self):
        heading, report, calls = self.replay()
        self.assertAlmostEqual(-42.58568666224796, heading)
        self.assertEqual(['turn', 'stop'], calls)
        self.assertTrue(report['stop_confirmation_recoveries'][0]['continued_same_step'])
        self.assertEqual(1, len(report['turns']))
        self.assertEqual([heading], report['observed_headings'])
        self.assertTrue(report['cable_heading_known'])

    def test_changed_reference_or_unconfirmed_stop_cannot_resume(self):
        for options in ({'changed': True}, {'stop_fails': True}, {'error': 'chassis turned in the wrong direction'}):
            with self.subTest(options=options), self.assertRaises(ScanError):
                self.replay(**options)


if __name__ == '__main__':
    unittest.main()
