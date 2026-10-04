import ast
import contextlib
import io
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from robot.jetson.camera.camera_recovery import CameraRestartWatchdog
from robot.jetson.mission.bounded_target_scan import ScanError, parse_args
from robot.jetson.mission.recovery_policy import same_motor_references
from robot.jetson.navigation.cable_guard import record_cable_turn


def method(name, scope):
    tree = ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), '<camera-drop>', 'exec'), scope)
    return scope[name]


class CameraDropRecoveryTests(unittest.TestCase):
    def test_stalled_capture_requests_managed_restart_once(self):
        now, last = [100.], [100.]
        exit_process = Mock()
        watch = CameraRestartWatchdog(lambda:last[0], clock=lambda:now[0], exit_process=exit_process)
        now[0] = 111.9
        self.assertFalse(watch.check())
        now[0] = 112.
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(watch.check())
        self.assertFalse(watch.check())
        exit_process.assert_called_once_with(75)

    def test_brief_drop_recovers_without_service_restart_and_shutdown_disarms(self):
        now, last = [100.], [None]
        exit_process = Mock()
        watch = CameraRestartWatchdog(lambda:last[0], clock=lambda:now[0], exit_process=exit_process)
        now[0] = 108.
        self.assertFalse(watch.check())
        last[0] = 108.
        now[0] = 118.
        self.assertFalse(watch.check())
        watch.close()
        now[0] = 150.
        self.assertFalse(watch.check())
        exit_process.assert_not_called()

    def test_recovery_waits_for_new_frames_and_preserves_scan_position(self):
        for changed in (False, True):
            with self.subTest(changed_reference=changed):
                report, order = {}, []
                node = NS(robot={'motors':{'left':{'generation':'left'},'right':{'generation':'right'}}},
                          head_status={'reference_id':'head'}, frame_sequence=10,
                          stop=lambda:order.append('stop'), reset_continuity=lambda:order.append('clear-old-face'),
                          wait_for_camera_after_head_move=lambda:order.append('wait-camera'),
                          safety_ready=lambda:True, yaw=.2)
                def wait(predicate, timeout, reason):
                    self.assertFalse(predicate())
                    node.frame_sequence += 3
                    if changed:node.head_status={'reference_id':'replacement'}
                    self.assertTrue(predicate())
                node.spin_until = wait
                recover = method('recover_camera', dict(time=NS(monotonic=lambda:100.), report=report,
                    args=NS(camera_recovery_seconds=30.), ScanError=ScanError,
                    same_motor_references=same_motor_references))
                if changed:
                    with self.assertRaisesRegex(ScanError, 'reference changed'):
                        recover(node, 'live camera frame heartbeat is stale')
                else:
                    self.assertTrue(recover(node, 'live camera frame heartbeat is stale'))
                    self.assertTrue(report['camera_recoveries'][0]['continued_same_step'])
                    self.assertEqual(.2, node.yaw)
                self.assertEqual(['stop', 'clear-old-face', 'wait-camera'], order)

    def test_drop_during_turn_continues_original_target_instead_of_new_scan(self):
        now, commands, report = [0.], [], {}
        node = NS(yaw=0., yaw_at=0., head_status={'reference_id':'head'},
                  robot={'motors':{'left':{'generation':'left'},'right':{'generation':'right'}}},
                  safety_ready=lambda:True, motion_ready=lambda:True,
                  reset_continuity=lambda:None, recover_feedback=lambda reason:False)
        dropped = [False]
        def spin(node, **kwargs):
            now[0] += .1
            node.yaw_at = now[0]
            if dropped[0]:node.yaw = -math.radians(29.)
        node.safety_reason = lambda **kw:None if dropped[0] else 'live camera frame heartbeat is stale'
        def recover(reason):
            node.stop()
            now[0] += 17.  # Camera watchdog, managed restart, warm-up.
            node.yaw = -math.radians(10.)
            dropped[0] = True
            return True
        node.recover_camera = recover
        node.stop = lambda:commands.append(0.)
        node.velocity = commands.append
        def wait(predicate, timeout, reason):
            now[0] += .1
            node.yaw_at = now[0]
            self.assertTrue(predicate())
        node.spin_until = wait
        turn = method('turn_relative', dict(time=NS(monotonic=lambda:now[0]), math=math,
            rclpy=NS(spin_once=spin), report=report, ScanError=ScanError,
            same_motor_references=same_motor_references,
            args=NS(turn_timeout=8., turn_speed=.6, yaw_tolerance_degrees=3.)))
        self.assertAlmostEqual(29., turn(node, 30.))
        self.assertTrue(dropped[0])
        self.assertEqual([0., 0.], commands)

    def test_recorded_live_overshoot_continues_with_actual_heading(self):
        args = parse_args([])
        self.assertEqual(25., args.maximum_turn_error_degrees)
        self.assertAlmostEqual(-40.8442645311,
            record_cable_turn(-6.807377421846326, -23.192622578153674, -34.03688710923164,
                              90., args.maximum_turn_error_degrees, 10.))


if __name__ == '__main__':
    unittest.main()
