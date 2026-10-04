"""Search retention behavior without ROS, model inference, or motors."""
import ast
import json
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest

from robot.jetson.mission.autonomous_find import target_is_at_standoff
from robot.jetson.mission.bounded_target_scan import (
    ScanError, chassis_motion_active, continuity_pose, continuity_pose_changed,
    next_tilt_toward, target_is_confirmed,
)
from robot.jetson.perception.person_continuity_stream import PersonContinuityStream


class SearchTrackingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.report = {}
        wanted = {'wait_for_tracked_face', 'reset_continuity', 'update_continuity_pose',
                  'offer_continuity', 'on_target', 'look_at_search_position', 'recover_feedback'}
        tree = ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        methods = [m for m in ast.walk(tree) if isinstance(m, ast.FunctionDef) and m.name in wanted]
        self.scope = dict(time=SimpleNamespace(monotonic=lambda: self.now), report=self.report,
                          continuity_pose=continuity_pose, continuity_pose_changed=continuity_pose_changed,
                          chassis_motion_active=chassis_motion_active, ScanError=ScanError,
                          next_tilt_toward=next_tilt_toward, String=SimpleNamespace, json=json,
                          uuid4=lambda:SimpleNamespace(hex='test-move'))
        self.rclpy = SimpleNamespace(spin_once=lambda node, timeout_sec: self.spin(timeout_sec))
        self.scope['rclpy'] = self.rclpy
        exec(compile(ast.Module(body=methods, type_ignores=[]), '<search-tracking>', 'exec'), self.scope)
        self.resets, self.offers, self.waits = [], [], []
        self.hint = {'state': 'body_continuity', 'face_age_seconds': .2,
                     'body_box': [.1, .1, .8, .9]}
        self.visible = False
        self.node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now*1e9))),
            continuity=SimpleNamespace(retained=lambda now: self.hint,
                                       reset=lambda now: self.resets.append(now),
                                       offer=lambda *args: self.offers.append(args),
                                       tracker=SimpleNamespace(maximum_face_age_seconds=10.)),
            target_is_visible=lambda: self.visible,
            motion_reason=lambda: None,
            safety_ready=lambda: True, safety_reason=lambda: None,
            parse=lambda message: json.loads(message.data),
            continuity_identity=('person', 'first'), continuity_pose_baseline=None,
            robot={'track_motion_active': False, 'motors': {'left': {'position': 100}, 'right': {'position': 200}}},
            head_status={'moving': False, 'homing': False, 'position': 10, 'reference_id': 'ref'},
        )
        for name in wanted:
            setattr(self.node, name, MethodType(self.scope[name], self.node))

        def wait(seconds):
            self.waits.append(seconds)
            self.now += seconds
            return self.visible
        self.node.wait_for_target = wait

    def spin(self, seconds):
        self.waits.append(seconds)
        self.now += seconds

    def test_no_track_adds_no_wait_to_normal_search(self):
        self.hint = None
        self.assertFalse(self.node.wait_for_tracked_face())
        self.assertFalse(self.waits)

    def test_fresh_face_returns_immediately_without_waiting_out_timeout(self):
        self.visible = True
        self.assertTrue(self.node.wait_for_tracked_face())
        self.assertFalse(self.waits)

    def test_body_retention_wait_finishes_as_soon_as_face_returns(self):
        self.node.target_is_visible = lambda: self.now >= 100.2
        self.assertTrue(self.node.wait_for_tracked_face())
        self.assertLess(self.now, 100.4)
        self.assertEqual('face_returned', self.report['tracking_holds'][0]['result'])

    def test_continually_refreshed_hint_cannot_extend_this_search_wait_forever(self):
        self.assertFalse(self.node.wait_for_tracked_face())
        self.assertLessEqual(self.now, 110.)
        self.assertEqual('face_not_returned', self.report['tracking_holds'][0]['result'])

    def test_brief_missing_body_holds_without_inventing_a_position(self):
        self.hint = {'state': 'temporarily_missing', 'face_age_seconds': .3}
        def spin(node, timeout_sec):
            self.now += timeout_sec
            self.hint = None
        self.rclpy.spin_once = spin
        self.assertFalse(self.node.wait_for_tracked_face())
        self.assertLess(self.now, 100.2)
        self.assertNotIn('target_observation', self.report)

    def test_bad_camera_exits_without_entering_long_recovery_wait(self):
        self.node.safety_ready = lambda: False
        self.node.safety_reason = lambda: 'camera is not streaming'
        with self.assertRaisesRegex(ScanError, 'camera is not streaming'):
            self.node.wait_for_tracked_face()
        self.assertFalse(self.waits)

    def test_late_callback_cannot_report_success_after_hold_deadline(self):
        def late_spin(node, timeout_sec):
            self.now += 11
            self.visible = True
        self.rclpy.spin_once = late_spin
        self.assertFalse(self.node.wait_for_tracked_face())
        self.assertEqual('face_not_returned', self.report['tracking_holds'][0]['result'])

    def test_body_hint_is_neither_target_confirmation_nor_face_standoff(self):
        self.assertFalse(target_is_confirmed(self.hint))
        self.assertFalse(target_is_at_standoff({'target_observation': self.hint}))

    def test_stationary_callbacks_do_not_continually_reset_pending_frames(self):
        for _ in range(5):
            self.node.update_continuity_pose()
        self.assertEqual([], self.resets)

    def test_accumulated_encoder_motion_resets_with_source_clock_barrier(self):
        self.node.update_continuity_pose()
        for position in (101, 102, 103):
            self.node.robot['motors']['left']['position'] = position
            self.node.update_continuity_pose()
        self.assertFalse(self.resets)
        self.node.robot['motors']['left']['position'] = 104
        self.node.update_continuity_pose()
        self.assertEqual([100.], self.resets)

    def test_head_reference_change_resets_even_at_same_position(self):
        self.node.update_continuity_pose()
        self.node.head_status['reference_id'] = 'new-reference'
        self.node.update_continuity_pose()
        self.assertEqual([100.], self.resets)

    def test_stopped_head_ack_clears_movement_frames_before_visual_verification(self):
        # The camera can complete a small move between stopped telemetry
        # samples. Its two-count change also falls below the pose-change gate.
        self.node.continuity = PersonContinuityStream()
        self.node.head_status.update(
            minimum_target_position=-14, maximum_target_position=43,
            minimum_position=-17, maximum_position=43,
        )
        self.node.update_continuity_pose()
        self.node.head_at = self.now
        self.node.frame_sequence = 7
        self.node.frame_fingerprint = object()
        self.node.ensure_homed = lambda: None
        commands, verified = [], []
        self.node.head = SimpleNamespace(publish=lambda message: commands.append(json.loads(message.data)))
        movement_frame = SimpleNamespace(header=SimpleNamespace(
            frame_id='camera', stamp=SimpleNamespace(sec=100, nanosec=100_000_000)))

        def acknowledge(predicate, timeout, reason):
            self.assertEqual(100., self.node.continuity.not_before)
            self.now = 100.1
            self.node.offer_continuity('image', movement_frame)
            self.assertTrue(self.node.continuity.cache['image'])
            self.now = 100.2
            self.node.head_at = self.now
            self.node.head_status.update(position=12, target_position=15,
                                         move_result=dict(request_id='test-move',ok=True))
            self.assertFalse(self.node.head_status['moving'])
            self.node.update_continuity_pose()
            self.assertTrue(self.node.continuity.cache['image'])
            self.assertTrue(predicate())

        def verify(sequence, frame):
            verified.append(sequence)
            self.assertIs(frame, self.node.frame_fingerprint)
            self.assertEqual(100.2, self.node.continuity.not_before)
            self.assertTrue(all(not cache for cache in self.node.continuity.cache.values()))
            # A callback captured during movement cannot repopulate the cache
            # even when it is delivered after the stopped acknowledgment.
            self.node.offer_continuity('image', movement_frame)
            self.assertFalse(self.node.continuity.cache['image'])

        self.node.spin_until = acknowledge
        self.node.verify_head_view_change = verify
        self.node.look_at_search_position(15)
        self.assertEqual([{'action': 'move_to', 'target': 15, 'request_id': 'test-move'}], commands)
        self.assertEqual([7], verified)
        self.assertEqual([12], self.report['camera_tilt_positions'])

    def test_motion_or_stale_status_prevents_frame_seeding(self):
        self.node.motion_reason = lambda: 'camera head is moving'
        self.node.offer_continuity('image', object())
        self.assertEqual([100.], self.resets)
        self.assertEqual([], self.offers)

    def test_target_reenrollment_resets_even_when_label_is_unchanged(self):
        payload = {'target_label': 'person', 'target_revision': 'second'}
        self.node.on_target(SimpleNamespace(data=json.dumps(payload)))
        self.assertEqual([100.], self.resets)
        self.assertEqual(payload, self.node.target)
        self.node.on_target(SimpleNamespace(data=json.dumps(payload)))
        self.assertEqual([100.], self.resets)


if __name__ == '__main__':
    unittest.main()
