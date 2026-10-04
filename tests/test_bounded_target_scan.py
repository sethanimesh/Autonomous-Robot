import io
import unittest
from contextlib import redirect_stderr

from robot.jetson.mission.bounded_target_scan import incremental_scan_turns
from robot.jetson.mission.bounded_target_scan import chassis_motion_active
from robot.jetson.mission.bounded_target_scan import largest_body_observation
from robot.jetson.mission.bounded_target_scan import next_tilt_toward
from robot.jetson.mission.bounded_target_scan import parse_args
from robot.jetson.mission.bounded_target_scan import run
from robot.jetson.mission.bounded_target_scan import target_is_confirmed
from robot.jetson.mission.bounded_target_scan import target_centering_step, ScanError
from robot.jetson.mission.bounded_target_scan import search_view_usable
from robot.jetson.mission.bounded_target_scan import fast_search_positions
from robot.jetson.mission.bounded_target_scan import person_tilt_target
from robot.jetson.mission.bounded_target_scan import face_tilt_target, preferred_search_positions
from robot.jetson.mission.bounded_target_scan import small_lower_person_candidate
from robot.jetson.navigation.local_planner import cable_safe_scan_headings
from robot.jetson.mission.recovery_policy import same_motor_references


class BoundedTargetScanTests(unittest.TestCase):
    def test_search_waits_for_its_command_even_if_old_target_already_matches(self):
        import ast, json
        from uuid import uuid4
        from pathlib import Path
        from types import SimpleNamespace as NS
        method = next(n for n in ast.walk(ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'look_at_search_position')
        head = dict(position=-13, target_position=2, moving=False, homing=False,
                    minimum_target_position=-19, maximum_target_position=22,
                    minimum_position=-22, maximum_position=28, settle_tolerance=4)
        commands, verified = [], []
        node = NS(head_status=head, head_at=1., frame_sequence=1, frame_fingerprint={},
                  ensure_homed=lambda:None, reset_continuity=lambda:None,
                  head=NS(publish=lambda message:commands.append(json.loads(message.data))),
                  verify_head_view_change=lambda *args:verified.append(True))
        waits = []
        def wait(predicate, timeout, reason):
            waits.append(reason)
            if len(waits)==1:
                node.head_at=2.; head.update(position=2, moving=False)
                self.assertFalse(predicate(), 'Stopped feedback is not a command acknowledgement')
                head['move_result']=dict(request_id='previous',ok=True)
                self.assertFalse(predicate(), 'A previous command cannot acknowledge this move')
                head['move_result']=dict(request_id=commands[0]['request_id'],ok=True)
            self.assertTrue(predicate())
        node.spin_until=wait
        scope=dict(ScanError=ScanError,String=NS,json=json,uuid4=uuid4,report={})
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<search-command>','exec'),scope)
        scope['look_at_search_position'](node,2)
        self.assertEqual(1,len(commands)); self.assertEqual([True],verified)
        # An accepted landing does not issue another tiny correction.
        head['position']=-2
        scope['look_at_search_position'](node,2)
        self.assertEqual(1,len(commands))

    def test_feedback_recovery_requires_new_matching_feedback_and_preserves_search_state(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        method = next(n for n in ast.walk(ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'recover_feedback')
        for changed in (False, True):
            with self.subTest(changed_reference=changed):
                report, stops = {}, []
                node = SimpleNamespace(head_status={'reference_id':'head','position':13},
                    robot={'motors':{'left':{'generation':'left'},'right':{'generation':'right'}}},
                    robot_at=99.,head_at=99.,yaw_at=99.,yaw=.2,motion_ready=lambda:True,
                    stop=lambda:stops.append('stop'))
                def wait(predicate, timeout, reason):
                    self.assertFalse(predicate(), 'Old status must not authorize resumption')
                    node.robot_at=node.head_at=node.yaw_at=101.
                    self.assertTrue(predicate())
                    if changed: node.robot['motors']['left']['generation']='replacement'
                node.spin_until=wait
                scope=dict(time=SimpleNamespace(monotonic=lambda:100.),report=report,ScanError=ScanError)
                exec(compile(ast.Module(body=[method],type_ignores=[]),'<feedback-recovery>','exec'),scope)
                recover=scope['recover_feedback']
                self.assertFalse(recover(node,'camera head is not homed'))
                self.assertFalse(stops)
                if changed:
                    with self.assertRaisesRegex(ScanError,'reference changed'):
                        recover(node,'robot status is stale')
                else:
                    self.assertTrue(recover(node,'robot status is stale'))
                    node.robot_at=node.head_at=node.yaw_at=99.
                    self.assertTrue(recover(node,'robot unexpectedly reports motion'))
                    self.assertEqual(2,len(report['feedback_recoveries']))
                    self.assertEqual(13,node.head_status['position'])
                    self.assertEqual(.2,node.yaw)
                    report['feedback_recoveries'] *= 3
                    self.assertFalse(recover(node,'robot status is stale'))
                self.assertEqual(['stop'] if changed else ['stop','stop'],stops)

    def test_turn_retries_once_after_fresh_stop_and_keeps_original_target(self):
        import ast
        import math
        from pathlib import Path
        from types import SimpleNamespace
        method = next(n for n in ast.walk(ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'turn_relative')
        for outcome in ('recovers', 'still_blocked', 'reference_changed'):
            with self.subTest(outcome=outcome):
                now, report, stops, commands = [0.], {}, [], []
                node = SimpleNamespace(yaw=0., head_status={'reference_id':'head'},
                    robot={'motors':{'left':{'generation':'left'},'right':{'generation':'right'}}},
                    safety_ready=lambda: True, motion_ready=lambda: True,
                    reset_continuity=lambda: None, safety_reason=lambda **kw: None)
                def spin_once(self, **kw):
                    now[0] += .5
                    if stops and outcome == 'recovers':
                        self.yaw = -math.radians(29.)
                def stop():
                    stops.append(now[0])
                    if outcome == 'reference_changed':
                        node.robot['motors']['left']['generation'] = 'new'
                def spin_until(predicate, timeout, reason):
                    if reason == 'stopped odometry is unavailable':
                        now[0] += .1
                        node.yaw_at = now[0]
                        node.yaw = -math.radians(32.)  # Includes final coasting.
                    self.assertTrue(predicate())
                node.stop, node.spin_until = stop, spin_until
                node.velocity = commands.append
                scope = dict(math=math, time=SimpleNamespace(monotonic=lambda: now[0]),
                    rclpy=SimpleNamespace(spin_once=spin_once), report=report, ScanError=ScanError,
                    args=SimpleNamespace(turn_timeout=12.,turn_speed=.4,yaw_tolerance_degrees=3.),
                    same_motor_references=same_motor_references)
                exec(compile(ast.Module(body=[method],type_ignores=[]),'<turn-retry-test>','exec'),scope)
                if outcome == 'recovers':
                    self.assertAlmostEqual(32.,scope['turn_relative'](node,30.))
                    self.assertEqual(1,len(report['turn_retries']))
                else:
                    with self.assertRaisesRegex(ScanError, 'after retry' if outcome == 'still_blocked' else 'reference changed'):
                        scope['turn_relative'](node,30.)
                self.assertEqual(2,len(stops))  # Recovery stop and unconditional final stop.

    def test_zero_commands_do_not_claim_stopped_while_tracks_are_still_coasting(self):
        status = dict(motion_active=False, motors=dict(left=dict(speed=14,commanded_speed=0),
                                                       right=dict(speed=-23,commanded_speed=0)))
        self.assertTrue(chassis_motion_active(status))
        status['motors']['left']['speed'] = 0
        status['motors']['right']['speed'] = 0
        self.assertFalse(chassis_motion_active(status))

    def test_startup_revalidates_same_reference_limits_without_motion(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        method = next(n for n in ast.walk(ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'ensure_homed')
        scope = dict(String=SimpleNamespace,ScanError=ScanError)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<restore-test>','exec'),scope)
        for reference in ('same', 'changed'):
            commands=[]
            node=SimpleNamespace(head_at=1.,head_status=dict(homed=True,calibrated=False,
                moving=False,homing=False,manual_override=False,reference_id=reference,
                approved_reference_id='same',saved_limits=dict(reference_id='same',upper=4,lower=50)),
                head=SimpleNamespace(publish=lambda m:commands.append(m.data)))
            def acknowledge(predicate,timeout,message):
                node.head_at=2.;node.head_status['calibrated']=True
                self.assertTrue(predicate())
            node.spin_until=acknowledge
            if reference=='same':
                scope['ensure_homed'](node)
                self.assertEqual(['use_saved_limits'],commands)
            else:
                with self.assertRaises(ScanError):scope['ensure_homed'](node)
                self.assertEqual([],commands)

    def test_small_lower_fragment_gets_higher_view_without_rejecting_cropped_body(self):
        self.assertTrue(small_lower_person_candidate(dict(height_fraction=.08656, top_fraction=.6753)))
        self.assertFalse(small_lower_person_candidate(dict(height_fraction=.98, top_fraction=0)))
        self.assertFalse(small_lower_person_candidate(dict(height_fraction=.4, top_fraction=.6)))
        self.assertFalse(small_lower_person_candidate(dict(height_fraction=.1, top_fraction=.1)))
        self.assertFalse(small_lower_person_candidate(dict(height_fraction=float('nan'), top_fraction=.8)))

    def test_priority_stop_requires_new_stopped_feedback(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        method = next(n for n in ast.walk(ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'stop')
        for feedback in (True, False):
            now = [10.]
            commands, zeros = [], []
            node = SimpleNamespace(robot_at=10., robot={'motion_active': False},
                velocity=lambda: zeros.append(now[0]),
                head=SimpleNamespace(publish=lambda m: commands.append(m.data)))
            def spin(n, timeout_sec):
                now[0] += timeout_sec
                if feedback: n.robot_at = now[0]
            scope = dict(time=SimpleNamespace(monotonic=lambda:now[0]),
                         rclpy=SimpleNamespace(spin_once=spin),String=SimpleNamespace,
                         chassis_motion_active=chassis_motion_active,ScanError=ScanError)
            exec(compile(ast.Module(body=[method],type_ignores=[]),'<stop-test>','exec'),scope)
            if feedback: scope['stop'](node)
            else:
                with self.assertRaisesRegex(ScanError,'stopped status'):
                    scope['stop'](node)
            self.assertEqual(['stop'],commands)
            self.assertGreaterEqual(len(zeros),1)

    def test_face_edge_correction_uses_actual_face_and_stays_within_saved_range(self):
        head = dict(position=19, up_position=7, down_position=44)
        self.assertEqual(11, face_tilt_target(dict(center_y_fraction=.0577, box_height_fraction=.1153), head))
        head['position'] = 11
        self.assertEqual(7, face_tilt_target(dict(center_y_fraction=.06, box_height_fraction=.12), head))
        head['position'] = 7
        self.assertEqual(7, face_tilt_target(dict(center_y_fraction=.06, box_height_fraction=.12), head))
        self.assertEqual(7, face_tilt_target(dict(center_y_fraction=.4, box_height_fraction=.2), head))
        self.assertEqual(7, face_tilt_target(dict(center_y_fraction=float('nan'), box_height_fraction=.2), head))

    def test_reacquisition_angle_requires_same_reference_and_valid_limits(self):
        head = dict(forward_position=26, up_position=7, reference_id='boot',
                    minimum_target_position=7, maximum_target_position=44)
        self.assertEqual([11, 26], preferred_search_positions(head, 11, 'boot'))
        self.assertEqual([26], preferred_search_positions(head, 11, 'old'))
        self.assertEqual([26], preferred_search_positions(head, 99, 'boot'))
        head['position'] = 11
        self.assertEqual([26], preferred_search_positions(head))
        head['position'] = 44
        self.assertEqual([26], preferred_search_positions(head))

    def test_visible_face_holds_camera_while_waiting_for_identity(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        tree = ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'investigate_person')
        report, waits = {}, []
        now = [0.]
        scope = dict(report=report,time=SimpleNamespace(monotonic=lambda:now[0]),args=SimpleNamespace(candidate_seconds=12.),math=__import__('math'))
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<scan>', 'exec'), scope)
        def wait(seconds):
            waits.append(seconds); now[0]+=seconds
            return len(waits)==2
        node = SimpleNamespace(wait_for_target=wait, yaw=0., search_height_limits={},target={},
                               head_status={'position':10,'reference_id':'head'},
                               face_is_visible=lambda: True, face={'center_y_fraction': .3,
                                   'top_fraction':.2,'bottom_fraction':.4,'height_fraction':.2},
                               continuity=SimpleNamespace(retained=lambda now: None),
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)))
        self.assertTrue(scope['investigate_person'](node))
        self.assertEqual([1.2, 1.8], waits)
        self.assertEqual(node.face, report['face_observation'])

    def test_fast_search_never_sweeps_at_calibration_ceiling_limit(self):
        self.assertEqual([13, 22], fast_search_positions(dict(forward_position=13, up_position=-14, down_position=22)))
        self.assertEqual([2, 22], fast_search_positions(dict(forward_position=2, up_position=-19, down_position=22)))
        args = parse_args(['--fast-search'])
        self.assertEqual((.25, .1, .8, 15), (args.visual_settle_seconds, args.visual_hold_seconds,
                                         args.up_dwell_seconds, args.tilt_step_degrees))

    def test_person_guidance_raises_for_cropped_head_and_lowers_for_low_body(self):
        head = dict(position=0, up_position=-14, forward_position=13)
        self.assertEqual(-5, person_tilt_target(dict(top_fraction=0, height_fraction=.9), head))
        self.assertEqual(5, person_tilt_target(dict(top_fraction=.65, height_fraction=.3), head))
        self.assertEqual(0, person_tilt_target(dict(top_fraction=.3, height_fraction=.6), head))
    def test_plain_wall_does_not_abort_person_search(self):
        self.assertTrue(search_view_usable({'mean_luma': 143.5, 'mean_second_difference': 11.1, 'usable': False}))
        for value in (0, 11, float('nan'), None):
            self.assertFalse(search_view_usable({'mean_luma': value}))

    def test_centering_turns_toward_target_with_a_lenient_deadband(self):
        for x, expected in ((.02, -8), (.35, 0), (.5, 0), (.65, 0), (.98, 8)):
            self.assertEqual(expected, target_centering_step({'center_x_fraction': x}))
        for x in (None, float('nan'), -1, 2, True):
            with self.assertRaises(ScanError):
                target_centering_step({'center_x_fraction': x})

    def test_scan_rejects_a_frozen_previously_confirmed_target_message(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        source = Path('robot/jetson/mission/bounded_target_scan.py').read_text()
        tree = ast.parse(source)
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'target_is_visible')
        namespace = dict(time=SimpleNamespace(monotonic=lambda: 100.), target_is_confirmed=target_is_confirmed)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<scan>', 'exec'), namespace)
        node = SimpleNamespace(target_at=99.8, target=dict(ok=True, confirmed=True, age_seconds=.05, box_height_fraction=.3))
        self.assertTrue(namespace['target_is_visible'](node))
        node.target['age_seconds'] = .6  # Receipt age must count too.
        self.assertFalse(namespace['target_is_visible'](node))
        node.target['age_seconds'] = .05
        node.target_at = 99.
        self.assertFalse(namespace['target_is_visible'](node))
        node.target_at = None
        self.assertFalse(namespace['target_is_visible'](node))

    def test_expected_turn_motion_is_allowed_but_stale_heading_is_not(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        tree = ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'motion_reason')
        import math
        namespace = dict(time=SimpleNamespace(monotonic=lambda: 100.), math=math,
                         chassis_motion_active=chassis_motion_active)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<scan>', 'exec'), namespace)
        node = SimpleNamespace(robot_at=99.9, robot={'track_motion_active':True},
                               head_at=99.9, head_status={'homed':True,'moving':False,'homing':False},
                               yaw=.1, yaw_at=99.9)
        check = namespace['motion_reason']
        self.assertIsNone(check(node, allow_chassis_motion=True))
        self.assertIn('unexpectedly', check(node))
        node.yaw_at = 99.4  # Idle bridge publishes every 0.5 seconds plus jitter.
        self.assertIsNone(check(node, True))
        node.yaw_at = 98.9
        self.assertIsNone(check(node, True))
        node.yaw_at = 98.4
        self.assertEqual('odometry is stale', check(node, True))
        node.robot['track_motion_active'] = False
        self.assertIsNone(check(node))
        node.yaw_at = 97.4
        self.assertEqual('odometry is stale', check(node))
        node.yaw_at = 99.9
        node.head_status['moving'] = True
        self.assertEqual('camera head is moving', check(node, True))

    def test_turn_speed_is_bounded_and_head_search_can_be_disabled_for_chassis_check(self):
        self.assertFalse(parse_args(['--no-search-up']).search_up)
        for value in ('0', '-1', 'nan', 'inf', '0.7'):
            with self.subTest(value=value), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(['--turn-speed', value])

    def test_incremental_turns_are_small_and_unwind(self):
        headings = cable_safe_scan_headings(30, 180)
        turns = incremental_scan_turns(headings)
        self.assertTrue(all(abs(value) == 30 for value in turns))
        self.assertEqual(sum(turns), 0)
        self.assertEqual(headings[-1], 0)

    def test_target_requires_current_box_and_fresh_confirmation(self):
        value = {
            "ok": True,
            "confirmed": True,
            "age_seconds": 0.1,
            "box_height_fraction": 0.4,
        }
        self.assertTrue(target_is_confirmed(value))
        value["age_seconds"] = 1.0
        self.assertFalse(target_is_confirmed(value))
        value["age_seconds"] = 0.1
        value["box_height_fraction"] = 0.0
        self.assertFalse(target_is_confirmed(value))

    def test_invalid_target_payload_is_not_confirmation(self):
        for value in (None, {}, {"ok": False}, {"ok": True, "age_seconds": "x"}):
            self.assertFalse(target_is_confirmed(value))

    def test_dry_run_can_unwind_a_known_initial_heading(self):
        report = run(parse_args(["--initial-cable-heading-degrees", "30"]))
        self.assertEqual(report["incremental_turns"][0], -30)
        self.assertEqual(sum(report["incremental_turns"]), -30)

    def test_scan_can_start_to_the_left_and_still_unwinds(self):
        report = run(parse_args(["--first-direction", "left"]))
        self.assertEqual(report["requested_headings"][1], -30)
        self.assertEqual(report["requested_headings"][-1], 0)
        self.assertEqual(sum(report["incremental_turns"]), 0)

    def test_vertical_only_mode_is_available_for_locked_chassis_calibration(self):
        args = parse_args(["--vertical-only"])
        self.assertTrue(args.vertical_only)
        self.assertEqual(5, args.tilt_step_degrees)
        self.assertTrue(args.search_up)
        self.assertEqual(-120, args.face_search_position)
        self.assertEqual(1.5, args.visual_settle_seconds)
        self.assertEqual(0.75, args.visual_hold_seconds)
        self.assertEqual(30.0, args.camera_recovery_seconds)

    def test_visual_feedback_requires_positive_settle_and_hold_intervals(self):
        for option in (
            "--visual-settle-seconds",
            "--visual-hold-seconds",
            "--camera-recovery-seconds",
        ):
            with self.subTest(option=option), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args([option, "0"])

    def test_execute_scan_requires_physical_cable_neutral_confirmation(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--execute"])
        self.assertTrue(
            parse_args(["--execute", "--cable-zero-confirmed"]).execute
        )
        self.assertTrue(parse_args(["--execute", "--vertical-only"]).execute)

    def test_largest_person_box_drives_vertical_camera_guidance(self):
        body = largest_body_observation(
            [(300, 300, 100, 200), (320, 240, 300, 440)], 640, 480
        )
        self.assertAlmostEqual(0.5, body["center_x_fraction"])
        self.assertAlmostEqual(440 / 480.0, body["height_fraction"])
        self.assertLess(body["top_fraction"], 0.05)

    def test_no_valid_person_box_produces_no_guidance(self):
        self.assertIsNone(largest_body_observation([], 640, 480))
        self.assertIsNone(largest_body_observation([(1, 2, 0, 4)], 640, 480))

    def test_camera_head_motion_is_not_mistaken_for_chassis_motion(self):
        self.assertFalse(
            chassis_motion_active(
                {"motion_active": True, "tool_motion_active": True}
            )
        )
        self.assertTrue(
            chassis_motion_active(
                {"motion_active": True, "tool_motion_active": False}
            )
        )

    def test_upward_tilt_uses_the_discovered_direction(self):
        self.assertEqual(5, next_tilt_toward(-120, 0))
        self.assertEqual(-5, next_tilt_toward(0, -120))
        self.assertEqual(5, next_tilt_toward(-5, 0))
        self.assertIsNone(next_tilt_toward(-3, 0))


if __name__ == "__main__":
    unittest.main()
