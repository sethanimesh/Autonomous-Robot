"""Replay the live identity -> lower-view handoff without ROS or motors."""
import ast
import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from robot.jetson.mission.autonomous_find import parse_args,run
from robot.jetson.mission.person_approach import approach_decision
from robot.jetson.mission.bounded_target_scan import ScanError


def target(distance=.24, **changes):
    value=dict(identity_confirmed=True,profile_id='mom',track_id='t',age_seconds=.1,
        identity_source='face',clothing_approach_enabled=True,approximate_approach_enabled=True,
        range=dict(available=True,validated=False,mode='approximate',source='metric_depth_floor_estimate',
                   distance_reference='camera_ground_projection',distance_m=distance,
                   lower_m=max(0,distance-.15),upper_m=distance+.15,age_seconds=.1,
                   consistent_samples=2,feet_checked=True,track_id='t',profile_id='mom'))
    value.update(changes)
    return dict(outcome='target_found',target_observation=value)


class ApproachHandoffTests(unittest.TestCase):
    def test_face_on_last_identity_retry_still_gets_lower_view_then_short_step(self):
        lost=target(identity_confirmed=False,track_id=None,range=None)
        needs_floor=target(range=dict(available=False,validated=False,mode='approximate',
                                      source='metric_depth_floor_estimate',track_id='t',profile_id='mom'))
        replies=[lost,copy.deepcopy(lost),needs_floor,target(),
                 dict(outcome='success',drive_started=True,travelled_m=.02),target(.18)]
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed',
                             '--maximum-approach-steps','1','--report',directory+'/mission.json'])
            with patch('robot.jetson.mission.autonomous_find.run_child',side_effect=replies) as child:
                report=run(args)
        self.assertEqual('target_found_at_estimated_standoff',report['outcome'])
        commands=[c.args[0] for c in child.call_args_list]
        self.assertIn('--range-lower-view',commands[3])
        self.assertNotIn('--try-up',commands[3]);self.assertNotIn('--search-up',commands[3])
        self.assertEqual('0.02',commands[4][commands[4].index('--maximum-step-distance')+1])

    def test_range_worker_waits_through_a_short_identity_gap(self):
        tree=ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='target_seen_at')
        for returns in (True,False):
            now=[0.];report={};visible=[False]
            node=NS(target=target(identity_confirmed=False)['target_observation'],
                    head_status=dict(position=6,reference_id='boot'),target_is_visible=lambda:visible[0],
                    safety_ready=lambda:True)
            def spin(node,timeout_sec):
                now[0]+=timeout_sec
                if returns and now[0]>=.2:
                    visible[0]=True;node.target=target()['target_observation']
            scope=dict(node=node,args=NS(center_target=False,range_lower_view=True),report=report,
                       time=NS(monotonic=lambda:now[0]),rclpy=NS(spin_once=spin),
                       approach_decision=approach_decision,ScanError=ScanError,
                       cable_heading=0.,scan_origin_heading=0.)
            exec(compile(ast.Module(body=[method],type_ignores=[]),'<range-handoff>','exec'),scope)
            result=scope['target_seen_at'](0)
            with self.subTest(returns=returns):
                self.assertEqual(returns,result)
                if returns:
                    self.assertTrue(report['target_observation']['identity_confirmed'])
                    self.assertLess(now[0],.3)
                else:
                    self.assertNotIn('target_observation',report)
                    self.assertNotEqual('target_found',report.get('outcome'))
                    self.assertLess(now[0],7.1)

    def test_lower_inspection_does_not_raise_camera_or_scan_other_headings(self):
        tree=ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        block=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                   and ast.unparse(n.test)=='args.fast_search and (not args.vertical_only)')
        wrapper=ast.parse('def replay():\n    pass').body[0];wrapper.body=[block]
        moves=[];report={}
        def unexpected(*a,**k):raise AssertionError('A floor check must not turn into person search')
        node=NS(head_status=dict(down_position=6),look_at_search_position=moves.append,
                wait_for_target=lambda:False,wait_for_tracked_face=lambda:False,
                body_is_visible=unexpected,face_is_visible=unexpected,assess_search_scene=unexpected,
                investigate_person=unexpected)
        scope=dict(node=node,report=report,headings=[0,15,-15],scan_origin_heading=0,cable_heading=0,
                   args=NS(fast_search=True,vertical_only=False,range_lower_view=True,try_up=True,yaw_tolerance_degrees=3),
                   human_framing_plan=lambda *a:None,guarded_turn=unexpected)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'<floor-only>','exec'),scope)
        scope['replay']()
        self.assertEqual([6],moves)
        self.assertEqual([0.],report['search_passes'][0]['headings'])
        self.assertEqual('scan_complete_no_target',report['outcome'])


if __name__=='__main__':unittest.main()
