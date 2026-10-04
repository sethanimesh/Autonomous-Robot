import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import subprocess
import sys

from robot.jetson.mission.bounded_target_scan import next_tilt_toward, preferred_search_positions, background_scene_request
from robot.jetson.mission.search_view_policy import search_scene_action, human_framing_plan, upward_budget_reached


def method(name, scope):
    tree = ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
    value = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[value], type_ignores=[]), '<empty-room-search>', 'exec'), scope)
    return scope[name]


class EmptyRoomSearchTests(unittest.TestCase):
    def test_pending_cloud_does_not_delay_scan_process_exit(self):
        code='from robot.jetson.mission.bounded_target_scan import background_scene_request; import time; background_scene_request(time.sleep, 10); print("local result ready")'
        result=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=3,check=True)
        self.assertIn('local result ready',result.stdout)

    def test_scene_worker_preserves_results_and_errors(self):
        self.assertEqual(5,background_scene_request(lambda a:a+1,4).result(timeout=2))
        def fail():raise ValueError('unavailable')
        with self.assertRaisesRegex(ValueError,'unavailable'):
            background_scene_request(fail).result(timeout=2)

    def run_search(self, scene='room', human=False):
        positions, checks, report, investigated = [], [], {}, []
        head = dict(position=16, up_position=3, settle_tolerance=4,
                    minimum_target_position=3, maximum_target_position=16)
        def look(position):
            positions.append(position)
            head['position'] = position
        def assess():
            checks.append(head['position'])
            return dict(observations=[dict(quality='clear', scene=scene,
                human_visible='yes' if human else 'no', visible_parts=['legs'] if human else [],
                framing_hint='raise')])
        node = NS(head_status=head, stop=lambda:None, target_is_visible=lambda:False,
                  face_is_visible=lambda:False, body_is_visible=lambda:False,
                  assess_search_scene=assess, look_at_search_position=look,
                  wait_for_target=lambda seconds:False,
                  investigate_person=lambda **kwargs:investigated.append(kwargs.get('initial_advice')) or True)
        scope = dict(report=report,args=NS(tilt_step_degrees=15,up_dwell_seconds=.8),
                     search_scene_action=search_scene_action,human_framing_plan=human_framing_plan,
                     upward_budget_reached=upward_budget_reached,next_tilt_toward=next_tilt_toward)
        found = method('seek_target_upward',scope)(node)
        return found,positions,checks,report,investigated

    def test_empty_room_checks_both_heights_and_ends_bounded(self):
        found,moves,checks,report,_ = self.run_search()
        self.assertFalse(found)
        self.assertEqual([3],moves)
        self.assertEqual([16,3],checks)
        self.assertEqual(['empty_higher_views'],report['view_returns'])

    def test_ceiling_does_not_cause_more_upward_steps(self):
        found,moves,checks,report,_ = self.run_search(scene='ceiling_only')
        self.assertFalse(found)
        self.assertFalse(moves)
        self.assertEqual(['ceiling_view'],report['view_returns'])

    def test_cloud_body_part_uses_person_investigation_when_detector_misses(self):
        found,moves,checks,report,investigated = self.run_search(human=True)
        self.assertTrue(found)
        self.assertEqual(['legs'],investigated[0]['observations'][0]['visible_parts'])
        self.assertFalse(moves)

    def test_camera_only_empty_scan_restores_forward_view(self):
        restored=[]
        node=NS(wait_for_target=lambda:False,body_is_visible=lambda:False,
                seek_target_upward=lambda:False,look_forward=lambda:restored.append(True))
        scope=dict(node=node,args=NS(try_up=True,search_up=True,vertical_only=True),report={},
                   target_seen_at=lambda heading:True)
        self.assertFalse(method('observe_heading',scope)(0))
        self.assertEqual([True],restored)

    def test_fast_scan_checks_cloud_without_a_local_detection(self):
        tree=ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        block=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                   and ast.unparse(n.test)=='args.fast_search and (not args.vertical_only)')
        wrapper=ast.parse('def replay():\n    pass').body[0]
        wrapper.body=[block]
        for human in (True,False,None):
            with self.subTest(human=human):
                calls=[];investigations=[];report={}
                advice=(None if human is None else dict(observations=[dict(quality='clear',scene='room',
                    human_visible='yes' if human else 'no',visible_parts=['legs'] if human else [],framing_hint='raise')]))
                def assess():calls.append(True);return advice
                def investigate(**kwargs):investigations.append(kwargs['initial_advice']);return True
                head=dict(position=16,forward_position=16,down_position=16,minimum_target_position=3,maximum_target_position=16)
                node=NS(head_status=head,look_at_search_position=lambda _:None,wait_for_target=lambda:False,
                    wait_for_tracked_face=lambda:False,body_is_visible=lambda:False,face_is_visible=lambda:False,
                    target_is_visible=lambda:False,assess_search_scene=assess,investigate_person=investigate,body=None)
                def found(_):report['outcome']='target_found';return True
                scope=dict(node=node,report=report,headings=[0],scan_origin_heading=0,cable_heading=0,
                    args=NS(fast_search=True,vertical_only=False,range_lower_view=False,preferred_head_position=None,
                        preferred_head_reference=None,yaw_tolerance_degrees=3,try_up=True),
                    preferred_search_positions=preferred_search_positions,human_framing_plan=human_framing_plan,
                    target_seen_at=found)
                exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'<fast-cloud-search>','exec'),scope)
                scope['replay']()
                self.assertEqual([True],calls)
                self.assertEqual([advice] if human else [],investigations)
                self.assertEqual('target_found' if human else 'scan_complete_no_target',report['outcome'])
