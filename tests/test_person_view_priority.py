import ast
import math
from robot.jetson.mission.search_view_policy import search_scene_action, upward_budget_reached, human_framing_plan, combine_framing_plans
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from robot.jetson.mission.bounded_target_scan import person_investigation_plan


class PersonViewPriorityTests(unittest.TestCase):
    def head(self, position=62):
        return dict(position=position, up_position=43, forward_position=60,
                    minimum_target_position=43, maximum_target_position=77,
                    reference_id='same')

    def test_seated_body_cropped_at_top_raises_incrementally(self):
        body=dict(top_fraction=.0038, height_fraction=.8868)
        self.assertEqual([54],person_investigation_plan(body,None,self.head())['positions'])

    def test_face_at_top_never_selects_lower_view(self):
        face=dict(top_fraction=.00048,bottom_fraction=.1026,height_fraction=.1021)
        self.assertEqual([43],person_investigation_plan(None,face,self.head(45))['positions'])

    def test_scenarios_choose_vertical_framing_before_horizontal_search(self):
        body = dict(top_fraction=.2,height_fraction=.6,center_x_fraction=.5)
        face = dict(top_fraction=.2,bottom_fraction=.4,height_fraction=.2,center_x_fraction=.5)
        cases = [
            (body, dict(face,top_fraction=0.,center_x_fraction=.1), 'face_above_view', False),
            (body, dict(face,top_fraction=.8,bottom_fraction=1.), 'face_below_view', False),
            (body, dict(face,center_x_fraction=.1), 'face_at_side', True),
            (body, dict(face,center_x_fraction=.9), 'face_at_side', True),
            (body, face, 'face_visible_identity_pending', False),
            (dict(body,top_fraction=0.),None,'body_cropped_above',False),
            (dict(body,top_fraction=.75,height_fraction=.1),None,'small_lower_body_fragment',False),
            (dict(body,top_fraction=.06),None,'body_suggests_higher_face',False),
            (dict(body,top_fraction=.65),None,'body_suggests_lower_face',False),
            (dict(body,center_x_fraction=.1),None,'body_at_side',True),
            (body,None,'body_only_check_upper_view',False),
            (None,None,'person_temporarily_missing',False),
        ]
        for candidate, detection, scenario, center in cases:
            with self.subTest(scenario=scenario):
                decision=person_investigation_plan(candidate,detection,self.head())
                self.assertEqual(scenario,decision['scenario'])
                self.assertEqual(center,decision['center_person'])
                self.assertTrue(all(43<=p<=77 for p in decision['positions']))
                if scenario=='face_visible_identity_pending':
                    self.assertEqual([],decision['positions'])

    def test_losing_face_in_exploration_restores_best_face_view(self):
        tree=ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text())
        fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='investigate_person')
        now=[100.];moves=[];report={}
        face=dict(top_fraction=.8,bottom_fraction=1.,height_fraction=.2,center_x_fraction=.5)
        node=NS(head_status=self.head(60),face=face,body={},yaw=0.,search_height_limits={},
                continuity=NS(retained=lambda _:None),
                get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(now[0]*1e9))),
                body_is_visible=lambda:False,target_is_visible=lambda:False)
        node.face_is_visible=lambda:node.head_status['position']==60
        def wait(seconds):
            now[0]+=seconds
            return bool(moves and moves[-1]==60)
        def look(position):
            moves.append(position);node.head_status['position']=position
        node.wait_for_target=wait;node.look_at_search_position=look;node.assess_search_scene=lambda:None
        scope=dict(time=NS(monotonic=lambda:now[0]),args=NS(candidate_seconds=20.,search_view_timeout_seconds=22.),
                   report=report,person_investigation_plan=person_investigation_plan,math=math,
                   search_scene_action=search_scene_action,upward_budget_reached=upward_budget_reached,human_framing_plan=human_framing_plan,combine_framing_plans=combine_framing_plans)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<person-view>', 'exec'),scope)
        self.assertTrue(scope['investigate_person'](node))
        self.assertEqual([68,60],moves)
        self.assertEqual([60],report['retained_face_views'])


if __name__=='__main__':unittest.main()
