import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from robot.jetson.mission.bounded_target_scan import person_investigation_plan, ScanError
from robot.jetson.mission.search_view_policy import search_scene_action, upward_budget_reached, human_framing_plan, combine_framing_plans


class SearchViewPolicyTests(unittest.TestCase):
    def test_ceiling_returns_but_real_face_overrides_scene_guess(self):
        result = {'observations':[dict(scene='ceiling_only',quality='clear')]}
        self.assertEqual('return_from_ceiling',search_scene_action(result))
        self.assertEqual('keep_person',search_scene_action(result,True))
        self.assertEqual('keep_person',search_scene_action({'observations':[dict(
            scene='ceiling_only',quality='clear',human_visible='yes',visible_parts=['head'])]}))
        result['observations'][0]['quality']='unknown'
        self.assertEqual('use_detector_rules',search_scene_action(result))
        self.assertEqual('use_detector_rules',search_scene_action(None))
        self.assertTrue(upward_budget_reached(22,-2))
        self.assertFalse(upward_budget_reached(22,-2,True))

    def test_body_parts_choose_direction_without_requiring_full_body(self):
        head=dict(position=22,minimum_target_position=-19,maximum_target_position=22)
        for hint,parts,expected in [('raise',['legs','feet'],[14]),('raise',['torso'],[14]),
                                    ('lower',['face'],[22]),('hold',['head','face'],[])]:
            advice={'observations':[dict(quality='clear',human_visible='yes',visible_parts=parts,framing_hint=hint)]}
            self.assertEqual(expected,human_framing_plan(advice,head)['positions'])
            self.assertIsNone(human_framing_plan(advice,head,True))

    def test_combination_favors_gemini_but_preserves_face_precision_and_fallback(self):
        up=dict(scenario='gemini_raise',positions=[14],center_person=False)
        down=dict(scenario='body_suggests_lower_face',positions=[30],center_person=False)
        result=combine_framing_plans(up,down,22)
        self.assertEqual('gemini',result['selected_source'])
        self.assertEqual({'raise':3,'lower':1},result['framing_scores'])
        self.assertFalse(result['sources_agree'])
        self.assertEqual('face_detector',combine_framing_plans(up,down,22,True)['selected_source'])
        self.assertEqual('person_detector',combine_framing_plans(None,down,22)['selected_source'])
        agreement=combine_framing_plans(up,dict(down,positions=[14]),22)
        self.assertEqual({'raise':4},agreement['framing_scores'])
        self.assertTrue(agreement['sources_agree'])

    def replay(self, outcome, cloud_delay=0., candidate_seconds=30., initial_advice=None):
        method=next(n for n in ast.walk(ast.parse(Path('robot/jetson/mission/bounded_target_scan.py').read_text()))
                    if isinstance(n,ast.FunctionDef) and n.name=='investigate_person')
        now=[0.]; moves=[]; report={}; calls=[]
        head=dict(position=22,reference_id='ref',minimum_target_position=-19,maximum_target_position=22,up_position=-19)
        node=NS(head_status=head, yaw=0., search_height_limits={},
                body=dict(top_fraction=0.,height_fraction=.8), face={},
                continuity=NS(retained=lambda _:None),get_clock=lambda:NS(now=lambda:NS(nanoseconds=0)),
                face_is_visible=lambda:False,target_is_visible=lambda:False)
        def wait(seconds):now[0]+=seconds;return False
        def look(target):moves.append(target);head['position']=target
        def visible():return outcome not in ('lost', 'occlusion', 'occlusion_restore', 'covered') or head['position']==22
        def assess():
            calls.append(head['position'])
            now[0] += cloud_delay
            if head['position'] < 22 and outcome in ('occlusion', 'occlusion_restore', 'covered'):
                recovery = dict(action='wait_then_rescan', wait_seconds=1.2)
                if outcome == 'occlusion_restore':recovery = dict(action='restore_previous_view', position=20)
                if outcome == 'covered':recovery = dict(action='stop', reason='lens obstructed')
                return dict(occlusion_recovery=recovery)
            if outcome=='guided':
                return {'observations':[dict(scene='room',quality='clear',human_visible='yes',
                    visible_parts=['torso'] if head['position']> -2 else ['head','face'],
                    framing_hint='raise' if head['position']> -2 else 'hold')]}
            return {'observations':[dict(scene='ceiling_only',quality='clear')]} if outcome=='ceiling' and head['position']<22 else None
        node.wait_for_target=wait;node.look_at_search_position=look;node.body_is_visible=visible;node.assess_search_scene=assess
        scope=dict(math=math,ScanError=ScanError,time=NS(monotonic=lambda:now[0]),report=report,args=NS(candidate_seconds=candidate_seconds,search_view_timeout_seconds=22.),
                   person_investigation_plan=person_investigation_plan,search_scene_action=search_scene_action,
                   upward_budget_reached=upward_budget_reached,human_framing_plan=human_framing_plan,combine_framing_plans=combine_framing_plans)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<view-policy>','exec'),scope)
        self.assertFalse(scope['investigate_person'](node, initial_advice=initial_advice))
        return moves,report,node,calls,scope

    def test_occlusion_holds_once_or_restores_the_bound_previous_view(self):
        moves, report, _, calls, _ = self.replay('occlusion')
        self.assertEqual([14, 22], moves)
        self.assertEqual([22, 14], calls)
        self.assertEqual(['person_lost'], report['view_returns'])
        moves, _, _, _, _ = self.replay('occlusion_restore')
        self.assertEqual([14, 20], moves)

    def test_obstructed_lens_stops_investigation_instead_of_raising_or_driving(self):
        with self.assertRaisesRegex(ScanError, 'lens obstructed'):
            self.replay('covered')

    def test_cloud_time_does_not_skip_observation_of_final_raised_view(self):
        moves,report,node,calls,_=self.replay('guided',cloud_delay=4.,candidate_seconds=12.)
        self.assertEqual([14,6,-2,-2],moves)
        self.assertEqual([22,14,6,-2],calls)
        self.assertEqual('gemini_hold',report['candidate_decisions'][-1]['scenario'])
        self.assertEqual(['head','face'],report['cloud_person_candidate']['visible_parts'])

    def test_search_view_answer_is_reused_for_first_person_adjustment(self):
        advice={'observations':[dict(scene='room',quality='clear',human_visible='yes',
                    visible_parts=['legs'],framing_hint='raise')]}
        moves,report,_,calls,_=self.replay('guided',initial_advice=advice)
        self.assertEqual([14,6,-2,-2],moves)
        self.assertEqual([14,6,-2],calls)
        self.assertEqual('gemini_raise',report['candidate_decisions'][0]['scenario'])

    def test_lost_person_returns_down_and_does_not_repeat_failed_raise(self):
        moves,report,node,calls,scope=self.replay('lost')
        self.assertEqual([14,22],moves)
        self.assertEqual(['person_lost'],report['view_returns'])
        self.assertEqual([22,14],calls)
        scope['investigate_person'](node)
        self.assertEqual([14,22,22],moves) # only restoration; no repeated upward excursion

    def test_gemini_ceiling_restores_room_view(self):
        moves,report,node,calls,_=self.replay('ceiling')
        self.assertEqual([14,22],moves)
        self.assertEqual(['ceiling_view'],report['view_returns'])
        self.assertEqual([22,14],calls)

    def test_unavailable_cloud_continues_three_probes_then_returns(self):
        moves,report,node,calls,_=self.replay('unavailable')
        self.assertEqual([14,6,-2,22],moves)
        self.assertEqual(['upward_checks_exhausted'],report['view_returns'])
        self.assertEqual(4,len(calls))


if __name__=='__main__':unittest.main()
