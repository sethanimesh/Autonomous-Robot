import copy
import unittest

from robot.jetson.navigation.navigation_reasoning import (
    fuse_route_evidence, occlusion_recovery, validate_route_advice, validate_occlusion_advice,
)
from scripts.diagnostics.simulate_navigation_reasoning import local_scene, route_advice, scenarios


class NavigationReasoningTests(unittest.TestCase):
    def test_prepared_scenarios(self):
        result = scenarios()
        for case in result['scenarios']:
            with self.subTest(case=case['name']):
                self.assertTrue(case['passed'], case)

    def test_gemini_preference_cannot_clear_any_local_veto(self):
        for key, value in [('floor_fraction', .94), ('floor_fraction', float('nan')),
                           ('known_fraction', .74), ('known_fraction', 1.01),
                           ('sample_count', 0), ('sample_count', True)]:
            for stage in ('initial', 'fresh'):
                initial, fresh = local_scene(), local_scene()
                selected = initial if stage == 'initial' else fresh
                selected['evidence'][1][key] = value
                with self.subTest(stage=stage, key=key, value=value):
                    result = fuse_route_evidence(initial, fresh, route_advice())
                    self.assertNotIn(0, result['approved_headings'])

    def test_missing_duplicate_and_unbounded_local_candidates_fail_closed(self):
        for mutate in (
            lambda value: value.pop('candidates'),
            lambda value: value.update(floor_horizon_y=None),
            lambda value: value['candidates'].extend(copy.deepcopy(value['candidates'])),
            lambda value: [row.update(confidence=2.) for row in value['candidates']],
            lambda value: [row.update(clear_distance_m=float('inf')) for row in value['candidates']],
        ):
            local = local_scene(); mutate(local)
            self.assertTrue(fuse_route_evidence(local, local_scene(), route_advice())['decision']['blocked'])

    def test_semantic_hazard_cannot_be_averaged_away(self):
        for hazard in ('solid_object', 'thin_object', 'person', 'drop_or_step', 'unknown'):
            advice = route_advice();advice['corridors'][1]['hazard'] = hazard
            result = fuse_route_evidence(local_scene(), local_scene(), advice)
            self.assertNotIn(0, result['approved_headings'])
            self.assertLessEqual(result['decision']['distance_m'], .05)

    def test_schema_rejects_duplicates_commands_and_bad_types(self):
        for mutate in (
            lambda value: value.update(preference=['left'] * 3),
            lambda value: value['corridors'][0].update(id='center'),
            lambda value: value.update(linear=.2),
            lambda value: value.update(quality=True),
            lambda value: value.update(evidence=' '),
        ):
            advice = route_advice();mutate(advice)
            with self.assertRaises(ValueError):validate_route_advice(advice)
            self.assertTrue(fuse_route_evidence(local_scene(), local_scene(), advice)['decision']['blocked'])

    def test_occlusion_never_confirms_identity_or_enables_drive(self):
        advice = dict(quality='clear', cause='behind_object', inspect_side='right', evidence='Chair overlap.')
        for age, reference, attempts in ((4, True, 0), (31, True, 0), (4, False, 0), (4, True, 2)):
            result = occlusion_recovery(advice, 22, 14, memory_age=age,
                                        same_reference=reference, attempts=attempts)
            self.assertFalse(result['drive_allowed'])
            self.assertNotIn('confirmed', result)
        self.assertEqual('hold_for_identity', occlusion_recovery(advice, 22, 14,
            memory_age=4, same_reference=True, face_visible=True)['action'])
        with self.assertRaises(ValueError):validate_occlusion_advice(dict(advice, identity='Mom'))


if __name__ == '__main__':unittest.main()
