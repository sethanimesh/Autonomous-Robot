#!/usr/bin/env python3
"""Replay route/occlusion rules without ROS, hardware, credentials or cloud calls."""
import argparse
import json
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot.jetson.navigation.navigation_reasoning import HEADINGS, fuse_route_evidence, occlusion_recovery


def local_scene():
    return dict(ok=True, floor_horizon_y=.35,
        candidates=[dict(heading_degrees=h, clear_distance_m=.15, confidence=.99, known_fraction=.99)
                    for h in HEADINGS.values()],
        evidence=[dict(heading_degrees=h, floor_fraction=.99, known_fraction=.99, sample_count=500)
                  for h in HEADINGS.values()])


def route_advice():
    return dict(quality='clear', near_turn_space='visible_clear', preference=['center', 'left', 'right'],
        evidence='Visible floor between furniture.', corridors=[dict(id=name, visibility='visible',
        hazard='none', evidence='Continuous visible floor.') for name in HEADINGS])


def scenarios():
    cases = []
    for name in ('clear', 'chair_center_left', 'chair_center_right', 'cable_center',
                 'hidden_floor', 'all_blocked', 'unknown_floor', 'new_obstacle',
                 'dark', 'covered_lens', 'uncertain_turn_space', 'malformed_gemini'):
        before, fresh, advice = local_scene(), local_scene(), route_advice()
        expected = 0
        if name in ('chair_center_left', 'chair_center_right', 'cable_center', 'uncertain_turn_space'):
            advice['corridors'][1]['hazard'] = 'thin_object' if name == 'cable_center' else 'solid_object'
            expected = -30
            if name == 'chair_center_right':
                advice['preference'] = ['right', 'left', 'center']
                expected = 30
            if name == 'uncertain_turn_space':
                advice['near_turn_space'] = 'unknown'
                expected = None
        if name == 'hidden_floor':
            for corridor in advice['corridors']:
                corridor['visibility'] = 'occluded'
            expected = None
        if name == 'all_blocked':
            for corridor in advice['corridors']:
                corridor['hazard'] = 'solid_object'
            expected = None
        if name == 'unknown_floor':
            for row in before['evidence']:
                row['known_fraction'] = .60
            expected = None
        if name == 'new_obstacle':
            for row in fresh['evidence']:
                row['floor_fraction'] = .50
            expected = None
        if name in ('dark', 'covered_lens'):
            advice['quality'] = 'dark' if name == 'dark' else 'lens_obstructed'
            expected = None
        if name == 'malformed_gemini':
            advice['motor_speed'] = 100
            expected = None
        result = fuse_route_evidence(before, fresh, advice)
        actual = None if result['decision']['blocked'] else result['decision']['heading_degrees']
        cases.append(dict(name=name, passed=actual == expected, expected_heading=expected, result=result))
    for name, cause, quality, position, age, reference, expected in (
        ('person_behind_chair', 'behind_object', 'clear', 22, 5, True, 'wait_then_rescan'),
        ('person_lost_after_tilt', 'camera_changed', 'clear', 14, 5, True, 'restore_previous_view'),
        ('person_left_frame', 'left_frame', 'clear', 22, 5, True, 'rescan'),
        ('occluded_lens', 'unknown', 'lens_obstructed', 22, 5, True, 'stop'),
        ('expired_person_memory', 'behind_object', 'clear', 22, 31, True, 'rescan'),
        ('rebooted_head', 'behind_object', 'clear', 14, 5, False, 'rescan'),
    ):
        advice = dict(quality=quality, cause=cause, inspect_side='left', evidence='Synthetic test observation.')
        result = occlusion_recovery(advice, 22, position, memory_age=age, same_reference=reference)
        cases.append(dict(name=name, passed=result['action'] == expected and not result['drive_allowed'],
                          expected_action=expected, result=result))
    return dict(ok=all(c['passed'] for c in cases), hardware_used=False, cloud_called=False,
                kind='synthetic rule replay; not measured vision accuracy', scenarios=cases)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    result = scenarios()
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('{0}/{1} offline route and occlusion scenarios passed.'.format(
        sum(c['passed'] for c in result['scenarios']), len(result['scenarios'])))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
