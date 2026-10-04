import unittest

from scripts.diagnostics.observe_family_mission import summarize


def sample(source='face', epoch=0, **kwargs):
    return dict(profile_id='mom', track_id='a', identity_confirmed=True,
                identity_source=source, motion_epoch=epoch, age_seconds=.2, **kwargs)


class CombinedFamilyObserverTests(unittest.TestCase):
    def test_combined_evidence_keeps_tracking_distinct_from_saved_outfit_and_face(self):
        samples = [sample(), sample('tracking'), sample('tracking', 1), sample('clothing', 2)]
        mission = dict(outcome='target_found_not_at_standoff', steps=[dict(kind='target_approach',
            report=dict(drive_started=True, travelled_m=.063))])
        report = summarize(samples, 'mom', mission)
        self.assertEqual(1, report['face_observations'])
        self.assertEqual(2, report['local_clothing_continuity_observations'])
        self.assertEqual(1, report['saved_outfit_match_observations'])
        self.assertEqual(2, report['face_free_observations_after_motion'])
        self.assertEqual(1, report['approach_steps'])
        self.assertAlmostEqual(.063, report['encoder_travelled_m'])
        self.assertIn('unavailable', report['distance_check'])

    def test_stale_other_profile_and_unconfirmed_tracks_are_not_success(self):
        for field, value in (('age_seconds', 10), ('age_seconds', float('nan')),
                             ('profile_id', 'other'), ('identity_confirmed', False)):
            observation = sample()
            observation[field] = value
            self.assertEqual(0, summarize([observation], 'mom')['face_observations'])

    def test_distance_requires_a_fresh_validated_result_for_this_person_track(self):
        distance = dict(validated=True, age_seconds=.1, track_id='a', profile_id='mom')
        self.assertEqual(1, summarize([sample(range=distance)], 'mom')['validated_range_observations'])
        for field, value in (('validated', False), ('track_id', 'another'), ('profile_id', 'other'),
                             ('age_seconds', 4), ('age_seconds', float('nan'))):
            invalid = dict(distance)
            invalid[field] = value
            self.assertEqual(0, summarize([sample(range=invalid)], 'mom')['validated_range_observations'])

    def test_approximate_distance_is_reported_separately_from_verified_distance(self):
        distance = dict(validated=False, available=True, mode='approximate',
                        source='metric_depth_floor_estimate', distance_m=.65,
                        distance_reference='camera_ground_projection', age_seconds=.1,
                        track_id='a', profile_id='mom')
        report = summarize([sample('clothing', range=distance)], 'mom')
        self.assertEqual(1, report['approximate_range_observations'])
        self.assertEqual(0, report['validated_range_observations'])
        self.assertIn('calibrated camera distance confirmed at standoff', report['distance_check'])
        for field, value in (('available', False), ('distance_m', float('nan')),
                             ('track_id', 'another'), ('age_seconds', 4)):
            invalid = dict(distance, **{field: value})
            self.assertEqual(0, summarize([sample(range=invalid)], 'mom')['approximate_range_observations'])


if __name__ == '__main__':unittest.main()
