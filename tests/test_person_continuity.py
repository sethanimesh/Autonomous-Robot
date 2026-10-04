import unittest

from robot.jetson.perception.person_continuity import PersonContinuity


BODY = (0.10, 0.10, 0.60, 0.95)
FACE = (0.23, 0.15, 0.35, 0.30)


def person(box=BODY, appearance=(1.0, 0.0)):
    return {"box": box, "appearance": appearance}


class PersonContinuityTests(unittest.TestCase):
    def setUp(self):
        self.tracker = PersonContinuity()

    def observe(self, stamp, people=None, faces=None, now=None):
        return self.tracker.observe(
            [person()] if people is None else people,
            [FACE] if faces is None else faces,
            stamp,
            stamp if now is None else now,
        )

    def confirm(self):
        self.assertEqual("confirming_face", self.observe(0.1)["state"])
        self.assertEqual("confirming_face", self.observe(0.2)["state"])
        result = self.observe(0.3)
        self.assertEqual("face_confirmed", result["state"])
        return result

    def test_body_alone_cannot_establish_identity(self):
        for stamp in (0.1, 0.2, 0.3, 0.4):
            result = self.observe(stamp, faces=[])
            self.assertEqual("waiting_face", result["state"])
            self.assertEqual(0, result["face_hits"])

    def test_three_distinct_consistent_faces_establish_identity(self):
        result = self.confirm()
        self.assertEqual(3, result["face_hits"])
        self.assertEqual(BODY, tuple(result["body_box"]))
        self.assertAlmostEqual(0, result["face_age_seconds"])

    def test_duplicate_and_out_of_order_frames_cannot_add_face_hits(self):
        self.observe(0.1)
        self.assertEqual("ignored", self.observe(0.1, now=0.15)["state"])
        self.observe(0.2)
        self.assertEqual("ignored", self.observe(0.15, now=0.25)["state"])
        self.assertEqual("face_confirmed", self.observe(0.3)["state"])

    def test_short_missing_face_keeps_pending_hits_when_same_body_is_visible(self):
        self.observe(0.1)
        self.observe(0.2)
        pending = self.observe(0.3, faces=[])
        self.assertNotIn(pending["state"], ("face_confirmed", "body_continuity"))
        self.assertEqual(2, pending["face_hits"])
        self.assertAlmostEqual(0.1, pending["face_age_seconds"])
        result = self.observe(0.4)
        self.assertEqual("face_confirmed", result["state"])
        self.assertEqual(3, result["face_hits"])

    def test_pending_body_observations_do_not_add_face_hits_or_extend_face_timeout(self):
        self.observe(0.1)
        self.observe(0.2)
        for stamp in (0.4, 0.7, 0.94):
            result = self.observe(stamp, faces=[])
            self.assertEqual(2, result["face_hits"])
            self.assertNotIn(result["state"], ("face_confirmed", "body_continuity"))
            self.assertAlmostEqual(stamp - 0.2, result["face_age_seconds"])
        self.assertEqual("lost", self.observe(0.96, faces=[])["state"])
        self.assertEqual("waiting_face", self.observe(0.97, faces=[])["state"])
        self.assertEqual(1, self.observe(0.98)["face_hits"])

    def test_pending_face_evidence_does_not_survive_ambiguous_body_selection(self):
        self.observe(0.1)
        self.observe(0.2)
        people = [person(), person((0.12, 0.10, 0.63, 0.95))]
        self.assertEqual("lost", self.observe(0.3, people, [])["state"])
        self.assertEqual(1, self.observe(0.4)["face_hits"])

    def test_inconsistent_person_cannot_inherit_earlier_face_hits(self):
        self.observe(0.1)
        self.observe(0.2)
        other = person((0.65, 0.10, 0.99, 0.95), (0.0, 1.0))
        result = self.observe(0.3, [other], [(0.75, 0.15, 0.85, 0.30)])
        self.assertNotEqual("face_confirmed", result["state"])
        self.assertLessEqual(result["face_hits"], 1)

    def test_unassociated_face_cannot_establish_identity(self):
        for stamp in (0.1, 0.2, 0.3):
            result = self.observe(stamp, faces=[(0.75, 0.15, 0.85, 0.30)])
            self.assertNotEqual("face_confirmed", result["state"])
            self.assertEqual(0, result["face_hits"])

    def test_face_inside_two_person_boxes_is_ambiguous(self):
        people = [person(), person((0.15, 0.10, 0.65, 0.95))]
        for stamp in (0.1, 0.2, 0.3):
            result = self.observe(stamp, people)
            self.assertNotEqual("face_confirmed", result["state"])
            self.assertEqual(0, result["face_hits"])

    def test_multiple_matched_faces_cannot_select_an_identity_anchor(self):
        faces = [FACE, (0.40, 0.15, 0.50, 0.30)]
        for stamp in (0.1, 0.2, 0.3):
            result = self.observe(stamp, faces=faces)
            self.assertNotEqual("face_confirmed", result["state"])
            self.assertEqual(0, result["face_hits"])

    def test_body_tracks_after_face_turns_away_and_reports_current_position(self):
        self.confirm()
        moved_box = (0.12, 0.10, 0.62, 0.95)
        result = self.observe(0.4, [person(moved_box)], [])
        self.assertEqual("body_continuity", result["state"])
        self.assertEqual(moved_box, tuple(result["body_box"]))
        self.assertAlmostEqual(0.1, result["face_age_seconds"])

    def test_histograms_are_normalized_before_comparison(self):
        self.confirm()
        result = self.observe(0.4, [person(appearance=(25.0, 0.0))], [])
        self.assertEqual("body_continuity", result["state"])

    def test_matching_appearance_elsewhere_is_not_continuity(self):
        self.confirm()
        result = self.observe(0.4, [person((0.70, 0.10, 0.99, 0.95))], [])
        self.assertEqual("lost", result["state"])
        self.assertEqual("waiting_face", self.observe(0.5, faces=[])["state"])

    def test_spatial_overlap_with_different_appearance_is_not_continuity(self):
        self.confirm()
        result = self.observe(0.4, [person(appearance=(0.0, 1.0))], [])
        self.assertEqual("lost", result["state"])

    def test_two_qualifying_bodies_end_continuity_instead_of_selecting_largest(self):
        self.confirm()
        result = self.observe(0.4, [person(), person((0.12, 0.10, 0.63, 0.95))], [])
        self.assertEqual("lost", result["state"])

    def test_body_only_updates_cannot_drift_the_appearance_reference(self):
        self.confirm()
        result = self.observe(0.4, [person(appearance=(0.64, 0.36))], [])
        self.assertEqual("body_continuity", result["state"])
        # This resembles the last body observation, but no longer resembles
        # the appearance that was associated with the confirmed face.
        result = self.observe(0.5, [person(appearance=(0.25, 0.75))], [])
        self.assertEqual("lost", result["state"])

    def test_short_missing_body_has_no_position_and_can_return_within_gap(self):
        self.confirm()
        for stamp in (0.4, 0.7, 0.9):
            result = self.observe(stamp, [], [])
            self.assertEqual("temporarily_missing", result["state"])
            self.assertEqual(3, result["face_hits"])
            self.assertNotIn("body_box", result)
            self.assertAlmostEqual(stamp - 0.3, result["face_age_seconds"])
        result = self.observe(1.0, faces=[])
        self.assertEqual("body_continuity", result["state"])
        self.assertEqual(BODY, tuple(result["body_box"]))
        self.assertAlmostEqual(0.7, result["face_age_seconds"])

    def test_pending_missing_body_does_not_report_position_or_forget_valid_face_hits(self):
        self.observe(0.1)
        result = self.observe(0.2, [], [])
        self.assertEqual("temporarily_missing", result["state"])
        self.assertEqual(1, result["face_hits"])
        self.assertNotIn("body_box", result)
        result = self.observe(0.3)
        self.assertEqual("confirming_face", result["state"])
        self.assertEqual(2, result["face_hits"])
        self.assertEqual("face_confirmed", self.observe(0.4)["state"])

    def test_missing_frames_do_not_renew_body_gap_and_expired_body_cannot_return(self):
        self.confirm()
        for stamp in (0.4, 0.7, 1.0):
            self.assertEqual("temporarily_missing", self.observe(stamp, [], [])["state"])
        self.assertEqual("lost", self.observe(1.06, [], [])["state"])
        self.assertEqual("waiting_face", self.observe(1.1, faces=[])["state"])
        self.assertEqual("confirming_face", self.observe(1.2)["state"])

    def test_face_without_valid_body_does_not_add_hits_or_refresh_pending_evidence(self):
        self.observe(0.1)
        for stamp in (0.2, 0.4, 0.8):
            result = self.observe(stamp, [], [FACE])
            self.assertEqual("temporarily_missing", result["state"])
            self.assertEqual(1, result["face_hits"])
            self.assertNotIn("body_box", result)
            self.assertAlmostEqual(stamp - 0.1, result["face_age_seconds"])
        self.assertEqual("lost", self.observe(0.86, [], [FACE])["state"])
        self.assertEqual(1, self.observe(0.9)["face_hits"])

    def test_face_without_valid_body_does_not_refresh_established_identity(self):
        self.confirm()
        result = self.observe(0.5, [], [FACE])
        self.assertEqual("temporarily_missing", result["state"])
        self.assertNotIn("body_box", result)
        self.assertAlmostEqual(0.2, result["face_age_seconds"])
        result = self.observe(0.6, faces=[])
        self.assertEqual("body_continuity", result["state"])
        self.assertAlmostEqual(0.3, result["face_age_seconds"])

    def test_unassociated_face_with_other_visible_body_cannot_refresh_identity(self):
        self.confirm()
        result = self.observe(0.4, faces=[(0.75, 0.15, 0.85, 0.30)])
        self.assertEqual("lost", result["state"])
        self.assertEqual("waiting_face", self.observe(0.5, faces=[])["state"])

    def test_update_gap_expires_continuity(self):
        self.confirm()
        self.assertEqual("lost", self.observe(1.06, faces=[])["state"])
        self.assertEqual("waiting_face", self.observe(1.1, faces=[])["state"])

    def test_regular_body_updates_cannot_extend_absolute_face_expiry(self):
        self.confirm()
        for index in range(1, 20):
            result = self.observe(0.3 + index * 0.5, faces=[])
            self.assertEqual("body_continuity", result["state"])
        self.assertEqual("lost", self.observe(10.31, faces=[])["state"])
        self.assertEqual("waiting_face", self.observe(10.4, faces=[])["state"])

    def test_face_return_after_seven_seconds_preserves_consistent_track(self):
        self.confirm()
        for index in range(1, 15):
            self.assertEqual('body_continuity', self.observe(.3+index*.5, faces=[])['state'])
        result = self.observe(7.6)
        self.assertEqual('face_confirmed', result['state'])
        self.assertEqual(3, result['face_hits'])
        self.assertEqual(0., result['face_age_seconds'])

    def test_original_five_second_lifetime_can_be_reproduced(self):
        self.tracker = PersonContinuity(maximum_face_age_seconds=5.)
        self.confirm()
        for index in range(1, 10):
            self.assertEqual('body_continuity', self.observe(.3+index*.5, faces=[])['state'])
        self.assertEqual('face_anchor_expired', self.observe(5.31, faces=[])['reason'])

    def test_face_lifetime_requires_a_finite_bounded_duration(self):
        for value in (None, True, 0, -1, float('nan'), float('inf'), 31):
            with self.subTest(value=value), self.assertRaises(ValueError):
                PersonContinuity(maximum_face_age_seconds=value)

    def test_stale_and_future_source_frames_cannot_establish_identity(self):
        for stamp, now in ((0.1, 0.86), (0.3, 0.20)):
            with self.subTest(stamp=stamp, now=now):
                self.tracker.reset()
                result = self.observe(stamp, now=now)
                self.assertNotEqual("face_confirmed", result["state"])
                self.assertEqual(0, result["face_hits"])

    def test_reset_requires_new_face_evidence(self):
        self.confirm()
        self.tracker.reset()
        result = self.observe(0.4, faces=[])
        self.assertEqual("waiting_face", result["state"])
        self.assertEqual(0, result["face_hits"])


if __name__ == "__main__":
    unittest.main()
