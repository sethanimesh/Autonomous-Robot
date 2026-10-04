"""ROS-independent contracts for continuity joining and evidence freshness."""

import itertools
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from robot.jetson.perception.person_continuity_stream import PersonContinuityStream


def detection(box, class_id):
    x1, y1, x2, y2 = box
    return SimpleNamespace(
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=(x1+x2)*50, y=(y1+y2)*50)),
            size_x=(x2-x1)*100, size_y=(y2-y1)*100,
        ),
        results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id=class_id))],
    )


def message(stamp, detections=(), frame_id='camera'):
    seconds = int(stamp)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=SimpleNamespace(
            sec=seconds, nanosec=round((stamp-seconds)*1e9))),
        width=100, height=100, step=300, encoding='bgr8', data=bytes([100])*30_000,
        detections=list(detections),
    )


BODY = (.2, .05, .8, .95)
FACE = (.4, .1, .6, .3)


class PersonContinuityStreamTests(unittest.TestCase):
    def setUp(self):
        self.stream = PersonContinuityStream()
        # Pixel statistics are tested by PersonContinuity separately. These
        # adapter tests exercise source matching with no ROS or CV dependency.
        self.addCleanup(patch.stopall)
        patch.dict('sys.modules', {'numpy': MagicMock()}).start()
        self.appearance = patch(
            'robot.jetson.perception.person_continuity_stream.person_appearance',
            return_value=[1., 0.],
        ).start()

    def triplet(self, stamp, face=True, body=True, now=None,
                order=('image', 'people', 'matches'), frame_id='camera'):
        messages = {
            'image': message(stamp, frame_id=frame_id),
            'people': message(stamp, [detection(BODY, 'person')] if body else [], frame_id),
            'matches': message(stamp, [detection(FACE, 'target_person')] if face else [], frame_id),
        }
        result = None
        for name in order:
            offered = self.stream.offer(name, messages[name], stamp+.01 if now is None else now)
            if offered is not None:
                result = offered
        return result

    def seed(self):
        for stamp in (100., 100.1, 100.2):
            self.triplet(stamp)
        self.assertIsNotNone(self.stream.observed(100.22))

    def test_each_callback_order_can_join_exact_source_frame(self):
        for index, order in enumerate(itertools.permutations(('image', 'people', 'matches'))):
            result = self.triplet(100.+index*.1, face=False, order=order)
            self.assertEqual('waiting_face', result['state'])
        self.assertEqual(6, self.appearance.call_count)

    def test_nearby_frames_are_never_associated(self):
        for name, stamp in zip(('image', 'people', 'matches'), (100., 100.01, 100.02)):
            self.assertIsNone(self.stream.offer(name, message(stamp), 100.1))
        self.appearance.assert_not_called()
        self.assertIsNone(self.stream.observed(100.1))

    def test_same_timestamp_from_different_camera_does_not_join(self):
        self.stream.offer('image', message(100., frame_id='a'), 100.1)
        self.stream.offer('people', message(100., frame_id='a'), 100.1)
        self.assertIsNone(self.stream.offer('matches', message(100., frame_id='b'), 100.1))
        self.appearance.assert_not_called()

    def test_body_never_establishes_identity(self):
        for index in range(20):
            self.triplet(100.+index*.1, face=False)
            self.assertIsNone(self.stream.observed(100.+index*.1+.01))

    def test_only_three_distinct_face_frames_establish_hint(self):
        for _ in range(3):
            self.triplet(100.)
        self.assertEqual(1, self.stream.tracker.face_hits)
        self.assertIsNone(self.stream.observed(100.1))
        self.triplet(100.1)
        self.assertIsNone(self.stream.observed(100.11))
        self.triplet(100.2)
        self.assertEqual('face_confirmed', self.stream.observed(100.21)['state'])

    def test_hint_has_body_position_but_no_face_or_movement_fields(self):
        self.seed()
        self.triplet(100.4, face=False)
        hint = self.stream.observed(100.45)
        self.assertEqual('body_continuity', hint['state'])
        for actual, expected in zip(hint['body_box'], BODY):
            self.assertAlmostEqual(expected, actual)
        self.assertAlmostEqual(.25, hint['face_age_seconds'])
        self.assertNotIn('confirmed', hint)
        self.assertNotIn('box_height_fraction', hint)
        self.assertNotIn('center_x_fraction', hint)
        self.assertNotIn('target_observation', hint)
        self.assertNotIn('appearance', hint)

    def test_silent_stream_does_not_keep_fresh_position(self):
        self.seed()
        self.assertIsNone(self.stream.observed(101.))

    def test_missing_matches_stream_does_not_reuse_old_face_array(self):
        self.seed()
        self.stream.offer('image', message(100.8), 100.9)
        self.stream.offer('people', message(100.8, [detection(BODY, 'person')]), 100.9)
        self.assertEqual(3, self.stream.tracker.face_hits)
        self.assertEqual(100.2, self.stream.tracker.face_at)
        self.assertIsNone(self.stream.observed(101.))

    def test_missing_body_hides_position_but_can_recover_briefly(self):
        self.seed()
        self.assertEqual('temporarily_missing', self.triplet(100.4, face=False, body=False)['state'])
        self.assertIsNone(self.stream.observed(100.42))
        self.triplet(100.6, face=False)
        self.assertEqual('body_continuity', self.stream.observed(100.62)['state'])

    def test_body_updates_cannot_extend_original_face_deadline(self):
        self.seed()
        for index in range(1, 20):
            self.triplet(100.2+index*.5, face=False)
        self.triplet(110.1, face=False)
        self.assertIsNotNone(self.stream.observed(110.15))
        # Body is recent, but face evidence has passed the ten-second limit.
        self.assertIsNone(self.stream.observed(110.21))

    def test_reset_discards_inflight_and_delayed_pre_reset_frames(self):
        self.seed()
        self.stream.offer('image', message(100.25), 100.26)
        self.stream.offer('people', message(100.25, [detection(BODY, 'person')]), 100.26)
        self.stream.reset(100.3)
        self.assertTrue(all(not cache for cache in self.stream.cache.values()))
        self.assertIsNone(self.stream.offer('matches', message(100.25, [detection(FACE, 'target_person')]), 100.31))
        self.assertIsNone(self.triplet(100.28, now=100.32))
        self.assertIsNone(self.triplet(100.3, now=100.32))
        self.assertIsNone(self.stream.observed(100.32))
        result = self.triplet(100.4)
        self.assertEqual('confirming_face', result['state'])
        self.assertEqual(1, result['face_hits'])

    def test_backwards_reset_cannot_reopen_old_source_barrier(self):
        self.stream.reset(100.5)
        self.stream.reset(100.3)
        self.assertIsNone(self.triplet(100.4, now=100.6))
        self.assertEqual(100.5, self.stream.not_before)

    def test_delayed_frame_does_not_get_freshness_from_receipt(self):
        result = self.triplet(100., now=101.)
        self.assertEqual('stale_frame', result['reason'])
        self.appearance.assert_not_called()
        self.assertIsNone(self.stream.observed(101.))

    def test_far_future_frame_does_not_establish_evidence(self):
        result = self.triplet(101., now=100.)
        self.assertEqual('stale_frame', result['reason'])
        self.assertIsNone(self.stream.observed(100.))

    def test_out_of_order_triplet_does_not_replace_current_hint(self):
        self.seed()
        result = self.triplet(100.1, now=100.3)
        self.assertEqual('ignored', result['state'])
        self.assertEqual(100.2, self.stream.observed(100.3)['source_time'])

    def test_each_unmatched_cache_has_fixed_capacity(self):
        self.stream = PersonContinuityStream(capacity=4)
        for index in range(10):
            self.stream.offer('image', message(100.+index*.01), 100.2)
        self.assertEqual(4, len(self.stream.cache['image']))
        self.assertEqual(0, len(self.stream.cache['people']))

    def test_invalid_image_clears_previous_hint(self):
        for field, value in (('width', 0), ('width', True), ('step', 1),
                             ('data', b''), ('encoding', 'mono8')):
            with self.subTest(field=field):
                self.stream = PersonContinuityStream()
                self.seed()
                frame = message(100.4)
                setattr(frame, field, value)
                self.stream.offer('image', frame, 100.41)
                self.stream.offer('people', message(100.4), 100.41)
                self.assertIsNone(self.stream.offer('matches', message(100.4), 100.41))
                self.assertIsNone(self.stream.observed(100.42))

    def test_layout_change_cannot_retain_old_person(self):
        self.seed()
        self.triplet(100.4, frame_id='replacement-camera')
        self.assertIsNone(self.stream.observed(100.41))
        self.assertEqual(0, self.stream.tracker.face_hits)

    def test_padded_color_image_is_accepted(self):
        frame = message(100.)
        frame.step = 304
        frame.data = bytes([100])*(frame.height*frame.step)
        self.stream.offer('image', frame, 100.01)
        self.stream.offer('people', message(100., [detection(BODY, 'person')]), 100.01)
        result = self.stream.offer('matches', message(100., [detection(FACE, 'target_person')]), 100.01)
        self.assertEqual('confirming_face', result['state'])

    def test_wrong_detection_class_does_not_seed_face_identity(self):
        for stamp in (100., 100.1, 100.2):
            self.stream.offer('image', message(stamp), stamp+.01)
            self.stream.offer('people', message(stamp, [detection(BODY, 'person')]), stamp+.01)
            self.stream.offer('matches', message(stamp, [detection(FACE, 'person')]), stamp+.01)
        self.assertIsNone(self.stream.observed(100.21))

    def test_ambiguous_people_clear_current_hint(self):
        self.seed()
        self.stream.offer('image', message(100.4), 100.41)
        self.stream.offer('people', message(100.4, [detection(BODY, 'person'), detection(BODY, 'person')]), 100.41)
        self.stream.offer('matches', message(100.4), 100.41)
        self.assertIsNone(self.stream.observed(100.42))

    def test_hint_copy_does_not_mutate_internal_body(self):
        self.seed()
        expected = self.stream.observed(100.21)['body_box']
        self.stream.observed(100.21)['body_box'][0] = .99
        self.assertEqual(expected, self.stream.observed(100.21)['body_box'])

    def test_retained_fresh_body_matches_observed(self):
        self.seed()
        self.assertEqual(self.stream.observed(100.22), self.stream.retained(100.22))

    def test_retained_missing_body_allows_hold_without_position(self):
        self.seed()
        self.triplet(100.4, face=False, body=False)
        result = self.stream.retained(100.42)
        self.assertEqual('temporarily_missing', result['state'])
        self.assertAlmostEqual(.22, result['body_gap_seconds'])
        self.assertNotIn('body_box', result)
        self.assertIsNone(self.stream.observed(100.42))

    def test_repeated_missing_frames_do_not_renew_hold_deadline(self):
        self.seed()
        calls_before = self.appearance.call_count
        for stamp in (100.4, 100.6, 100.8, 100.9):
            self.triplet(stamp, face=False, body=False)
        self.assertEqual(calls_before, self.appearance.call_count)
        self.assertEqual(3, self.stream.tracker.face_hits)
        self.assertEqual(100.2, self.stream.tracker.face_at)
        self.assertEqual(100.2, self.stream.tracker.seen_at)
        self.assertIsNotNone(self.stream.retained(100.94))
        self.assertIsNone(self.stream.retained(100.96))

    def test_pending_face_with_missing_body_cannot_offer_hold(self):
        self.triplet(100.)
        self.triplet(100.2, face=False, body=False)
        self.assertIsNone(self.stream.retained(100.21))

    def test_missing_body_hold_cannot_extend_face_anchor(self):
        self.seed()
        for index in range(1, 20):
            self.triplet(100.2+index*.5, face=False)
        self.triplet(110.1, face=False, body=False)
        self.assertIsNotNone(self.stream.retained(110.15))
        self.assertIsNone(self.stream.retained(110.21))

    def test_lost_or_invalid_time_never_retains_position(self):
        self.seed()
        self.triplet(101., face=False, body=False)
        self.assertIsNone(self.stream.retained(101.01))
        for value in (True, float('nan'), float('inf'), '100'):
            self.assertIsNone(self.stream.retained(value))

    def test_invalid_time_and_stream_name_are_rejected(self):
        for value in (True, float('nan'), float('inf'), '100'):
            with self.subTest(value=value):
                self.assertIsNone(self.stream.observed(value))
                with self.assertRaises(ValueError):
                    self.stream.reset(value)
                with self.assertRaises(ValueError):
                    self.stream.offer('image', message(100.), value)
        with self.assertRaises(ValueError):
            self.stream.offer('face', message(100.), 100.1)


if __name__ == '__main__':
    unittest.main()
