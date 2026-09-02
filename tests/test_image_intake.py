import unittest

from robot.jetson.perception.image_intake import REJECT_DIMENSION_MISMATCH
from robot.jetson.perception.image_intake import REJECT_EMPTY_IMAGE
from robot.jetson.perception.image_intake import REJECT_MALFORMED
from robot.jetson.perception.image_intake import REJECT_UNSUPPORTED_ENCODING
from robot.jetson.perception.image_intake import SUPPORTED_ENCODINGS
from robot.jetson.perception.image_intake import frame_age_seconds
from robot.jetson.perception.image_intake import is_frame_too_old
from robot.jetson.perception.image_intake import validate_image_message

BGR_640x480 = 640 * 480 * 3


class ImageValidationTests(unittest.TestCase):
    def test_a_well_formed_bgr_frame_is_accepted(self):
        self.assertIsNone(validate_image_message("bgr8", 640, 480, BGR_640x480))

    def test_rgb_is_also_accepted(self):
        self.assertIsNone(validate_image_message("rgb8", 640, 480, BGR_640x480))

    def test_every_advertised_encoding_really_is_accepted(self):
        for encoding in SUPPORTED_ENCODINGS:
            self.assertIsNone(validate_image_message(encoding, 8, 8, 8 * 8 * 3))

    def test_mono8_is_rejected_rather_than_misread(self):
        self.assertEqual(
            REJECT_UNSUPPORTED_ENCODING, validate_image_message("mono8", 640, 480, 640 * 480)
        )

    def test_compressed_encodings_are_rejected(self):
        self.assertEqual(
            REJECT_UNSUPPORTED_ENCODING,
            validate_image_message("jpeg", 640, 480, 12345),
        )

    def test_an_empty_encoding_is_rejected(self):
        self.assertEqual(
            REJECT_UNSUPPORTED_ENCODING, validate_image_message("", 640, 480, BGR_640x480)
        )

    def test_a_non_string_encoding_is_rejected(self):
        self.assertEqual(
            REJECT_UNSUPPORTED_ENCODING, validate_image_message(None, 640, 480, BGR_640x480)
        )

    def test_zero_width_is_rejected(self):
        self.assertEqual(REJECT_EMPTY_IMAGE, validate_image_message("bgr8", 0, 480, 0))

    def test_zero_height_is_rejected(self):
        self.assertEqual(REJECT_EMPTY_IMAGE, validate_image_message("bgr8", 640, 0, 0))

    def test_negative_dimensions_are_rejected(self):
        self.assertEqual(REJECT_EMPTY_IMAGE, validate_image_message("bgr8", -640, 480, 100))

    def test_an_empty_payload_is_rejected(self):
        self.assertEqual(REJECT_EMPTY_IMAGE, validate_image_message("bgr8", 640, 480, 0))

    def test_a_truncated_payload_is_rejected(self):
        self.assertEqual(
            REJECT_DIMENSION_MISMATCH,
            validate_image_message("bgr8", 640, 480, BGR_640x480 - 1),
        )

    def test_an_oversized_payload_is_rejected(self):
        self.assertEqual(
            REJECT_DIMENSION_MISMATCH,
            validate_image_message("bgr8", 640, 480, BGR_640x480 + 1),
        )

    def test_a_payload_sized_for_one_channel_is_rejected(self):
        self.assertEqual(
            REJECT_DIMENSION_MISMATCH, validate_image_message("bgr8", 640, 480, 640 * 480)
        )

    def test_non_numeric_geometry_is_rejected(self):
        self.assertEqual(
            REJECT_MALFORMED, validate_image_message("bgr8", "wide", 480, BGR_640x480)
        )

    def test_a_one_pixel_image_is_valid(self):
        self.assertIsNone(validate_image_message("bgr8", 1, 1, 3))


class FrameAgeTests(unittest.TestCase):
    def test_age_combines_seconds_and_nanoseconds(self):
        self.assertAlmostEqual(0.5, frame_age_seconds(100, 500000000, 101.0))

    def test_a_frame_stamped_now_has_no_age(self):
        self.assertAlmostEqual(0.0, frame_age_seconds(100, 0, 100.0))

    def test_a_frame_from_the_future_reports_a_negative_age(self):
        self.assertAlmostEqual(-0.25, frame_age_seconds(100, 250000000, 100.0))


class StaleFrameTests(unittest.TestCase):
    """A frame older than the budget is dropped instead of being detected on."""

    def test_a_fresh_frame_is_processed(self):
        self.assertFalse(is_frame_too_old(0.05, 0.5))

    def test_a_frame_at_the_limit_is_processed(self):
        self.assertFalse(is_frame_too_old(0.5, 0.5))

    def test_a_frame_past_the_limit_is_dropped(self):
        self.assertTrue(is_frame_too_old(0.51, 0.5))

    def test_a_very_old_frame_is_dropped(self):
        self.assertTrue(is_frame_too_old(30.0, 0.5))

    def test_an_unstamped_frame_is_dropped(self):
        # A zero stamp against a real clock produces an enormous age.
        age = frame_age_seconds(0, 0, 1772000000.0)

        self.assertTrue(is_frame_too_old(age, 0.5))

    def test_clock_jitter_from_the_future_is_not_treated_as_stale(self):
        self.assertFalse(is_frame_too_old(-0.01, 0.5))
