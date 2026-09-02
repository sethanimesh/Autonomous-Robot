import unittest

from robot.jetson.camera.frame_health import CaptureHealth
from robot.jetson.camera.frame_health import STATE_RECONNECTING
from robot.jetson.camera.frame_health import STATE_STREAMING
from robot.jetson.camera.frame_health import validate_frame


class FakeFrame(object):
    """The minimal surface the validator reads from an OpenCV frame."""

    def __init__(self, height, width, channels=3, dtype="uint8"):
        self.shape = (height, width, channels)
        self.size = height * width * channels
        self.dtype = dtype


class FrameValidationTests(unittest.TestCase):
    def test_a_correct_frame_is_accepted(self):
        self.assertIsNone(validate_frame(FakeFrame(480, 640), 640, 480))

    def test_missing_frame_is_rejected(self):
        self.assertEqual("no_frame", validate_frame(None, 640, 480))

    def test_object_without_shape_is_rejected(self):
        self.assertEqual("no_shape", validate_frame(object(), 640, 480))

    def test_grayscale_frame_is_rejected(self):
        frame = FakeFrame(480, 640)
        frame.shape = (480, 640)

        self.assertEqual("not_three_dimensional", validate_frame(frame, 640, 480))

    def test_zero_sized_frame_is_rejected(self):
        frame = FakeFrame(0, 0)

        self.assertEqual("empty_frame", validate_frame(frame, 640, 480))

    def test_frame_with_no_bytes_is_rejected(self):
        frame = FakeFrame(480, 640)
        frame.size = 0

        self.assertEqual("empty_frame", validate_frame(frame, 640, 480))

    def test_four_channel_frame_is_rejected(self):
        self.assertEqual(
            "unexpected_channel_count", validate_frame(FakeFrame(480, 640, 4), 640, 480)
        )

    def test_wrong_dimensions_are_rejected(self):
        self.assertEqual(
            "unexpected_dimensions", validate_frame(FakeFrame(720, 1280), 640, 480)
        )

    def test_wrong_dtype_is_rejected(self):
        frame = FakeFrame(480, 640, dtype="float32")

        self.assertEqual("unexpected_dtype", validate_frame(frame, 640, 480))


class FailureCountingTests(unittest.TestCase):
    def setUp(self):
        self.health = CaptureHealth(max_read_failures=3, reconnect_interval_sec=2.0)

    def test_construction_rejects_unusable_limits(self):
        with self.assertRaises(ValueError):
            CaptureHealth(0, 2.0)
        with self.assertRaises(ValueError):
            CaptureHealth(3, 0.0)

    def test_reopen_is_requested_only_after_the_budget_is_spent(self):
        self.health.record_read_failure("read_failed")
        self.health.record_read_failure("read_failed")
        self.assertFalse(self.health.needs_reopen())

        self.health.record_read_failure("read_failed")

        self.assertTrue(self.health.needs_reopen())
        self.assertEqual(3, self.health.read_failures)

    def test_a_good_frame_clears_the_consecutive_count(self):
        self.health.record_read_failure("read_failed")
        self.health.record_read_failure("read_failed")

        self.health.record_frame(1.0)

        self.assertEqual(0, self.health.consecutive_read_failures)
        self.assertFalse(self.health.needs_reopen())
        self.assertEqual(2, self.health.read_failures)

    def test_rejected_frames_also_drive_recovery(self):
        for _ in range(3):
            self.health.record_rejected_frame("unexpected_dimensions")

        self.assertTrue(self.health.needs_reopen())
        self.assertEqual(3, self.health.frames_rejected)
        self.assertEqual("unexpected_dimensions", self.health.last_error)

    def test_closing_clears_the_consecutive_count(self):
        for _ in range(3):
            self.health.record_read_failure("read_failed")

        self.health.note_closed()

        self.assertFalse(self.health.needs_reopen())


class ReconnectTimingTests(unittest.TestCase):
    def setUp(self):
        self.health = CaptureHealth(max_read_failures=3, reconnect_interval_sec=2.0)

    def test_first_attempt_is_immediate(self):
        self.assertTrue(self.health.reconnect_ready(100.0))
        self.assertEqual(0.0, self.health.seconds_until_reconnect(100.0))

    def test_retry_waits_for_the_configured_interval(self):
        self.health.note_open_attempt(100.0)

        self.assertFalse(self.health.reconnect_ready(101.9))
        self.assertAlmostEqual(0.1, self.health.seconds_until_reconnect(101.9))
        self.assertTrue(self.health.reconnect_ready(102.0))
        self.assertEqual(0.0, self.health.seconds_until_reconnect(102.0))

    def test_failed_attempts_are_counted_and_still_paced(self):
        self.health.note_open_attempt(100.0)
        self.health.note_open_failed("open_failed")
        self.health.note_open_attempt(102.0)
        self.health.note_open_failed("open_failed")

        self.assertEqual(2, self.health.open_failures)
        self.assertFalse(self.health.reconnect_ready(103.0))

    def test_successful_open_counts_and_resets_stream_state(self):
        self.health.record_read_failure("read_failed")
        self.health.record_frame(1.0, signature=b"a")

        self.health.note_open_attempt(100.0)
        self.health.note_opened()

        self.assertEqual(1, self.health.open_count)
        self.assertEqual(0, self.health.consecutive_read_failures)
        self.assertEqual(0.0, self.health.measured_fps())


class RateAndContentTests(unittest.TestCase):
    def setUp(self):
        self.health = CaptureHealth(max_read_failures=3, reconnect_interval_sec=2.0)

    def test_rate_is_unknown_until_two_frames_exist(self):
        self.assertEqual(0.0, self.health.measured_fps())
        self.health.record_frame(1.0)
        self.assertEqual(0.0, self.health.measured_fps())

    def test_rate_is_measured_over_the_window(self):
        for index in range(11):
            self.health.record_frame(index * 0.04)

        self.assertAlmostEqual(25.0, self.health.measured_fps())

    def test_frame_history_stays_bounded(self):
        health = CaptureHealth(3, 2.0, rate_window=5)

        for index in range(500):
            health.record_frame(index * 0.04)

        self.assertEqual(5, len(health._frame_times))
        self.assertEqual(500, health.frames_published)
        self.assertAlmostEqual(25.0, health.measured_fps())

    def test_identical_frames_are_reported_as_duplicates(self):
        self.health.record_frame(1.0, signature=b"same")
        self.health.record_frame(2.0, signature=b"same")
        self.health.record_frame(3.0, signature=b"same")

        self.assertEqual(2, self.health.duplicate_frames)
        self.assertEqual(2, self.health.consecutive_duplicate_frames)

    def test_changing_content_clears_the_duplicate_streak(self):
        self.health.record_frame(1.0, signature=b"same")
        self.health.record_frame(2.0, signature=b"same")

        self.health.record_frame(3.0, signature=b"different")

        self.assertEqual(1, self.health.duplicate_frames)
        self.assertEqual(0, self.health.consecutive_duplicate_frames)

    def test_max_gap_is_zero_before_two_frames(self):
        self.assertEqual(0.0, self.health.max_frame_gap())
        self.health.record_frame(1.0)
        self.assertEqual(0.0, self.health.max_frame_gap())

    def test_max_gap_finds_the_worst_publication_stall(self):
        for stamp in (0.0, 0.04, 0.08, 0.60, 0.64):
            self.health.record_frame(stamp)

        self.assertAlmostEqual(0.52, self.health.max_frame_gap())

    def test_max_gap_forgets_stalls_that_leave_the_window(self):
        health = CaptureHealth(3, 2.0, rate_window=3)
        for stamp in (0.0, 0.60, 0.64, 0.68, 0.72):
            health.record_frame(stamp)

        self.assertAlmostEqual(0.04, health.max_frame_gap())

    def test_seconds_since_frame_is_none_before_any_frame(self):
        self.assertIsNone(self.health.seconds_since_frame(10.0))

        self.health.record_frame(10.0)

        self.assertAlmostEqual(0.5, self.health.seconds_since_frame(10.5))


class StatusReportTests(unittest.TestCase):
    def setUp(self):
        self.health = CaptureHealth(max_read_failures=3, reconnect_interval_sec=2.0)

    def test_initial_status_is_honest_about_having_no_frames(self):
        status = self.health.status(0.0)

        self.assertEqual("starting", status["state"])
        self.assertEqual(0, status["frames_published"])
        self.assertEqual(0.0, status["measured_fps"])
        self.assertIsNone(status["seconds_since_frame"])
        self.assertIsNone(status["mean_intensity"])
        self.assertEqual("", status["last_error"])

    def test_streaming_status_reports_rate_and_content(self):
        self.health.set_state(STATE_STREAMING)
        for index in range(11):
            self.health.record_frame(
                index * 0.04, signature=bytes([index]), mean_intensity=105.5
            )

        status = self.health.status(0.4)

        self.assertEqual("streaming", status["state"])
        self.assertEqual(11, status["frames_published"])
        self.assertEqual(25.0, status["measured_fps"])
        self.assertEqual(0.04, status["max_frame_gap_sec"])
        self.assertEqual(105.5, status["mean_intensity"])
        self.assertEqual(0, status["duplicate_frames"])

    def test_failure_status_reports_the_reason_and_counts(self):
        self.health.record_read_failure("read_failed")
        self.health.set_state(STATE_RECONNECTING)

        status = self.health.status(1.0)

        self.assertEqual("reconnecting", status["state"])
        self.assertEqual(1, status["read_failures"])
        self.assertEqual(1, status["consecutive_read_failures"])
        self.assertEqual("read_failed", status["last_error"])

    def test_state_change_is_reported_once(self):
        self.assertTrue(self.health.set_state(STATE_STREAMING))
        self.assertFalse(self.health.set_state(STATE_STREAMING))

    def test_status_keys_are_stable(self):
        expected = {
            "state",
            "frames_published",
            "frames_rejected",
            "read_failures",
            "consecutive_read_failures",
            "open_count",
            "open_failures",
            "duplicate_frames",
            "consecutive_duplicate_frames",
            "measured_fps",
            "max_frame_gap_sec",
            "seconds_since_frame",
            "mean_intensity",
            "last_error",
        }

        self.assertEqual(expected, set(self.health.status(0.0)))


if __name__ == "__main__":
    unittest.main()
