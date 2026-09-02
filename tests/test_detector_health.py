import unittest

from robot.jetson.perception.detector_health import STATE_DETECTING
from robot.jetson.perception.detector_health import STATE_STALE_INPUT
from robot.jetson.perception.detector_health import STATE_STARTING
from robot.jetson.perception.detector_health import DetectorHealth
from robot.jetson.perception.detector_health import LatestFrameSlot
from robot.jetson.perception.detector_health import RollingLatency
from robot.jetson.perception.detector_health import RollingRate
from robot.jetson.perception.image_intake import REJECT_STALE_FRAME
from robot.jetson.perception.image_intake import REJECT_UNSUPPORTED_ENCODING


class LatestFrameSlotTests(unittest.TestCase):
    """The one-deep slot is what makes an unbounded backlog impossible."""

    def test_a_new_slot_is_empty(self):
        slot = LatestFrameSlot()

        self.assertIsNone(slot.take())
        self.assertFalse(slot.occupied)
        self.assertEqual(0, slot.dropped)

    def test_a_single_frame_is_returned(self):
        slot = LatestFrameSlot()

        self.assertFalse(slot.offer("frame-1"))

        self.assertEqual("frame-1", slot.take())

    def test_taking_empties_the_slot(self):
        slot = LatestFrameSlot()
        slot.offer("frame-1")
        slot.take()

        self.assertIsNone(slot.take())
        self.assertFalse(slot.occupied)

    def test_the_newest_frame_wins_and_the_old_one_is_dropped(self):
        slot = LatestFrameSlot()
        slot.offer("old")
        displaced = slot.offer("new")

        self.assertTrue(displaced)
        self.assertEqual("new", slot.take())
        self.assertEqual(1, slot.dropped)

    def test_a_burst_keeps_only_the_last_frame(self):
        slot = LatestFrameSlot()
        for index in range(10):
            slot.offer("frame-{0}".format(index))

        self.assertEqual("frame-9", slot.take())
        self.assertEqual(9, slot.dropped)

    def test_dropped_count_survives_taking(self):
        slot = LatestFrameSlot()
        slot.offer("a")
        slot.offer("b")
        slot.take()
        slot.offer("c")

        self.assertEqual(1, slot.dropped)

    def test_peek_does_not_consume(self):
        slot = LatestFrameSlot()
        slot.offer("frame")

        self.assertEqual("frame", slot.peek())
        self.assertEqual("frame", slot.take())

    def test_clear_discards_a_pending_frame(self):
        slot = LatestFrameSlot()
        slot.offer("stale")
        slot.clear()

        self.assertIsNone(slot.take())


class RollingRateTests(unittest.TestCase):
    def test_a_single_sample_has_no_rate(self):
        rate = RollingRate()
        rate.record(1.0)

        self.assertEqual(0.0, rate.rate())

    def test_even_samples_give_the_expected_rate(self):
        rate = RollingRate()
        for index in range(11):
            rate.record(index * 0.1)

        self.assertAlmostEqual(10.0, rate.rate())

    def test_the_window_is_bounded(self):
        rate = RollingRate(window=5)
        for index in range(100):
            rate.record(index * 0.1)

        self.assertEqual(5, len(rate))

    def test_identical_timestamps_do_not_divide_by_zero(self):
        rate = RollingRate()
        for _ in range(5):
            rate.record(2.0)

        self.assertEqual(0.0, rate.rate())

    def test_a_tiny_window_is_rejected(self):
        with self.assertRaises(ValueError):
            RollingRate(window=1)


class RollingLatencyTests(unittest.TestCase):
    def test_an_empty_window_reports_zero(self):
        latency = RollingLatency()

        self.assertEqual(0.0, latency.mean())
        self.assertEqual(0.0, latency.percentile(0.95))

    def test_the_mean_is_the_average_of_the_window(self):
        latency = RollingLatency()
        for value in (0.010, 0.020, 0.030):
            latency.record(value)

        self.assertAlmostEqual(0.020, latency.mean())

    def test_the_percentile_uses_nearest_rank(self):
        latency = RollingLatency()
        for value in range(1, 101):
            latency.record(value / 1000.0)

        self.assertAlmostEqual(0.095, latency.percentile(0.95))

    def test_the_median_is_available(self):
        latency = RollingLatency()
        for value in range(1, 101):
            latency.record(value / 1000.0)

        self.assertAlmostEqual(0.050, latency.percentile(0.5))

    def test_the_percentile_of_one_is_the_worst_in_the_window(self):
        latency = RollingLatency()
        for value in (0.01, 0.05, 0.02):
            latency.record(value)

        self.assertAlmostEqual(0.05, latency.percentile(1.0))

    def test_the_window_is_bounded_but_the_total_is_not(self):
        latency = RollingLatency(window=10)
        for index in range(1000):
            latency.record(0.001 * index)

        self.assertEqual(10, len(latency))
        self.assertEqual(1000, latency.total)

    def test_the_worst_value_survives_the_window_sliding(self):
        latency = RollingLatency(window=3)
        latency.record(5.0)
        for _ in range(10):
            latency.record(0.001)

        self.assertAlmostEqual(5.0, latency.worst)

    def test_a_negative_latency_is_rejected(self):
        with self.assertRaises(ValueError):
            RollingLatency().record(-0.1)

    def test_an_out_of_range_percentile_is_rejected(self):
        latency = RollingLatency()
        latency.record(0.01)

        with self.assertRaises(ValueError):
            latency.percentile(0.0)
        with self.assertRaises(ValueError):
            latency.percentile(1.5)


class DetectorHealthStateTests(unittest.TestCase):
    def test_a_new_detector_is_starting(self):
        self.assertEqual(STATE_STARTING, DetectorHealth("m", "p").state)

    def test_a_state_change_is_reported_once(self):
        health = DetectorHealth("m", "p")

        self.assertTrue(health.set_state(STATE_DETECTING))
        self.assertFalse(health.set_state(STATE_DETECTING))

    def test_input_is_not_stale_before_any_frame_arrives(self):
        health = DetectorHealth("m", "p")

        self.assertFalse(health.input_is_stale(1000.0, 2.0))

    def test_input_becomes_stale_after_the_timeout(self):
        health = DetectorHealth("m", "p")
        health.record_frame(100.0)

        self.assertFalse(health.input_is_stale(101.9, 2.0))
        self.assertTrue(health.input_is_stale(102.1, 2.0))

    def test_a_resumed_stream_clears_the_rate_history(self):
        health = DetectorHealth("m", "p")
        for index in range(10):
            health.record_frame(index * 0.1)
        self.assertGreater(health.input_rate(), 0.0)

        health.note_input_resumed()

        self.assertEqual(0.0, health.input_rate())

    def test_seconds_since_frame_tracks_the_last_arrival(self):
        health = DetectorHealth("m", "p")
        health.record_frame(50.0)

        self.assertAlmostEqual(1.5, health.seconds_since_frame(51.5))


class DetectorHealthStatusTests(unittest.TestCase):
    def build(self):
        health = DetectorHealth("yolox_tiny", "tensorrt_fp16")
        health.note_model_loaded("tensorrt_fp16", 1.25, 0.15)
        health.set_state(STATE_DETECTING)
        for index in range(10):
            health.record_frame(index * 0.1)
            health.record_inference(index * 0.1, 0.020, 1)
        return health

    def test_status_reports_the_model_and_provider(self):
        status = self.build().status(1.0)

        self.assertEqual("yolox_tiny", status["model"])
        self.assertEqual("tensorrt_fp16", status["provider"])
        self.assertEqual(STATE_DETECTING, status["state"])

    def test_status_reports_every_required_field(self):
        status = self.build().status(1.0)

        for field in (
            "model",
            "provider",
            "last_frame_age_sec",
            "input_rate_hz",
            "inference_rate_hz",
            "mean_latency_ms",
            "p95_latency_ms",
            "recent_person_count",
            "frames_dropped",
            "inference_errors",
        ):
            self.assertIn(field, status)

    def test_status_reports_measured_rates(self):
        status = self.build().status(1.0)

        self.assertAlmostEqual(10.0, status["input_rate_hz"])
        self.assertAlmostEqual(10.0, status["inference_rate_hz"])

    def test_status_reports_latency_in_milliseconds(self):
        status = self.build().status(1.0)

        self.assertAlmostEqual(20.0, status["mean_latency_ms"])

    def test_status_carries_the_dropped_frame_count(self):
        status = self.build().status(1.0, dropped_frames=7)

        self.assertEqual(7, status["frames_dropped"])

    def test_status_reports_the_recent_person_count(self):
        health = self.build()
        health.record_inference(2.0, 0.02, 3)

        self.assertEqual(3, health.status(2.0)["recent_person_count"])

    def test_status_reports_model_load_and_warmup_time(self):
        status = self.build().status(1.0)

        self.assertAlmostEqual(1.25, status["model_load_sec"])
        self.assertAlmostEqual(0.15, status["warmup_sec"])

    def test_a_fresh_detector_reports_no_frame_age(self):
        status = DetectorHealth("m", "p").status(1.0)

        self.assertIsNone(status["last_frame_age_sec"])
        self.assertEqual(0, status["frames_received"])

    def test_extra_fields_are_merged(self):
        status = self.build().status(1.0, extra={"image_topic": "/camera/image_raw"})

        self.assertEqual("/camera/image_raw", status["image_topic"])

    def test_status_is_json_serialisable(self):
        import json

        json.dumps(self.build().status(1.0, dropped_frames=2))


class DetectorHealthCountingTests(unittest.TestCase):
    def test_rejections_are_counted_by_reason(self):
        health = DetectorHealth("m", "p")
        health.record_rejected_frame(REJECT_UNSUPPORTED_ENCODING)
        health.record_rejected_frame(REJECT_UNSUPPORTED_ENCODING)
        health.record_rejected_frame(REJECT_STALE_FRAME)

        status = health.status(1.0)

        self.assertEqual(3, status["frames_rejected"])
        self.assertEqual(2, status["reject_reasons"][REJECT_UNSUPPORTED_ENCODING])

    def test_stale_frames_get_their_own_counter(self):
        health = DetectorHealth("m", "p")
        health.record_rejected_frame(REJECT_STALE_FRAME)
        health.record_rejected_frame(REJECT_UNSUPPORTED_ENCODING)

        self.assertEqual(1, health.frames_stale)
        self.assertEqual(2, health.frames_rejected)

    def test_inference_errors_are_counted_and_recorded(self):
        health = DetectorHealth("m", "p")
        health.record_inference_error("RuntimeError: cuda failure")
        health.record_inference_error("RuntimeError: cuda failure")

        status = health.status(1.0)

        self.assertEqual(2, status["inference_errors"])
        self.assertIn("cuda failure", status["last_error"])

    def test_an_inference_error_does_not_count_as_an_inference(self):
        health = DetectorHealth("m", "p")
        health.record_inference_error("boom")

        self.assertEqual(0, health.inferences)
        self.assertEqual(0, health.detections_published)
