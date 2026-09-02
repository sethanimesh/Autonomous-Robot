import unittest

from robot.jetson.camera.camera_config import CameraConfig
from robot.jetson.camera.camera_config import CameraConfigError
from robot.jetson.camera.camera_config import PARAMETER_DEFAULTS


class CameraConfigDefaultsTests(unittest.TestCase):
    def test_defaults_are_valid(self):
        config = CameraConfig.from_mapping({})

        self.assertEqual("/dev/video0", config.video_device)
        self.assertEqual(640, config.image_width)
        self.assertEqual(480, config.image_height)
        self.assertEqual("camera_optical_frame", config.frame_id)
        self.assertEqual("MJPG", config.fourcc)

    def test_every_default_round_trips(self):
        config = CameraConfig.from_mapping(dict(PARAMETER_DEFAULTS))

        self.assertEqual(PARAMETER_DEFAULTS, config.as_dict())

    def test_timer_period_matches_requested_rate(self):
        config = CameraConfig.from_mapping({"requested_fps": 25.0})

        self.assertAlmostEqual(0.04, config.capture_timer_period_sec())

    def test_step_bytes_covers_three_channels(self):
        config = CameraConfig.from_mapping({"image_width": 320})

        self.assertEqual(960, config.step_bytes())


class CaptureTargetTests(unittest.TestCase):
    def test_device_path_is_used_verbatim(self):
        config = CameraConfig.from_mapping({"video_device": "/dev/video2"})

        self.assertEqual("/dev/video2", config.capture_target())

    def test_numeric_device_becomes_an_index(self):
        config = CameraConfig.from_mapping({"video_device": "1"})

        self.assertEqual(1, config.capture_target())

    def test_surrounding_whitespace_is_trimmed(self):
        config = CameraConfig.from_mapping({"video_device": " /dev/video0 "})

        self.assertEqual("/dev/video0", config.video_device)


class CameraConfigValidationTests(unittest.TestCase):
    def assert_rejected(self, **overrides):
        with self.assertRaises(CameraConfigError):
            CameraConfig.from_mapping(overrides)

    def test_unknown_parameter_is_rejected(self):
        self.assert_rejected(exposure_mode="auto")

    def test_empty_device_is_rejected(self):
        self.assert_rejected(video_device="   ")

    def test_zero_and_negative_dimensions_are_rejected(self):
        self.assert_rejected(image_width=0)
        self.assert_rejected(image_height=-480)

    def test_absurd_dimensions_are_rejected(self):
        self.assert_rejected(image_width=100000)

    def test_non_positive_frame_rate_is_rejected(self):
        self.assert_rejected(requested_fps=0.0)
        self.assert_rejected(requested_fps=-30.0)

    def test_absurd_frame_rate_is_rejected(self):
        self.assert_rejected(requested_fps=1000.0)

    def test_fourcc_must_be_four_characters(self):
        self.assert_rejected(fourcc="MJP")
        self.assert_rejected(fourcc="MJPEG")

    def test_frame_id_must_be_present_and_relative(self):
        self.assert_rejected(frame_id="")
        self.assert_rejected(frame_id="/camera_optical_frame")

    def test_reconnect_interval_must_be_positive(self):
        self.assert_rejected(reconnect_interval_sec=0.0)
        self.assert_rejected(reconnect_interval_sec=-1.0)

    def test_reconnect_interval_must_stay_bounded(self):
        self.assert_rejected(reconnect_interval_sec=600.0)

    def test_zero_warmup_is_allowed_but_negative_is_not(self):
        self.assertEqual(0.0, CameraConfig.from_mapping({"warmup_sec": 0.0}).warmup_sec)
        self.assert_rejected(warmup_sec=-0.5)

    def test_read_failure_budget_must_allow_at_least_one_failure(self):
        self.assert_rejected(max_read_failures=0)

    def test_read_failure_pause_must_stay_short(self):
        self.assert_rejected(read_failure_pause_sec=5.0)

    def test_status_interval_must_be_positive(self):
        self.assert_rejected(status_interval_sec=0.0)

    def test_non_numeric_values_are_rejected(self):
        self.assert_rejected(image_width="wide")
        self.assert_rejected(requested_fps="fast")

    def test_missing_calibration_file_defaults_to_empty(self):
        config = CameraConfig.from_mapping({"calibration_file": None})

        self.assertEqual("", config.calibration_file)


if __name__ == "__main__":
    unittest.main()
