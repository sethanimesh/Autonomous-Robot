#!/usr/bin/env python3
"""Configuration for the Echora ROS 2 USB camera node.

This module is deliberately free of ROS, OpenCV, and numpy imports so the
validation rules can be tested on any machine.
"""

PARAMETER_DEFAULTS = {
    "video_device": "/dev/video0",
    "image_width": 640,
    "image_height": 480,
    "requested_fps": 30.0,
    "fourcc": "MJPG",
    "frame_id": "camera_optical_frame",
    "reconnect_interval_sec": 2.0,
    "warmup_sec": 2.0,
    "max_read_failures": 15,
    "read_failure_pause_sec": 0.02,
    "status_interval_sec": 5.0,
    "calibration_file": "",
}

MAX_DIMENSION = 8000
MAX_FPS = 240.0
MAX_RECONNECT_INTERVAL_SEC = 60.0
MAX_WARMUP_SEC = 30.0


class CameraConfigError(ValueError):
    """Raised when a camera parameter would produce an unusable capture."""


def _positive_int(name, value, maximum):
    try:
        number = int(value)
    except (TypeError, ValueError):
        raise CameraConfigError("{0} must be an integer, got {1!r}".format(name, value))
    if number <= 0:
        raise CameraConfigError("{0} must be positive, got {1}".format(name, number))
    if number > maximum:
        raise CameraConfigError(
            "{0} must be {1} or less, got {2}".format(name, maximum, number)
        )
    return number


def _bounded_float(name, value, minimum, maximum, allow_minimum=False):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise CameraConfigError("{0} must be a number, got {1!r}".format(name, value))
    if number != number:
        raise CameraConfigError("{0} must be a real number, got NaN".format(name))
    if number < minimum or (number == minimum and not allow_minimum):
        comparison = "at least" if allow_minimum else "greater than"
        raise CameraConfigError(
            "{0} must be {1} {2}, got {3}".format(name, comparison, minimum, number)
        )
    if number > maximum:
        raise CameraConfigError(
            "{0} must be {1} or less, got {2}".format(name, maximum, number)
        )
    return number


class CameraConfig(object):
    """Validated settings for one USB camera stream."""

    def __init__(
        self,
        video_device=PARAMETER_DEFAULTS["video_device"],
        image_width=PARAMETER_DEFAULTS["image_width"],
        image_height=PARAMETER_DEFAULTS["image_height"],
        requested_fps=PARAMETER_DEFAULTS["requested_fps"],
        fourcc=PARAMETER_DEFAULTS["fourcc"],
        frame_id=PARAMETER_DEFAULTS["frame_id"],
        reconnect_interval_sec=PARAMETER_DEFAULTS["reconnect_interval_sec"],
        warmup_sec=PARAMETER_DEFAULTS["warmup_sec"],
        max_read_failures=PARAMETER_DEFAULTS["max_read_failures"],
        read_failure_pause_sec=PARAMETER_DEFAULTS["read_failure_pause_sec"],
        status_interval_sec=PARAMETER_DEFAULTS["status_interval_sec"],
        calibration_file=PARAMETER_DEFAULTS["calibration_file"],
    ):
        if not isinstance(video_device, str) or not video_device.strip():
            raise CameraConfigError("video_device must be a non-empty string")
        self.video_device = video_device.strip()

        self.image_width = _positive_int("image_width", image_width, MAX_DIMENSION)
        self.image_height = _positive_int("image_height", image_height, MAX_DIMENSION)
        self.requested_fps = _bounded_float("requested_fps", requested_fps, 0.0, MAX_FPS)

        if not isinstance(fourcc, str) or len(fourcc) != 4:
            raise CameraConfigError(
                "fourcc must be exactly four characters, got {0!r}".format(fourcc)
            )
        self.fourcc = fourcc

        if not isinstance(frame_id, str) or not frame_id.strip():
            raise CameraConfigError("frame_id must be a non-empty string")
        if frame_id.startswith("/"):
            raise CameraConfigError(
                "frame_id must not start with '/' in ROS 2, got {0!r}".format(frame_id)
            )
        self.frame_id = frame_id.strip()

        self.reconnect_interval_sec = _bounded_float(
            "reconnect_interval_sec",
            reconnect_interval_sec,
            0.0,
            MAX_RECONNECT_INTERVAL_SEC,
        )
        self.warmup_sec = _bounded_float(
            "warmup_sec", warmup_sec, 0.0, MAX_WARMUP_SEC, allow_minimum=True
        )
        self.max_read_failures = _positive_int("max_read_failures", max_read_failures, 10000)
        self.read_failure_pause_sec = _bounded_float(
            "read_failure_pause_sec", read_failure_pause_sec, 0.0, 1.0, allow_minimum=True
        )
        self.status_interval_sec = _bounded_float(
            "status_interval_sec", status_interval_sec, 0.0, 3600.0
        )

        if calibration_file is None:
            calibration_file = ""
        if not isinstance(calibration_file, str):
            raise CameraConfigError("calibration_file must be a string or empty")
        self.calibration_file = calibration_file.strip()

    @classmethod
    def from_mapping(cls, mapping):
        """Build a config from a parameter mapping, rejecting unknown names."""
        unknown = sorted(set(mapping) - set(PARAMETER_DEFAULTS))
        if unknown:
            raise CameraConfigError(
                "unknown camera parameters: {0}".format(", ".join(unknown))
            )
        merged = dict(PARAMETER_DEFAULTS)
        merged.update(mapping)
        return cls(**merged)

    def capture_target(self):
        """Return the OpenCV capture target: an index for digits, else a path."""
        if self.video_device.isdigit():
            return int(self.video_device)
        return self.video_device

    def capture_timer_period_sec(self):
        """Timer period used to poll the blocking camera read."""
        return 1.0 / self.requested_fps

    def step_bytes(self, channels=3):
        """Row stride of a published image in bytes."""
        return self.image_width * channels

    def as_dict(self):
        return {
            "video_device": self.video_device,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "requested_fps": self.requested_fps,
            "fourcc": self.fourcc,
            "frame_id": self.frame_id,
            "reconnect_interval_sec": self.reconnect_interval_sec,
            "warmup_sec": self.warmup_sec,
            "max_read_failures": self.max_read_failures,
            "read_failure_pause_sec": self.read_failure_pause_sec,
            "status_interval_sec": self.status_interval_sec,
            "calibration_file": self.calibration_file,
        }
