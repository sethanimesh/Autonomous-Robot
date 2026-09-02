#!/usr/bin/env python3
"""Frame validation, failure counting, reconnect pacing, and status reporting.

The camera node keeps every pixel operation to itself and hands this module
plain numbers, so the recovery rules can be tested without a camera, OpenCV,
numpy, or ROS.
"""

from collections import deque

EXPECTED_CHANNELS = 3
EXPECTED_DTYPE = "uint8"

STATE_STARTING = "starting"
STATE_OPENING = "opening"
STATE_WARMING_UP = "warming_up"
STATE_STREAMING = "streaming"
STATE_RECONNECTING = "reconnecting"
STATE_STOPPED = "stopped"


def validate_frame(frame, expected_width, expected_height):
    """Return None when a frame is publishable, otherwise a short reason."""
    if frame is None:
        return "no_frame"
    shape = getattr(frame, "shape", None)
    if shape is None:
        return "no_shape"
    if len(shape) != 3:
        return "not_three_dimensional"
    height, width, channels = shape[0], shape[1], shape[2]
    if getattr(frame, "size", 0) == 0 or width == 0 or height == 0:
        return "empty_frame"
    if channels != EXPECTED_CHANNELS:
        return "unexpected_channel_count"
    if width != expected_width or height != expected_height:
        return "unexpected_dimensions"
    dtype = getattr(frame, "dtype", None)
    if dtype is not None and str(dtype) != EXPECTED_DTYPE:
        return "unexpected_dtype"
    return None


class CaptureHealth(object):
    """Bookkeeping for one camera stream.

    Every method that needs the current time takes it as an argument so tests
    can drive reconnect timing deterministically. All history is bounded.
    """

    def __init__(self, max_read_failures, reconnect_interval_sec, rate_window=120):
        if int(max_read_failures) < 1:
            raise ValueError("max_read_failures must be at least 1")
        if float(reconnect_interval_sec) <= 0.0:
            raise ValueError("reconnect_interval_sec must be positive")
        if int(rate_window) < 2:
            raise ValueError("rate_window must be at least 2")

        self.max_read_failures = int(max_read_failures)
        self.reconnect_interval_sec = float(reconnect_interval_sec)
        self.state = STATE_STARTING
        self.frames_published = 0
        self.frames_rejected = 0
        self.read_failures = 0
        self.consecutive_read_failures = 0
        self.open_count = 0
        self.open_failures = 0
        self.duplicate_frames = 0
        self.consecutive_duplicate_frames = 0
        self.last_error = ""
        self.mean_intensity = None
        self._frame_times = deque(maxlen=int(rate_window))
        self._last_signature = None
        self._last_frame_time = None
        self._last_open_attempt = None

    def set_state(self, state):
        changed = state != self.state
        self.state = state
        return changed

    def record_frame(self, now, signature=None, mean_intensity=None):
        """Count one published frame and track content variation."""
        self.frames_published += 1
        self.consecutive_read_failures = 0
        self._frame_times.append(float(now))
        self._last_frame_time = float(now)
        if mean_intensity is not None:
            self.mean_intensity = float(mean_intensity)
        if signature is not None:
            if self._last_signature is not None and signature == self._last_signature:
                self.duplicate_frames += 1
                self.consecutive_duplicate_frames += 1
            else:
                self.consecutive_duplicate_frames = 0
            self._last_signature = signature

    def record_rejected_frame(self, reason):
        """Count a frame that failed validation. Treated as a read failure."""
        self.frames_rejected += 1
        self.read_failures += 1
        self.consecutive_read_failures += 1
        self.last_error = reason

    def record_read_failure(self, reason):
        self.read_failures += 1
        self.consecutive_read_failures += 1
        self.last_error = reason

    def needs_reopen(self):
        return self.consecutive_read_failures >= self.max_read_failures

    def reconnect_ready(self, now):
        """True when enough time has passed to attempt another open."""
        if self._last_open_attempt is None:
            return True
        return (float(now) - self._last_open_attempt) >= self.reconnect_interval_sec

    def seconds_until_reconnect(self, now):
        if self._last_open_attempt is None:
            return 0.0
        remaining = self.reconnect_interval_sec - (float(now) - self._last_open_attempt)
        return remaining if remaining > 0.0 else 0.0

    def note_open_attempt(self, now):
        self._last_open_attempt = float(now)

    def note_opened(self):
        self.open_count += 1
        self.consecutive_read_failures = 0
        self.consecutive_duplicate_frames = 0
        self._last_signature = None
        self._frame_times.clear()

    def note_open_failed(self, reason):
        self.open_failures += 1
        self.last_error = reason

    def note_closed(self):
        self.consecutive_read_failures = 0
        self._last_signature = None
        self._frame_times.clear()

    def measured_fps(self):
        """Publication rate over the bounded recent-frame window."""
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        if span <= 0.0:
            return 0.0
        return (len(self._frame_times) - 1) / span

    def max_frame_gap(self):
        """Longest gap between publications inside the recent-frame window.

        This is measured where the frames are produced, so it reports
        publication stability rather than what a subscriber happened to
        receive over best-effort QoS.
        """
        if len(self._frame_times) < 2:
            return 0.0
        times = list(self._frame_times)
        return max(times[index + 1] - times[index] for index in range(len(times) - 1))

    def seconds_since_frame(self, now):
        if self._last_frame_time is None:
            return None
        return float(now) - self._last_frame_time

    def status(self, now):
        """Concise health snapshot for /camera/status."""
        since = self.seconds_since_frame(now)
        return {
            "state": self.state,
            "frames_published": self.frames_published,
            "frames_rejected": self.frames_rejected,
            "read_failures": self.read_failures,
            "consecutive_read_failures": self.consecutive_read_failures,
            "open_count": self.open_count,
            "open_failures": self.open_failures,
            "duplicate_frames": self.duplicate_frames,
            "consecutive_duplicate_frames": self.consecutive_duplicate_frames,
            "measured_fps": round(self.measured_fps(), 2),
            "max_frame_gap_sec": round(self.max_frame_gap(), 4),
            "seconds_since_frame": None if since is None else round(since, 2),
            "mean_intensity": (
                None if self.mean_intensity is None else round(self.mean_intensity, 2)
            ),
            "last_error": self.last_error,
        }
