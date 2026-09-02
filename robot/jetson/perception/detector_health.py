#!/usr/bin/env python3
"""Frame arbitration, rate and latency measurement, and status reporting.

Like the camera node's health module, every method that needs the current
time takes it as an argument so the tests can drive timing deterministically,
and every history is bounded so a long run cannot grow without limit. Nothing
here imports ROS, numpy, OpenCV, or TensorRT.
"""

import math
from collections import deque

STATE_STARTING = "starting"
STATE_LOADING_MODEL = "loading_model"
STATE_WAITING_FOR_FRAMES = "waiting_for_frames"
STATE_DETECTING = "detecting"
STATE_STALE_INPUT = "stale_input"
STATE_MODEL_ERROR = "model_error"
STATE_STOPPED = "stopped"

# The reject reason codes live with the intake rules that produce them.
try:
    from image_intake import REJECT_STALE_FRAME
except ImportError:  # imported as part of a package rather than flat on the Jetson
    from .image_intake import REJECT_STALE_FRAME


class LatestFrameSlot(object):
    """A one-deep mailbox that always holds the newest frame.

    Offering a second frame before the first is taken overwrites it and counts
    a drop. This is what keeps inference from ever building an unbounded
    backlog: the queue depth is one by construction, not by policy.
    """

    def __init__(self):
        self._item = None
        self.dropped = 0

    def offer(self, item):
        """Store the newest item. Returns True when it displaced an unread one."""
        displaced = self._item is not None
        if displaced:
            self.dropped += 1
        self._item = item
        return displaced

    def take(self):
        """Remove and return the pending item, or None when empty."""
        item = self._item
        self._item = None
        return item

    def peek(self):
        return self._item

    @property
    def occupied(self):
        return self._item is not None

    def clear(self):
        self._item = None


class RollingRate(object):
    """Event rate over a bounded window of timestamps."""

    def __init__(self, window=120):
        if int(window) < 2:
            raise ValueError("window must be at least 2")
        self._times = deque(maxlen=int(window))

    def record(self, now):
        self._times.append(float(now))

    def rate(self):
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0.0:
            return 0.0
        return (len(self._times) - 1) / span

    def clear(self):
        self._times.clear()

    def __len__(self):
        return len(self._times)


class RollingLatency(object):
    """Bounded latency window with mean and percentile summaries."""

    def __init__(self, window=120):
        if int(window) < 2:
            raise ValueError("window must be at least 2")
        self._values = deque(maxlen=int(window))
        self.total = 0
        self.worst = 0.0

    def record(self, seconds):
        value = float(seconds)
        if value < 0.0:
            raise ValueError("latency must not be negative")
        self._values.append(value)
        self.total += 1
        if value > self.worst:
            self.worst = value

    def mean(self):
        if not self._values:
            return 0.0
        return sum(self._values) / len(self._values)

    def percentile(self, fraction):
        """Nearest-rank percentile over the current window."""
        if not 0.0 < fraction <= 1.0:
            raise ValueError("fraction must be in (0, 1]")
        if not self._values:
            return 0.0
        ordered = sorted(self._values)
        # Nearest-rank: the smallest value at or above the requested fraction
        # of the window. round() would use banker's rounding on an exact .5.
        rank = int(math.ceil(fraction * len(ordered)))
        rank = min(max(rank, 1), len(ordered))
        return ordered[rank - 1]

    def clear(self):
        self._values.clear()

    def __len__(self):
        return len(self._values)


class DetectorHealth(object):
    """Counters and rates behind /perception/status."""

    def __init__(self, model_name, provider, latency_window=120, rate_window=120):
        self.model_name = str(model_name)
        self.provider = str(provider)
        self.state = STATE_STARTING
        self.last_error = ""

        self.frames_received = 0
        self.frames_rejected = 0
        self.frames_stale = 0
        self.inferences = 0
        self.inference_errors = 0
        self.detections_published = 0
        self.recent_person_count = 0
        self.annotated_published = 0
        self.model_load_seconds = None
        self.warmup_seconds = None
        self.reject_reasons = {}

        self._input_rate = RollingRate(rate_window)
        self._inference_rate = RollingRate(rate_window)
        self._latency = RollingLatency(latency_window)
        self._last_frame_time = None
        self._last_inference_time = None

    # -- state ---------------------------------------------------------

    def set_state(self, state):
        """Set the state, returning True when it actually changed."""
        changed = state != self.state
        self.state = state
        return changed

    def note_model_loaded(self, provider, load_seconds, warmup_seconds):
        self.provider = str(provider)
        self.model_load_seconds = float(load_seconds)
        self.warmup_seconds = float(warmup_seconds)

    # -- input ---------------------------------------------------------

    def record_frame(self, now):
        self.frames_received += 1
        self._input_rate.record(now)
        self._last_frame_time = float(now)

    def record_rejected_frame(self, reason):
        self.frames_rejected += 1
        if reason == REJECT_STALE_FRAME:
            self.frames_stale += 1
        self.reject_reasons[reason] = self.reject_reasons.get(reason, 0) + 1
        self.last_error = reason

    # -- inference -----------------------------------------------------

    def record_inference(self, now, latency_seconds, person_count):
        self.inferences += 1
        self._inference_rate.record(now)
        self._latency.record(latency_seconds)
        self._last_inference_time = float(now)
        self.recent_person_count = int(person_count)
        self.detections_published += 1

    def record_inference_error(self, reason):
        self.inference_errors += 1
        self.last_error = str(reason)

    def record_annotated(self):
        self.annotated_published += 1

    # -- derived -------------------------------------------------------

    def seconds_since_frame(self, now):
        if self._last_frame_time is None:
            return None
        return float(now) - self._last_frame_time

    def seconds_since_inference(self, now):
        if self._last_inference_time is None:
            return None
        return float(now) - self._last_inference_time

    def input_rate(self):
        return self._input_rate.rate()

    def inference_rate(self):
        return self._inference_rate.rate()

    def mean_latency(self):
        return self._latency.mean()

    def latency_percentile(self, fraction=0.95):
        return self._latency.percentile(fraction)

    def input_is_stale(self, now, timeout_sec):
        """True when frames have stopped arriving for longer than the timeout."""
        since = self.seconds_since_frame(now)
        if since is None:
            return False
        return since > timeout_sec

    def note_input_resumed(self):
        """Clear rate history so a resumed stream is not averaged with the gap."""
        self._input_rate.clear()
        self._inference_rate.clear()

    # -- reporting -----------------------------------------------------

    def status(self, now, dropped_frames=0, extra=None):
        since_frame = self.seconds_since_frame(now)
        since_inference = self.seconds_since_inference(now)
        payload = {
            "state": self.state,
            "model": self.model_name,
            "provider": self.provider,
            "frames_received": self.frames_received,
            "frames_rejected": self.frames_rejected,
            "frames_stale": self.frames_stale,
            "frames_dropped": int(dropped_frames),
            "inferences": self.inferences,
            "inference_errors": self.inference_errors,
            "detections_published": self.detections_published,
            "annotated_published": self.annotated_published,
            "recent_person_count": self.recent_person_count,
            "input_rate_hz": round(self.input_rate(), 2),
            "inference_rate_hz": round(self.inference_rate(), 2),
            "mean_latency_ms": round(self.mean_latency() * 1000.0, 2),
            "p95_latency_ms": round(self.latency_percentile(0.95) * 1000.0, 2),
            "worst_latency_ms": round(self._latency.worst * 1000.0, 2),
            "last_frame_age_sec": (
                None if since_frame is None else round(since_frame, 3)
            ),
            "last_inference_age_sec": (
                None if since_inference is None else round(since_inference, 3)
            ),
            "model_load_sec": (
                None if self.model_load_seconds is None else round(self.model_load_seconds, 3)
            ),
            "warmup_sec": (
                None if self.warmup_seconds is None else round(self.warmup_seconds, 3)
            ),
            "reject_reasons": dict(self.reject_reasons),
            "last_error": self.last_error,
        }
        if extra:
            payload.update(extra)
        return payload
