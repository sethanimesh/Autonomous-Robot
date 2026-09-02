#!/usr/bin/env python3
"""Rules for accepting or rejecting one incoming camera frame.

The node owns the pixels; this module owns the decisions. Keeping the two
apart means encoding checks, payload-size checks, and staleness rules are
covered by the repository test suite on a machine with no ROS, numpy, or
camera.
"""

REJECT_UNSUPPORTED_ENCODING = "unsupported_encoding"
REJECT_EMPTY_IMAGE = "empty_image"
REJECT_DIMENSION_MISMATCH = "dimension_mismatch"
REJECT_STALE_FRAME = "stale_frame"
REJECT_MALFORMED = "malformed_image"

SUPPORTED_ENCODINGS = ("bgr8", "rgb8")
CHANNELS = 3


def validate_image_message(
    encoding, width, height, payload_size, supported_encodings=SUPPORTED_ENCODINGS
):
    """Return None when a frame is usable, otherwise a short reason code.

    ``payload_size`` is the number of bytes actually carried by the message,
    which is checked against what the declared geometry requires. A frame that
    lies about its size is rejected rather than reshaped into garbage.
    """
    if not isinstance(encoding, str) or encoding not in supported_encodings:
        return REJECT_UNSUPPORTED_ENCODING
    try:
        width = int(width)
        height = int(height)
        payload_size = int(payload_size)
    except (TypeError, ValueError):
        return REJECT_MALFORMED
    if width <= 0 or height <= 0:
        return REJECT_EMPTY_IMAGE
    if payload_size <= 0:
        return REJECT_EMPTY_IMAGE
    if payload_size != width * height * CHANNELS:
        return REJECT_DIMENSION_MISMATCH
    return None


def frame_age_seconds(stamp_sec, stamp_nanosec, now_seconds):
    """Age of a stamped frame against a clock reading, in seconds."""
    stamped = float(stamp_sec) + float(stamp_nanosec) * 1e-9
    return float(now_seconds) - stamped


def is_frame_too_old(age_seconds, max_age_seconds):
    """True when a frame should be dropped rather than run through the model.

    An unstamped frame reports a very large age and is dropped; a frame from
    slightly in the future (clock jitter between publisher and subscriber) is
    treated as current rather than as an error.
    """
    return float(age_seconds) > float(max_age_seconds)
