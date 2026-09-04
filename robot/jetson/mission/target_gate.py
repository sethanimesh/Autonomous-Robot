"""Fuse target-recognizer status with the newest target bounding boxes."""

import math

try:
    from .mission_types import TargetObservation
except ImportError:
    from mission_types import TargetObservation


class TargetGateError(ValueError):
    pass


def build_target_observation(
    recognition_status,
    status_age_seconds,
    match_frame_age_seconds,
    box_heights_pixels,
    image_height_pixels=480,
    maximum_status_age_seconds=2.5,
):
    """Return one conservative observation for the mission state machine."""
    values = (
        status_age_seconds,
        match_frame_age_seconds,
        image_height_pixels,
        maximum_status_age_seconds,
    )
    if not all(isinstance(value, (int, float)) for value in values):
        raise TargetGateError("target timing and image height must be numeric")
    if not all(math.isfinite(float(value)) for value in values):
        raise TargetGateError("target timing and image height must be finite")
    if image_height_pixels <= 0 or maximum_status_age_seconds <= 0:
        raise TargetGateError("image height and status age limit must be positive")
    if status_age_seconds < 0 or status_age_seconds > maximum_status_age_seconds:
        return TargetObservation(
            False,
            max(float(status_age_seconds), float(match_frame_age_seconds)),
            0.0,
        )
    if not isinstance(recognition_status, dict):
        raise TargetGateError("recognition status must be an object")

    confirmation = recognition_status.get("confirmation", {})
    confirmed = (
        recognition_status.get("state") == "target_confirmed"
        and isinstance(confirmation, dict)
        and confirmation.get("confirmed") is True
    )
    heights = []
    for height in box_heights_pixels:
        if isinstance(height, bool) or not isinstance(height, (int, float)):
            continue
        value = float(height)
        if math.isfinite(value) and value > 0:
            heights.append(value)
    # A latched confirmation without a current target box cannot guide motion.
    confirmed = confirmed and bool(heights)
    fraction = 0.0 if not heights else min(1.0, max(heights) / image_height_pixels)
    return TargetObservation(
        confirmed=confirmed,
        age_seconds=float(match_frame_age_seconds),
        box_height_fraction=fraction,
    )
