"""Validation helpers for the temporary camera-head browser controls."""


CAMERA_JOG_DEGREES = 5
CAMERA_STATUS_MAX_AGE_SECONDS = 2.0


def validate_camera_jog(status, status_age_seconds, degrees):
    if degrees not in (-CAMERA_JOG_DEGREES, CAMERA_JOG_DEGREES):
        raise ValueError("Camera tilt accepts only one 5-degree step.")
    if not isinstance(status, dict) or status_age_seconds is None:
        raise ValueError("Camera-head status is unavailable.")
    if status_age_seconds < 0 or status_age_seconds > CAMERA_STATUS_MAX_AGE_SECONDS:
        raise ValueError("Camera-head status is stale.")
    if not status.get("homed"):
        raise ValueError("Camera head must be homed before manual tilt.")
    if status.get("moving") or status.get("homing"):
        raise ValueError("Camera head is already moving.")
    try:
        position = int(status["position"])
        minimum = int(status["minimum_position"])
        maximum = int(status["maximum_position"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("Camera-head position limits are unavailable.")
    target = position + degrees
    if target < minimum or target > maximum:
        raise ValueError("That step would cross the camera-head limit.")
    return target
