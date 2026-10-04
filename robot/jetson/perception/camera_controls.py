"""Validation helpers for the camera-head browser controls."""

# With the current linkage, Motor A's encoder counts decrease while the camera
# is lifted and increase while it is lowered. Re-confirmed from the live UI on
# 2026-09-05 after the camera support/linkage change. Anything that turns a
# direction into encoder degrees must go through
# CAMERA_UP_SIGN / CAMERA_DOWN_SIGN rather than hard-coding a sign.
CAMERA_UP_SIGN = -1
CAMERA_DOWN_SIGN = 1

CAMERA_FINE_JOG_DEGREES = 5
CAMERA_COARSE_JOG_DEGREES = 15
CAMERA_JOG_DEGREES = CAMERA_FINE_JOG_DEGREES
CAMERA_ALLOWED_JOG_DEGREES = (
    -CAMERA_COARSE_JOG_DEGREES,
    -CAMERA_FINE_JOG_DEGREES,
    CAMERA_FINE_JOG_DEGREES,
    CAMERA_COARSE_JOG_DEGREES,
)
CAMERA_STATUS_MAX_AGE_SECONDS = 2.0


def validate_camera_jog(status, status_age_seconds, degrees):
    """Resolve a manual button independently of calibration and angle limits."""
    if isinstance(degrees, bool) or degrees not in CAMERA_ALLOWED_JOG_DEGREES:
        raise ValueError("Choose a 5-degree or 15-degree camera button.")
    if not isinstance(status, dict) or status_age_seconds is None:
        raise ValueError("Camera-head status is unavailable.")
    if status_age_seconds < 0 or status_age_seconds > CAMERA_STATUS_MAX_AGE_SECONDS:
        raise ValueError("Camera-head status is stale.")
    try:
        position = int(status["position"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("Camera-head position is unavailable.")
    if status.get("moving"):
        target = status.get("target_position")
        if target is not None:
            target = int(target)
            remaining = target - position
            if 0 < remaining * degrees and abs(remaining) <= CAMERA_COARSE_JOG_DEGREES:
                return target
    # Opposite direction is a takeover too; bridge stops the current move first.
    return position + degrees


def jog_degrees(direction, magnitude=CAMERA_FINE_JOG_DEGREES):
    """Encoder degrees for one step in a named direction.

    Callers say "up" or "down" and never spell out a sign, so the convention
    recorded at the top of this module stays the only place it is written down.
    """

    signs = {"up": CAMERA_UP_SIGN, "down": CAMERA_DOWN_SIGN}
    if direction not in signs:
        raise ValueError("Camera tilt direction must be 'up' or 'down'.")
    if magnitude not in (CAMERA_FINE_JOG_DEGREES, CAMERA_COARSE_JOG_DEGREES):
        raise ValueError(
            "Camera tilt magnitude must be {0} or {1} degrees.".format(
                CAMERA_FINE_JOG_DEGREES, CAMERA_COARSE_JOG_DEGREES
            )
        )
    return signs[direction] * magnitude
