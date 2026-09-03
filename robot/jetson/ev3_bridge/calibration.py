"""Pure helpers for repeatable encoder-odometry calibration runs."""

import json
import math
import os
import tempfile

try:
    from .odometry import estimate_track_width
    from .odometry import estimate_wheel_radius
except (ImportError, ValueError):
    from odometry import estimate_track_width
    from odometry import estimate_wheel_radius


CALIBRATION_REPORT_VERSION = 1


class CalibrationError(ValueError):
    """Raised when a calibration capture or measurement cannot be trusted."""


def motor_positions(status):
    """Return left/right encoder positions from an EV3 status document."""

    try:
        left = status["motors"]["left"]["position"]
        right = status["motors"]["right"]["position"]
    except (KeyError, TypeError):
        raise CalibrationError("status does not contain both drive encoders")
    if isinstance(left, bool) or isinstance(right, bool):
        raise CalibrationError("encoder positions must be numbers")
    try:
        return int(left), int(right)
    except (TypeError, ValueError):
        raise CalibrationError("encoder positions must be integers")


def encoder_deltas(start_status, end_status):
    start_left, start_right = motor_positions(start_status)
    end_left, end_right = motor_positions(end_status)
    return end_left - start_left, end_right - start_right


def validate_capture(motion, speed, duration_seconds):
    if motion not in ("straight", "turn"):
        raise CalibrationError("motion must be straight or turn")
    if isinstance(speed, bool) or not isinstance(speed, (int, float)):
        raise CalibrationError("speed must be a number")
    if not math.isfinite(speed) or speed == 0:
        raise CalibrationError("speed must be finite and non-zero")
    if not math.isfinite(duration_seconds) or not 0 < duration_seconds <= 5.0:
        raise CalibrationError("duration must be greater than zero and at most 5 seconds")
    if motion == "straight" and abs(speed) > 0.15:
        raise CalibrationError("straight calibration speed is limited to 0.15 m/s")
    if motion == "turn" and abs(speed) > 1.0:
        raise CalibrationError("turn calibration speed is limited to 1.0 rad/s")


def validate_encoder_motion(motion, left_delta, right_delta, minimum_counts=10):
    """Reject captures with a stalled track or the wrong differential pattern."""

    if minimum_counts <= 0:
        raise CalibrationError("minimum encoder counts must be positive")
    if abs(left_delta) < minimum_counts or abs(right_delta) < minimum_counts:
        raise CalibrationError("both drive encoders must exceed the minimum threshold")
    same_direction = (left_delta > 0) == (right_delta > 0)
    if motion == "straight" and not same_direction:
        raise CalibrationError("straight capture encoders moved in opposite directions")
    if motion == "turn" and same_direction:
        raise CalibrationError("turn capture encoders moved in the same direction")
    return True


def calibration_result(
    report,
    measured_distance_m=None,
    measured_yaw_degrees=None,
    wheel_radius_m=None,
):
    """Calculate one effective geometry value from a successful capture."""

    if report.get("outcome") != "success":
        raise CalibrationError("only a successful capture can be calibrated")
    motion = report.get("motion")
    try:
        left_delta = int(report["encoder_delta"]["left"])
        right_delta = int(report["encoder_delta"]["right"])
        counts_per_revolution = int(report["encoder_counts_per_rev"])
    except (KeyError, TypeError, ValueError):
        raise CalibrationError("capture report is missing valid encoder data")

    if motion == "straight":
        if measured_distance_m is None or measured_yaw_degrees is not None:
            raise CalibrationError("straight calibration requires only measured distance")
        radius = estimate_wheel_radius(
            float(measured_distance_m),
            left_delta,
            right_delta,
            counts_per_revolution,
        )
        return {"wheel_radius_m": radius}

    if motion == "turn":
        if measured_yaw_degrees is None or measured_distance_m is not None:
            raise CalibrationError("turn calibration requires only measured yaw")
        if wheel_radius_m is None:
            try:
                wheel_radius_m = float(report["wheel_radius_m"])
            except (KeyError, TypeError, ValueError):
                raise CalibrationError("turn capture is missing wheel_radius_m")
        else:
            wheel_radius_m = float(wheel_radius_m)
        width = estimate_track_width(
            math.radians(float(measured_yaw_degrees)),
            left_delta,
            right_delta,
            wheel_radius_m,
            counts_per_revolution,
        )
        return {"track_width_m": width}

    raise CalibrationError("capture report has an unknown motion type")


def write_report(path, report):
    """Atomically write a JSON report for both successful and failed runs."""

    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory):
        os.makedirs(directory)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".odometry-calibration-", suffix=".json", dir=directory
    )
    try:
        with os.fdopen(descriptor, "w") as output:
            json.dump(report, output, indent=2, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise


def load_report(path):
    with open(path, "r") as source:
        report = json.load(source)
    if not isinstance(report, dict):
        raise CalibrationError("capture report must contain a JSON object")
    return report
