#!/usr/bin/env python3
"""Camera calibration loading for the robot camera node.

Intrinsics are never invented. When no usable calibration file is present the
node publishes an explicitly uncalibrated CameraInfo: D, K, R and P are all
zero, which is the documented sensor_msgs/CameraInfo marker for an
uncalibrated camera (K[0] == 0.0), and the reason is reported on
/camera/status.
"""

import os

UNCALIBRATED_DISTORTION_MODEL = ""

NO_FILE_CONFIGURED = "no_calibration_file_configured"
FILE_NOT_FOUND = "calibration_file_not_found"
FILE_INVALID = "calibration_file_invalid"
RESOLUTION_MISMATCH = "calibration_resolution_mismatch"
LOADED = "calibration_file_loaded"


class CalibrationError(ValueError):
    """Raised when a calibration document cannot be trusted."""


class Calibration(object):
    """CameraInfo content, either loaded from a file or explicitly absent."""

    def __init__(
        self,
        image_width,
        image_height,
        camera_matrix,
        distortion_model,
        distortion_coefficients,
        rectification_matrix,
        projection_matrix,
        camera_name="",
        is_calibrated=True,
        reason=LOADED,
        source="",
    ):
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.camera_matrix = [float(value) for value in camera_matrix]
        self.distortion_model = str(distortion_model)
        self.distortion_coefficients = [float(value) for value in distortion_coefficients]
        self.rectification_matrix = [float(value) for value in rectification_matrix]
        self.projection_matrix = [float(value) for value in projection_matrix]
        self.camera_name = str(camera_name)
        self.is_calibrated = bool(is_calibrated)
        self.reason = reason
        self.source = source

    def matches_resolution(self, width, height):
        return self.image_width == int(width) and self.image_height == int(height)

    def describe(self):
        if self.is_calibrated:
            return "{0}:{1}".format(self.reason, self.source)
        return self.reason if not self.source else "{0}:{1}".format(self.reason, self.source)


def uncalibrated(image_width, image_height, reason=NO_FILE_CONFIGURED, source=""):
    """An explicitly uncalibrated CameraInfo payload with zeroed matrices."""
    return Calibration(
        image_width=image_width,
        image_height=image_height,
        camera_matrix=[0.0] * 9,
        distortion_model=UNCALIBRATED_DISTORTION_MODEL,
        distortion_coefficients=[],
        rectification_matrix=[0.0] * 9,
        projection_matrix=[0.0] * 12,
        camera_name="",
        is_calibrated=False,
        reason=reason,
        source=source,
    )


def _matrix(document, key, rows, cols):
    section = document.get(key)
    if not isinstance(section, dict):
        raise CalibrationError("{0} is missing or is not a mapping".format(key))
    if int(section.get("rows", -1)) != rows or int(section.get("cols", -1)) != cols:
        raise CalibrationError(
            "{0} must be declared {1}x{2}".format(key, rows, cols)
        )
    data = section.get("data")
    if not isinstance(data, (list, tuple)) or len(data) != rows * cols:
        raise CalibrationError(
            "{0}.data must hold {1} values".format(key, rows * cols)
        )
    try:
        return [float(value) for value in data]
    except (TypeError, ValueError):
        raise CalibrationError("{0}.data must be numeric".format(key))


def calibration_from_mapping(document, source=""):
    """Build a Calibration from a parsed ROS calibration document."""
    if not isinstance(document, dict):
        raise CalibrationError("calibration document must be a mapping")

    try:
        width = int(document["image_width"])
        height = int(document["image_height"])
    except (KeyError, TypeError, ValueError):
        raise CalibrationError("image_width and image_height must be integers")
    if width <= 0 or height <= 0:
        raise CalibrationError("image_width and image_height must be positive")

    camera_matrix = _matrix(document, "camera_matrix", 3, 3)
    if camera_matrix[0] == 0.0:
        raise CalibrationError("camera_matrix data is zeroed, which means uncalibrated")

    distortion_model = document.get("distortion_model", "")
    if not isinstance(distortion_model, str) or not distortion_model.strip():
        raise CalibrationError("distortion_model must be a non-empty string")

    distortion = document.get("distortion_coefficients")
    if not isinstance(distortion, dict):
        raise CalibrationError("distortion_coefficients is missing or is not a mapping")
    cols = int(distortion.get("cols", -1))
    if int(distortion.get("rows", -1)) != 1 or cols < 1:
        raise CalibrationError("distortion_coefficients must be declared 1xN")
    coefficients = _matrix(document, "distortion_coefficients", 1, cols)

    return Calibration(
        image_width=width,
        image_height=height,
        camera_matrix=camera_matrix,
        distortion_model=distortion_model.strip(),
        distortion_coefficients=coefficients,
        rectification_matrix=_matrix(document, "rectification_matrix", 3, 3),
        projection_matrix=_matrix(document, "projection_matrix", 3, 4),
        camera_name=document.get("camera_name", ""),
        is_calibrated=True,
        reason=LOADED,
        source=source,
    )


def load_calibration_file(path):
    """Parse a ROS camera calibration YAML file."""
    import yaml

    with open(path, "r") as handle:
        document = yaml.safe_load(handle)
    return calibration_from_mapping(document, source=path)


def resolve_calibration(path, image_width, image_height, loader=None, exists=None):
    """Return the CameraInfo payload to publish and never fabricate intrinsics."""
    if loader is None:
        loader = load_calibration_file
    if exists is None:
        exists = os.path.exists

    if not path:
        return uncalibrated(image_width, image_height, NO_FILE_CONFIGURED)
    if not exists(path):
        return uncalibrated(image_width, image_height, FILE_NOT_FOUND, source=path)

    try:
        calibration = loader(path)
    except Exception as exc:
        return uncalibrated(
            image_width,
            image_height,
            "{0}:{1}".format(FILE_INVALID, exc),
            source=path,
        )

    if not calibration.matches_resolution(image_width, image_height):
        return uncalibrated(
            image_width,
            image_height,
            "{0}:file_is_{1}x{2}".format(
                RESOLUTION_MISMATCH, calibration.image_width, calibration.image_height
            ),
            source=path,
        )
    calibration.source = path
    return calibration
