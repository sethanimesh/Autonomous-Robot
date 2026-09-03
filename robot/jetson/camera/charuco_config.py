#!/usr/bin/env python3
"""Dependency-free configuration and quality rules for ChArUco calibration."""

import math


DICTIONARY_NAME = "DICT_5X5_100"
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH_M = 0.025
MARKER_LENGTH_M = 0.018
MIN_VIEWS = 20
TARGET_VIEWS = 30
MIN_CORNERS = 12
MIN_BOARD_COVERAGE = 0.035
MIN_VIEW_DISTANCE = 0.12
MAX_RMS_ERROR_PX = 1.0
MAX_VIEW_ERROR_PX = 1.5


class CharucoConfigError(ValueError):
    """Raised when a calibration setting or result is unsafe to use."""


def validate_board(squares_x, squares_y, square_length_m, marker_length_m):
    squares_x = int(squares_x)
    squares_y = int(squares_y)
    square_length_m = float(square_length_m)
    marker_length_m = float(marker_length_m)
    if squares_x < 3 or squares_y < 3:
        raise CharucoConfigError("a ChArUco board needs at least 3x3 squares")
    if square_length_m <= 0.0:
        raise CharucoConfigError("square length must be positive")
    if marker_length_m <= 0.0 or marker_length_m >= square_length_m:
        raise CharucoConfigError("marker length must be positive and smaller than a square")
    return squares_x, squares_y, square_length_m, marker_length_m


def view_descriptor(points, image_width, image_height):
    """Describe a detected board by center, size and orientation.

    The normalized descriptor lets the live collector reject near-duplicate
    frames without importing OpenCV or numpy in this module.
    """
    if int(image_width) <= 0 or int(image_height) <= 0:
        raise CharucoConfigError("image dimensions must be positive")
    flattened = []
    for point in points:
        while isinstance(point, (list, tuple)) and len(point) == 1:
            point = point[0]
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise CharucoConfigError("corner points must contain x and y")
        flattened.append((float(point[0]), float(point[1])))
    if len(flattened) < 2:
        raise CharucoConfigError("at least two corners are required")

    xs = [point[0] for point in flattened]
    ys = [point[1] for point in flattened]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    center_x = (max(xs) + min(xs)) * 0.5 / float(image_width)
    center_y = (max(ys) + min(ys)) * 0.5 / float(image_height)
    coverage = (width * height) / float(image_width * image_height)

    first = flattened[0]
    farthest = max(flattened[1:], key=lambda p: (p[0] - first[0]) ** 2 + (p[1] - first[1]) ** 2)
    angle = math.atan2(farthest[1] - first[1], farthest[0] - first[0]) / math.pi
    return (center_x, center_y, math.sqrt(max(0.0, coverage)), angle)


def descriptor_distance(left, right):
    if len(left) != 4 or len(right) != 4:
        raise CharucoConfigError("view descriptors must contain four values")
    angle_delta = abs(float(left[3]) - float(right[3]))
    angle_delta = min(angle_delta, 2.0 - angle_delta)
    deltas = [
        float(left[0]) - float(right[0]),
        float(left[1]) - float(right[1]),
        float(left[2]) - float(right[2]),
        angle_delta * 0.5,
    ]
    return math.sqrt(sum(value * value for value in deltas))


def is_novel_view(descriptor, previous, minimum_distance=MIN_VIEW_DISTANCE):
    return not previous or min(descriptor_distance(descriptor, item) for item in previous) >= float(minimum_distance)


def validate_result(
    image_width,
    image_height,
    rms_error_px,
    camera_matrix,
    distortion_coefficients,
    per_view_errors,
    view_count,
):
    """Return rejection reasons for a calibration result; empty means usable."""
    reasons = []
    width = float(image_width)
    height = float(image_height)
    if int(view_count) < MIN_VIEWS:
        reasons.append("only {0} views; need at least {1}".format(view_count, MIN_VIEWS))
    if not math.isfinite(float(rms_error_px)) or float(rms_error_px) > MAX_RMS_ERROR_PX:
        reasons.append("RMS reprojection error is {0:.3f}px; limit is {1:.3f}px".format(float(rms_error_px), MAX_RMS_ERROR_PX))

    try:
        fx = float(camera_matrix[0][0])
        fy = float(camera_matrix[1][1])
        cx = float(camera_matrix[0][2])
        cy = float(camera_matrix[1][2])
    except (IndexError, TypeError, ValueError):
        return reasons + ["camera matrix is malformed"]
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        reasons.append("camera matrix contains non-finite values")
    if not 0.25 * width <= fx <= 4.0 * width or not 0.25 * width <= fy <= 4.0 * width:
        reasons.append("focal lengths are implausible for this image size")
    if not 0.0 <= cx <= width or not 0.0 <= cy <= height:
        reasons.append("principal point lies outside the image")

    distortion = [float(value) for value in distortion_coefficients]
    if len(distortion) not in (4, 5, 8):
        reasons.append("unexpected distortion coefficient count: {0}".format(len(distortion)))
    elif not all(math.isfinite(value) for value in distortion):
        reasons.append("distortion coefficients contain non-finite values")

    errors = [float(value) for value in per_view_errors]
    if not errors:
        reasons.append("per-view reprojection errors are missing")
    elif max(errors) > MAX_VIEW_ERROR_PX:
        reasons.append("worst view error is {0:.3f}px; limit is {1:.3f}px".format(max(errors), MAX_VIEW_ERROR_PX))
    return reasons
