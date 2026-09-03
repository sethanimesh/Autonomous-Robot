#!/usr/bin/env python3
"""Synthetic end-to-end test for the Jetson ChArUco calibration stack."""

import cv2
import numpy

from calibrate_charuco import _board
from calibrate_charuco import _dictionary
from calibrate_charuco import calibrate
from calibrate_charuco import detect_charuco
from charuco_config import MARKER_LENGTH_M
from charuco_config import SQUARE_LENGTH_M
from charuco_config import SQUARES_X
from charuco_config import SQUARES_Y
from charuco_config import validate_result


def synthetic_frame(board_image, camera_matrix, rotation, translation):
    width, height = 640, 480
    board_width = SQUARES_X * SQUARE_LENGTH_M
    board_height = SQUARES_Y * SQUARE_LENGTH_M
    object_corners = numpy.array(
        [[0.0, 0.0, 0.0], [board_width, 0.0, 0.0],
         [board_width, board_height, 0.0], [0.0, board_height, 0.0]],
        dtype=numpy.float32,
    )
    projected, _ = cv2.projectPoints(
        object_corners,
        numpy.asarray(rotation, dtype=numpy.float64),
        numpy.asarray(translation, dtype=numpy.float64),
        camera_matrix,
        numpy.zeros(5),
    )
    source = numpy.array(
        [[0.0, 0.0], [board_image.shape[1] - 1.0, 0.0],
         [board_image.shape[1] - 1.0, board_image.shape[0] - 1.0],
         [0.0, board_image.shape[0] - 1.0]],
        dtype=numpy.float32,
    )
    transform = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(numpy.float32))
    frame = cv2.warpPerspective(
        board_image, transform, (width, height), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=255,
    )
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def main():
    board = _board()
    dictionary = _dictionary()
    board_image = board.draw((1500, 2100), marginSize=0, borderBits=1)
    expected = numpy.array(
        [[520.0, 0.0, 320.0], [0.0, 515.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=numpy.float64,
    )
    samples = []
    poses = []
    for index in range(36):
        row = index // 6
        column = index % 6
        rotation = (
            -0.34 + 0.135 * row,
            -0.30 + 0.12 * column,
            -0.16 + 0.055 * ((index * 5) % 7),
        )
        translation = (
            -SQUARES_X * SQUARE_LENGTH_M * 0.5 + (-0.025 + 0.01 * column),
            -SQUARES_Y * SQUARE_LENGTH_M * 0.5 + (-0.018 + 0.007 * row),
            0.38 + 0.025 * ((index * 7) % 6),
        )
        poses.append((rotation, translation))

    for index, (rotation, translation) in enumerate(poses):
        frame = synthetic_frame(board_image, expected, rotation, translation)
        corners, ids = detect_charuco(frame, board, dictionary)
        if ids is not None:
            samples.append((corners, ids, index + 1))
    if len(samples) < 20:
        raise RuntimeError("synthetic detector found only {0}/36 views".format(len(samples)))

    result = calibrate(samples, (640, 480), board)
    reasons = validate_result(
        640, 480, result["rms"], result["camera_matrix"].tolist(),
        result["distortion"].tolist(), result["per_view_errors"].tolist(), len(samples),
    )
    fx_error = abs(float(result["camera_matrix"][0, 0]) - expected[0, 0]) / expected[0, 0]
    fy_error = abs(float(result["camera_matrix"][1, 1]) - expected[1, 1]) / expected[1, 1]
    if fx_error > 0.05 or fy_error > 0.05:
        reasons.append("recovered focal length differs by more than 5%")
    if reasons:
        raise RuntimeError("synthetic calibration rejected: {0}".format("; ".join(reasons)))
    print(
        "PASS: {0} synthetic views, RMS={1:.3f}px, fx error={2:.2f}%, fy error={3:.2f}%".format(
            len(samples), result["rms"], fx_error * 100.0, fy_error * 100.0
        )
    )


if __name__ == "__main__":
    main()
