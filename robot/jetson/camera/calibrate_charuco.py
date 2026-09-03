#!/usr/bin/env python3
"""Collect diverse ChArUco views from ROS and write a validated CameraInfo YAML."""

import argparse
import json
import math
import os
import tempfile
import time

import cv2
import numpy
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from charuco_config import DICTIONARY_NAME
from charuco_config import MARKER_LENGTH_M
from charuco_config import MAX_VIEW_ERROR_PX
from charuco_config import MIN_BOARD_COVERAGE
from charuco_config import MIN_CORNERS
from charuco_config import MIN_VIEWS
from charuco_config import SQUARE_LENGTH_M
from charuco_config import SQUARES_X
from charuco_config import SQUARES_Y
from charuco_config import TARGET_VIEWS
from charuco_config import is_novel_view
from charuco_config import validate_board
from charuco_config import validate_result
from charuco_config import view_descriptor


def _dictionary():
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY_NAME))


def _board():
    return cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_LENGTH_M, MARKER_LENGTH_M, _dictionary()
    )


def ros_image_to_bgr(message):
    if message.encoding != "bgr8" or message.is_bigendian or message.step != message.width * 3:
        raise ValueError("expected tightly packed little-endian bgr8 image")
    expected = int(message.height) * int(message.step)
    data = numpy.frombuffer(bytes(message.data), dtype=numpy.uint8)
    if data.size != expected:
        raise ValueError("image payload size does not match dimensions")
    return data.reshape((message.height, message.width, 3))


def detect_charuco(frame, board, dictionary):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 4:
        return None, None
    count, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    if corners is None or ids is None or int(count) < MIN_CORNERS:
        return None, None
    return corners, ids


def calibrate(samples, image_size, board):
    corners = [sample[0] for sample in samples]
    ids = [sample[1] for sample in samples]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9)
    result = cv2.aruco.calibrateCameraCharucoExtended(
        corners,
        ids,
        board,
        image_size,
        None,
        None,
        flags=0,
        criteria=criteria,
    )
    return {
        "rms": float(result[0]),
        "camera_matrix": result[1],
        "distortion": result[2].reshape(-1),
        "rvecs": result[3],
        "tvecs": result[4],
        "intrinsic_stddev": result[5].reshape(-1),
        "per_view_errors": result[7].reshape(-1),
    }


def prune_outliers(samples, image_size, board):
    kept = list(samples)
    removed = []
    maximum_removals = min(len(samples) - MIN_VIEWS, max(1, len(samples) // 5))
    while len(removed) < maximum_removals:
        result = calibrate(kept, image_size, board)
        errors = result["per_view_errors"]
        worst_index = int(numpy.argmax(errors))
        median = float(numpy.median(errors))
        threshold = max(MAX_VIEW_ERROR_PX, median * 2.5)
        if float(errors[worst_index]) <= threshold:
            return kept, removed, result
        removed.append({"sample": kept[worst_index][2], "error_px": float(errors[worst_index])})
        del kept[worst_index]
    return kept, removed, calibrate(kept, image_size, board)


def calibration_document(result, width, height, camera_name):
    matrix = result["camera_matrix"].reshape(-1).tolist()
    distortion = result["distortion"].reshape(-1).tolist()
    projection = [matrix[0], matrix[1], matrix[2], 0.0,
                  matrix[3], matrix[4], matrix[5], 0.0,
                  matrix[6], matrix[7], matrix[8], 0.0]
    return {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": len(distortion), "data": distortion},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        "projection_matrix": {"rows": 3, "cols": 4, "data": projection},
    }


def atomic_yaml(path, document):
    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory):
        os.makedirs(directory)
    descriptor, temporary = tempfile.mkstemp(prefix=".calibration-", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(descriptor, "w") as handle:
            yaml.safe_dump(document, handle, default_flow_style=False, sort_keys=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except Exception:
        if os.path.exists(temporary):
            os.unlink(temporary)
        raise


class Collector(Node):
    def __init__(self, args):
        super().__init__("echora_camera_calibrator")
        self.args = args
        self.board = _board()
        self.dictionary = _dictionary()
        self.samples = []
        self.descriptors = []
        self.image_size = None
        self.last_capture = 0.0
        self.frames = 0
        self.detected_frames = 0
        self.done = False
        self.last_notice = 0.0
        self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)
        print("Move the board around the whole image and tilt it in different directions.", flush=True)
        print("Collecting {0} diverse views; motors are not touched.".format(args.target_views), flush=True)

    def on_image(self, message):
        if self.done:
            return
        self.frames += 1
        try:
            frame = ros_image_to_bgr(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        size = (int(message.width), int(message.height))
        if self.image_size is None:
            self.image_size = size
        elif size != self.image_size:
            self.get_logger().error("image resolution changed during calibration")
            self.done = True
            return

        corners, ids = detect_charuco(frame, self.board, self.dictionary)
        now = time.monotonic()
        if corners is None:
            if now - self.last_notice >= 5.0:
                print("board not detected yet ({0}/{1} views)".format(len(self.samples), self.args.target_views), flush=True)
                self.last_notice = now
            return
        self.detected_frames += 1
        points = corners.reshape((-1, 2)).tolist()
        descriptor = view_descriptor(points, message.width, message.height)
        coverage = descriptor[2] ** 2
        if coverage < self.args.min_coverage:
            if now - self.last_notice >= 3.0:
                print("board is too small; move it closer", flush=True)
                self.last_notice = now
            return
        if now - self.last_capture < self.args.min_interval:
            return
        if not is_novel_view(descriptor, self.descriptors, self.args.min_view_distance):
            if now - self.last_notice >= 3.0:
                print("duplicate view; move/tilt the board ({0}/{1})".format(len(self.samples), self.args.target_views), flush=True)
                self.last_notice = now
            return

        sample_number = len(self.samples) + 1
        self.samples.append((corners.copy(), ids.copy(), sample_number))
        self.descriptors.append(descriptor)
        self.last_capture = now
        print("accepted view {0}/{1}: corners={2}, coverage={3:.1f}%".format(
            sample_number, self.args.target_views, len(ids), coverage * 100.0), flush=True)
        if len(self.samples) >= self.args.target_views:
            self.done = True


def write_report(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--output", default="/home/animesh/echora/camera_calibration.yaml")
    parser.add_argument("--report", default="/home/animesh/echora/camera_calibration_report.json")
    parser.add_argument("--camera-name", default="echora_usb_camera")
    parser.add_argument("--target-views", type=int, default=TARGET_VIEWS)
    parser.add_argument("--timeout-sec", type=float, default=300.0)
    parser.add_argument("--min-interval", type=float, default=0.6)
    parser.add_argument("--min-coverage", type=float, default=MIN_BOARD_COVERAGE)
    parser.add_argument("--min-view-distance", type=float, default=0.08)
    options = parser.parse_args(args)
    validate_board(SQUARES_X, SQUARES_Y, SQUARE_LENGTH_M, MARKER_LENGTH_M)
    if options.target_views < MIN_VIEWS:
        parser.error("--target-views must be at least {0}".format(MIN_VIEWS))

    rclpy.init()
    collector = Collector(options)
    deadline = time.monotonic() + options.timeout_sec
    try:
        while rclpy.ok() and not collector.done and time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    base_report = {
        "board": {
            "dictionary": DICTIONARY_NAME,
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_length_m": SQUARE_LENGTH_M,
            "marker_length_m": MARKER_LENGTH_M,
        },
        "frames_seen": collector.frames,
        "frames_with_board": collector.detected_frames,
        "views_collected": len(collector.samples),
        "image_size": list(collector.image_size) if collector.image_size else None,
        "accepted": False,
    }
    if len(collector.samples) < MIN_VIEWS:
        base_report["rejection_reasons"] = ["only {0} diverse views collected; need {1}".format(len(collector.samples), MIN_VIEWS)]
        write_report(options.report, base_report)
        print("CALIBRATION FAILED: {0}".format(base_report["rejection_reasons"][0]), flush=True)
        return 2

    kept, removed, result = prune_outliers(collector.samples, collector.image_size, collector.board)
    errors = result["per_view_errors"].tolist()
    reasons = validate_result(
        collector.image_size[0], collector.image_size[1], result["rms"],
        result["camera_matrix"].tolist(), result["distortion"].tolist(), errors, len(kept)
    )
    base_report.update({
        "accepted": not reasons,
        "views_used": len(kept),
        "removed_outliers": removed,
        "rms_error_px": result["rms"],
        "mean_view_error_px": float(numpy.mean(result["per_view_errors"])),
        "max_view_error_px": float(numpy.max(result["per_view_errors"])),
        "per_view_errors_px": errors,
        "camera_matrix": result["camera_matrix"].tolist(),
        "distortion_coefficients": result["distortion"].tolist(),
        "intrinsic_stddev": result["intrinsic_stddev"].tolist(),
        "rejection_reasons": reasons,
    })
    write_report(options.report, base_report)
    if reasons:
        print("CALIBRATION REJECTED: {0}".format("; ".join(reasons)), flush=True)
        return 3

    document = calibration_document(result, collector.image_size[0], collector.image_size[1], options.camera_name)
    atomic_yaml(options.output, document)
    print("CALIBRATION ACCEPTED: RMS={0:.3f}px, worst={1:.3f}px, views={2}".format(
        result["rms"], max(errors), len(kept)), flush=True)
    print("wrote {0}".format(options.output), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
