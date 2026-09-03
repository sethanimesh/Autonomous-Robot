#!/usr/bin/env python3
"""Verify live CameraInfo pairing and render a temporary raw/rectified check."""

import argparse
import json
import math
import time

import cv2
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from calibrate_charuco import ros_image_to_bgr
from charuco_config import MIN_VALID_RECTIFIED_FRACTION


def stamp_key(message):
    return (int(message.header.stamp.sec), int(message.header.stamp.nanosec))


class Verifier(Node):
    def __init__(self, target, comparison_path):
        super().__init__("echora_camera_calibration_verifier")
        self.target = int(target)
        self.comparison_path = comparison_path
        self.images = {}
        self.infos = {}
        self.pairs = 0
        self.failures = []
        self.saved = False
        self.done = False
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data)

    def on_image(self, message):
        self.images[stamp_key(message)] = message
        self.match(stamp_key(message))

    def on_info(self, message):
        self.infos[stamp_key(message)] = message
        self.match(stamp_key(message))

    def match(self, key):
        image = self.images.get(key)
        info = self.infos.get(key)
        if image is None or info is None:
            self.trim()
            return
        del self.images[key]
        del self.infos[key]
        self.check_pair(image, info)
        self.pairs += 1
        if self.pairs >= self.target:
            self.done = True

    def trim(self):
        while len(self.images) > 20:
            del self.images[next(iter(self.images))]
        while len(self.infos) > 20:
            del self.infos[next(iter(self.infos))]

    def check_pair(self, image, info):
        if image.header.frame_id != info.header.frame_id:
            self.failures.append("frame_id mismatch")
        if image.width != info.width or image.height != info.height:
            self.failures.append("resolution mismatch")
        if info.distortion_model != "plumb_bob":
            self.failures.append("distortion model is not plumb_bob")
        values = list(info.k) + list(info.d) + list(info.r) + list(info.p)
        if info.k[0] == 0.0 or not all(math.isfinite(float(value)) for value in values):
            self.failures.append("calibration contains zero or non-finite values")
        if len(info.d) != 5:
            self.failures.append("expected five distortion coefficients")
        if not self.saved and self.comparison_path:
            frame = ros_image_to_bgr(image)
            matrix = numpy.asarray(info.k, dtype=numpy.float64).reshape((3, 3))
            distortion = numpy.asarray(info.d, dtype=numpy.float64)
            optimal, roi = cv2.getOptimalNewCameraMatrix(
                matrix, distortion, (image.width, image.height), 1.0,
                (image.width, image.height)
            )
            rectified = cv2.undistort(frame, matrix, distortion, None, optimal)
            comparison = numpy.concatenate((frame, rectified), axis=1)
            cv2.putText(comparison, "RAW", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(comparison, "RECTIFIED", (image.width + 12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if not cv2.imwrite(self.comparison_path, comparison):
                self.failures.append("failed to write comparison image")
            else:
                self.saved = True
                self.roi = [int(value) for value in roi]
                self.valid_fraction = float(roi[2] * roi[3]) / float(image.width * image.height)
                if self.valid_fraction < MIN_VALID_RECTIFIED_FRACTION:
                    self.failures.append(
                        "rectification keeps only {0:.1f}% valid pixels".format(
                            self.valid_fraction * 100.0
                        )
                    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairs", type=int, default=30)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--comparison", default="/tmp/echora-calibration-comparison.jpg")
    options = parser.parse_args()
    rclpy.init()
    node = Verifier(options.pairs, options.comparison)
    deadline = time.monotonic() + options.timeout_sec
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        result = {
            "accepted": node.done and not node.failures,
            "matched_exact_timestamps": node.pairs,
            "failures": sorted(set(node.failures)),
            "comparison": options.comparison if node.saved else None,
            "valid_roi": getattr(node, "roi", None),
            "valid_fraction": getattr(node, "valid_fraction", None),
        }
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(result, sort_keys=True))
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
