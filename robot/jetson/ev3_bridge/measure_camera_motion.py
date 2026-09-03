#!/usr/bin/env python3
"""Jog the camera head once and measure actual motion in the camera image."""

import argparse
import json
import subprocess
import time
import urllib.request

import cv2
import numpy as np


MAX_STREAM_BYTES = 4 * 1024 * 1024


def capture_jpeg(stream_url, timeout_seconds=5.0):
    """Read one complete JPEG from an MJPEG stream without saving room images."""

    data = bytearray()
    with urllib.request.urlopen(stream_url, timeout=timeout_seconds) as response:
        while len(data) < MAX_STREAM_BYTES:
            chunk = response.read(16384)
            if not chunk:
                break
            data.extend(chunk)
            start = data.find(b"\xff\xd8")
            end = data.find(b"\xff\xd9", start + 2 if start >= 0 else 0)
            if start >= 0 and end >= 0:
                encoded = np.frombuffer(bytes(data[start : end + 2]), dtype=np.uint8)
                image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError("preview returned an invalid JPEG")
                return image
    raise RuntimeError("no complete JPEG received from preview")


def measure_motion(before, after):
    if before.shape != after.shape:
        raise ValueError("before and after frames have different shapes")
    before_gray = cv2.cvtColor(before, cv2.COLOR_BGR2GRAY)
    after_gray = cv2.cvtColor(after, cv2.COLOR_BGR2GRAY)
    phase_shift, phase_response = cv2.phaseCorrelate(
        before_gray.astype(np.float32), after_gray.astype(np.float32)
    )
    result = {
        "mean_absolute_pixel_change": round(
            float(
                np.mean(
                    np.abs(before.astype(np.int16) - after.astype(np.int16))
                )
            ),
            3,
        ),
        "phase_horizontal_shift_px": round(float(phase_shift[0]), 3),
        "phase_vertical_shift_px": round(float(phase_shift[1]), 3),
        "phase_response": round(float(phase_response), 4),
    }
    points = cv2.goodFeaturesToTrack(
        before_gray, maxCorners=300, qualityLevel=0.01, minDistance=8
    )
    if points is None or len(points) < 10:
        result["tracked_features"] = 0
        result["optical_flow_available"] = False
        return result
    moved, status, _error = cv2.calcOpticalFlowPyrLK(
        before_gray,
        after_gray,
        points,
        None,
        winSize=(31, 31),
        maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    if moved is None or status is None:
        result["tracked_features"] = 0
        result["optical_flow_available"] = False
        return result
    valid = status.reshape(-1) == 1
    shifts = moved.reshape(-1, 2)[valid] - points.reshape(-1, 2)[valid]
    if len(shifts) < 10:
        result["tracked_features"] = int(len(shifts))
        result["optical_flow_available"] = False
        return result
    magnitudes = np.linalg.norm(shifts, axis=1)
    result.update({
        "tracked_features": int(len(shifts)),
        "optical_flow_available": True,
        "median_horizontal_shift_px": round(float(np.median(shifts[:, 0])), 3),
        "median_vertical_shift_px": round(float(np.median(shifts[:, 1])), 3),
        "median_motion_px": round(float(np.median(magnitudes)), 3),
    })
    return result


def publish_jog(degrees):
    payload = json.dumps({"action": "jog", "degrees": degrees}, separators=(",", ":"))
    message = "{data: '" + payload + "'}"
    subprocess.run(
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/camera_head/command",
            "std_msgs/msg/String",
            message,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        timeout=20,
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--degrees", type=int, required=True)
    parser.add_argument("--stream-url", default="http://127.0.0.1:8080/stream")
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.degrees == 0 or abs(args.degrees) > 15:
        parser.error("degrees must be non-zero and no more than 15")
    if args.settle_seconds < 0.5 or args.settle_seconds > 5.0:
        parser.error("settle-seconds must be between 0.5 and 5")
    return args


def main(argv=None):
    args = parse_args(argv)
    before = capture_jpeg(args.stream_url)
    publish_jog(args.degrees)
    time.sleep(args.settle_seconds)
    after = capture_jpeg(args.stream_url)
    result = measure_motion(before, after)
    result["commanded_encoder_delta"] = args.degrees
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
