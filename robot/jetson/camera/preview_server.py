#!/usr/bin/env python3
"""LAN MJPEG preview for positioning the ChArUco board."""

import argparse
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

import cv2
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from calibrate_charuco import _board
from calibrate_charuco import _dictionary
from calibrate_charuco import detect_charuco
from calibrate_charuco import ros_image_to_bgr


PAGE = b"""<!doctype html>
<html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>Echora camera calibration</title>
<style>body{margin:0;background:#111;color:#eee;font:18px sans-serif;text-align:center}
h2{margin:12px}.note{margin:8px}img{width:min(100vw,960px);height:auto}</style></head>
<body><h2>Echora camera calibration</h2>
<div class=note id=status>Connecting...</div>
<div class=note>Move only the board. Green corners mean it is detectable.</div>
<img src=/stream.mjpg>
<script>setInterval(async()=>{try{let r=await fetch('/health',{cache:'no-store'}),j=await r.json();
document.getElementById('status').textContent=`Accepted ${j.accepted_views}/30 | corners ${j.charuco_corners} | board coverage ${j.board_coverage_percent}% | ${j.calibration_state}`}
catch(e){document.getElementById('status').textContent='Preview disconnected'}},500)</script>
</body></html>"""


def calibration_progress(path="/tmp/echora-calibrator.log"):
    accepted = 0
    state = "collecting"
    try:
        with open(path, "r") as handle:
            contents = handle.read()
    except OSError:
        return accepted, "collector not running"
    matches = re.findall(r"accepted view (\d+)/(\d+)", contents)
    if matches:
        accepted = int(matches[-1][0])
    if "CALIBRATION ACCEPTED" in contents:
        state = "calibration accepted"
    elif "CALIBRATION REJECTED" in contents or "CALIBRATION FAILED" in contents:
        state = "calibration failed"
    return accepted, state


class PreviewState(object):
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.frames = 0
        self.corners = 0
        self.coverage_percent = 0.0
        self.last_frame_monotonic = None

    def update(self, jpeg, corners, coverage_percent):
        with self.condition:
            self.jpeg = jpeg
            self.corners = int(corners)
            self.coverage_percent = float(coverage_percent)
            self.frames += 1
            self.sequence += 1
            self.last_frame_monotonic = time.monotonic()
            self.condition.notify_all()

    def health(self):
        with self.condition:
            age = None
            if self.last_frame_monotonic is not None:
                age = time.monotonic() - self.last_frame_monotonic
            accepted, calibration_state = calibration_progress()
            return {
                "frames": self.frames,
                "charuco_corners": self.corners,
                "board_coverage_percent": round(self.coverage_percent, 1),
                "frame_age_sec": age,
                "accepted_views": accepted,
                "calibration_state": calibration_state,
            }


class PreviewNode(Node):
    def __init__(self, state, topic, max_rate):
        super().__init__("echora_calibration_preview")
        self.state = state
        self.board = _board()
        self.dictionary = _dictionary()
        self.minimum_period = 1.0 / max_rate
        self.last_processed = 0.0
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_processed < self.minimum_period:
            return
        self.last_processed = now
        try:
            frame = ros_image_to_bgr(message).copy()
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        corners, ids = detect_charuco(frame, self.board, self.dictionary)
        count = 0 if ids is None else len(ids)
        coverage_percent = 0.0
        if corners is not None:
            points = corners.reshape((-1, 2))
            coverage_percent = (
                (float(points[:, 0].max()) - float(points[:, 0].min()))
                * (float(points[:, 1].max()) - float(points[:, 1].min()))
                * 100.0 / float(message.width * message.height)
            )
            cv2.aruco.drawDetectedCornersCharuco(frame, corners, ids, (0, 255, 0))
            label = "BOARD: {0} CORNERS  COVERAGE: {1:.1f}%".format(count, coverage_percent)
            color = (0, 210, 0)
        else:
            label = "BOARD NOT DETECTED"
            color = (0, 0, 255)
        cv2.rectangle(frame, (0, 0), (message.width, 42), (0, 0, 0), -1)
        cv2.putText(frame, label, (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            self.state.update(encoded.tobytes(), count, coverage_percent)


class Handler(BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path == "/health":
            payload = json.dumps(self.state.health()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        sequence = -1
        try:
            while True:
                with self.state.condition:
                    self.state.condition.wait_for(lambda: self.state.sequence != sequence, timeout=2.0)
                    if self.state.jpeg is None:
                        continue
                    sequence = self.state.sequence
                    jpeg = self.state.jpeg
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format_string, *args):
        return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--max-rate", type=float, default=8.0)
    options, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    state = PreviewState()
    Handler.state = state
    server = ThreadingHTTPServer((options.bind, options.port), Handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    node = PreviewNode(state, options.topic, options.max_rate)
    print("preview: http://192.168.1.48:{0}/".format(options.port), flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
