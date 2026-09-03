#!/usr/bin/env python3
"""Serve the annotated person-detection stream as MJPEG over HTTP.

Named to distinguish it from the camera calibration preview, which overlays
ChArUco corners rather than person boxes.

Lets an operator see what the detector sees while standing in front of the
robot, which is what makes physical perception testing practical: positioning
yourself blind does not work, and a dark or misaimed camera is invisible
without a picture.

    python3 detection_preview.py               # http://<jetson>:8088/
    python3 detection_preview.py --port 9000

Nothing is written to disk. The stream is built from whatever the detector
already publishes, so it adds no load to the camera.

SECURITY: this server is unauthenticated and binds to all interfaces by
default, so anyone on the network can watch the camera. It is a temporary
diagnostic tool, not a service. Run it while testing and stop it afterwards;
do not install it as a systemd unit without adding authentication and
restricting the bind address.
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

import cv2
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

BEST_EFFORT = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)
BANNER_HEIGHT = 26

PAGE = (
    b"<!doctype html><html><head><meta name='viewport' "
    b"content='width=device-width,initial-scale=1'><title>Echora preview</title>"
    b"</head><body style='margin:0;background:#111;text-align:center'>"
    b"<img src='/stream' style='max-width:100%;height:auto'></body></html>"
)

# Shared between the ROS thread and the HTTP threads. Only whole objects are
# swapped in, never mutated in place, so no lock is needed.
latest = {"jpeg": None, "people": 0, "score": 0.0}


class PreviewNode(Node):
    def __init__(self, image_topic, detections_topic, quality):
        super().__init__("echora_detection_preview")
        self.quality = int(quality)
        self.create_subscription(Image, image_topic, self.on_image, BEST_EFFORT)
        self.create_subscription(Detection2DArray, detections_topic, self.on_detections, 10)

    def on_detections(self, message):
        latest["people"] = len(message.detections)
        latest["score"] = max(
            [d.results[0].hypothesis.score for d in message.detections if d.results],
            default=0.0,
        )

    def on_image(self, message):
        if message.encoding != "bgr8":
            return
        expected = message.height * message.width * 3
        if len(message.data) != expected:
            return
        frame = numpy.frombuffer(message.data, numpy.uint8).reshape(
            message.height, message.width, 3
        )
        canvas = frame.copy()
        cv2.rectangle(canvas, (0, 0), (message.width, BANNER_HEIGHT), (0, 0, 0), -1)
        cv2.putText(
            canvas,
            "people: {0}   best: {1:.2f}".format(latest["people"], latest["score"]),
            (8, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        ok, buffer = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            latest["jpeg"] = buffer.tobytes()


class PreviewHandler(BaseHTTPRequestHandler):
    frame_interval = 0.07

    def log_message(self, *args):
        """Silence the default per-request logging."""

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.stream()
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(PAGE)))
        self.end_headers()
        self.wfile.write(PAGE)

    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while True:
                jpeg = latest["jpeg"]
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(
                        "Content-Length: {0}\r\n\r\n".format(len(jpeg)).encode("ascii")
                    )
                    self.wfile.write(jpeg + b"\r\n")
                time.sleep(self.frame_interval)
        except (BrokenPipeError, ConnectionResetError):
            # The viewer closed the tab. Normal, not an error.
            return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="interface to listen on; use 127.0.0.1 to keep it local",
    )
    parser.add_argument("--image-topic", default="/perception/person_image")
    parser.add_argument("--detections-topic", default="/perception/person_detections")
    parser.add_argument("--quality", type=int, default=75, help="JPEG quality, 1-100")
    args = parser.parse_args(argv)

    rclpy.init()
    node = PreviewNode(args.image_topic, args.detections_topic, args.quality)
    server = ThreadingHTTPServer((args.bind, args.port), PreviewHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(
        "preview on http://{0}:{1}/ (unauthenticated; stop it when finished)".format(
            args.bind, args.port
        ),
        flush=True,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGINT invalidates the rclpy context underneath the executor, which
        # surfaces as an RCLError rather than KeyboardInterrupt. Once the
        # context is gone that is a normal shutdown; anything else is real.
        if rclpy.ok():
            raise
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("preview stopped", flush=True)


if __name__ == "__main__":
    main()
