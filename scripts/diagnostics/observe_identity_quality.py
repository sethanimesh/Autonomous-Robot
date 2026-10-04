#!/usr/bin/env python3
"""Read live face quality and recognition scores without moving or storing images."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=25)
parser.add_argument('--report', required=True)
args = parser.parse_args()
rclpy.init()


class Observer(Node):
    def __init__(self):
        super().__init__('echora_identity_quality_observer')
        self.frames, self.faces = {}, {}
        self.samples, self.scores = [], []
        self.head, self.robot = {}, {}
        self.face_batches = self.empty_face_batches = 0
        self.status = {}
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/perception/face_detections', self.on_faces, 10)
        self.create_subscription(String, '/camera_head/status', lambda m: self.on_state('head', m), 10)
        self.create_subscription(String, '/robot_status', lambda m: self.on_state('robot', m), 10)
        self.create_subscription(String, '/perception/recognition_status', self.on_recognition, 10)
        self.create_subscription(String, '/perception/face_status', self.on_detector, 10)

    @staticmethod
    def key(m):
        return m.header.stamp.sec, m.header.stamp.nanosec

    @staticmethod
    def trim(values):
        while len(values) > 12:
            del values[next(iter(values))]

    def on_state(self, name, message):
        setattr(self, name, json.loads(message.data))

    def on_detector(self, message):
        self.status['detector'] = json.loads(message.data)

    def on_recognition(self, message):
        status = json.loads(message.data)
        self.status['recognition'] = status
        self.scores.append({k: status.get(k) for k in ('state', 'last_score', 'confirmation', 'last_batch_latency_ms')})

    def on_image(self, message):
        key = self.key(message)
        self.frames[key] = message
        self.trim(self.frames)
        self.observe(key)

    def on_faces(self, message):
        self.face_batches += 1
        self.empty_face_batches += int(not message.detections)
        key = self.key(message)
        self.faces[key] = message
        self.trim(self.faces)
        self.observe(key)

    def observe(self, key):
        if key not in self.frames or key not in self.faces:
            return
        frame, faces = self.frames.pop(key), self.faces.pop(key)
        if not faces.detections or self.head.get('moving', True) or self.robot.get('motion_active', True):
            return
        if frame.encoding not in ('rgb8', 'bgr8'):
            return
        pixels = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.step)
        pixels = pixels[:, :frame.width*3].reshape(frame.height, frame.width, 3)
        gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY if frame.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY)
        face = max(faces.detections, key=lambda d: d.bbox.size_x*d.bbox.size_y)
        box = face.bbox
        x, y = box.center.position.x, box.center.position.y
        x1, x2 = max(0, int(x-box.size_x/2)), min(frame.width, int(x+box.size_x/2))
        y1, y2 = max(0, int(y-box.size_y/2)), min(frame.height, int(y+box.size_y/2))
        crop = gray[y1:y2, x1:x2]
        if crop.size == 0:
            return
        self.samples.append(dict(head_position=self.head.get('position'),
                                 frame_mean_luma=round(float(gray.mean()), 2),
                                 face_mean_luma=round(float(crop.mean()), 2),
                                 face_dark_fraction=round(float((crop < 40).mean()), 3),
                                 face_laplacian_variance=round(float(cv2.Laplacian(crop, cv2.CV_64F).var()), 2),
                                 face_width=x2-x1, face_height=y2-y1,
                                 face_score=face.results[0].hypothesis.score if face.results else None))


node = Observer()
try:
    until = time.monotonic()+args.seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    report = dict(face_batches=node.face_batches, empty_face_batches=node.empty_face_batches,
                  stationary_face_samples=node.samples, recognition_samples=node.scores,
                  final_status=node.status, images_stored=False)
    if node.samples:
        report['median_face_mean_luma'] = float(np.median([s['face_mean_luma'] for s in node.samples]))
        report['median_frame_mean_luma'] = float(np.median([s['frame_mean_luma'] for s in node.samples]))
    path = Path(args.report)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('stationary_face_samples', 'recognition_samples', 'final_status')}))
finally:
    node.destroy_node()
    rclpy.shutdown()
