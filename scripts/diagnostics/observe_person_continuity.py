#!/usr/bin/env python3
"""Stationary face-to-body tracking evaluation. Ask the operator before running.

Subscribes only: no ROS publishers, no motor control, no stored images or
appearance vectors. This diagnostic is deliberately outside mission control.
"""

import argparse
from collections import Counter, OrderedDict
import json
import math
from pathlib import Path
import time

import numpy as np

try:
    from robot.jetson.perception.person_continuity import PersonContinuity, person_appearance
except ImportError:
    from person_continuity import PersonContinuity, person_appearance


def source_key(message):
    return (message.header.frame_id, message.header.stamp.sec, message.header.stamp.nanosec)


def normalized_boxes(detections, width, height, class_id):
    boxes = []
    for detection in detections:
        if not any(r.hypothesis.class_id == class_id for r in detection.results):
            continue
        b = detection.bbox
        x, y = b.center.position.x, b.center.position.y
        if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in (x, y, b.size_x, b.size_y)) or b.size_x <= 0 or b.size_y <= 0):
            continue
        boxes.append((max(0., (x-b.size_x/2)/width), max(0., (y-b.size_y/2)/height),
                      min(1., (x+b.size_x/2)/width), min(1., (y+b.size_y/2)/height)))
    return boxes


def pose_signature(name, value):
    def encoder(position):
        return position if (isinstance(position, (int, float)) and not isinstance(position, bool)
                            and math.isfinite(position)) else None

    if name == 'head':
        return (value.get('reference_id'), encoder(value.get('position')))
    motors = value.get('motors', {})
    return tuple(encoder(motors.get(side, {}).get('position')) for side in ('left', 'right'))


def pose_changed(name, previous, current, require_complete=True):
    if name == 'head':
        if previous[0] != current[0] and (require_complete or current[0] is not None):
            return True
        previous, current = previous[1:], current[1:]
    for before, after in zip(previous, current):
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (before, after)):
            if require_complete:
                return True
            continue
        if abs(after-before) > 3:
            return True
    return False


def observe_for(node, ros, seconds, report_path, shutdown_exception):
    """Always save diagnostic evidence, including an operator's early stop."""
    completion = 'completed'
    try:
        until = time.monotonic()+seconds
        while ros.ok() and time.monotonic() < until:
            ros.spin_once(node, timeout_sec=.1)
        if not ros.ok():
            completion = 'interrupted'
    except (KeyboardInterrupt, shutdown_exception):
        completion = 'interrupted'
    except Exception:
        completion = 'error'
        raise
    finally:
        report = {
            'scope': 'Stationary diagnostic only; body continuity does not authorize motion',
            'camera_only': node.camera_only, 'completion': completion,
            'maximum_face_age_seconds': node.tracker.maximum_face_age_seconds,
            'images_stored': False, 'appearance_vectors_stored': False,
            'readiness': {'ready': node.ready_at is not None,
                          'first_usable_triplet_after_seconds': node.ready_at},
            'status_available': {'head': node.head is not None, 'robot': node.robot is not None},
            'counts': dict(node.counts), 'samples': node.samples,
        }
        try:
            destination = Path(report_path)
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(report, indent=2)+'\n')
        finally:
            node.destroy_node()
            if ros.ok():
                ros.shutdown()
    print(json.dumps(report['counts']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=45.)
    parser.add_argument('--report', required=True)
    parser.add_argument('--camera-only', action='store_true',
                        help='Read-only stationary test without requiring EV3 status; leave the robot untouched')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('duration must be between 1 and 120 seconds')

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray

    class Observer(Node):
        def __init__(self, camera_only=False):
            super().__init__('echora_person_continuity_observer')
            self.camera_only = camera_only
            self.started_at = time.monotonic()
            self.ready_at = None
            self.cache = {name: OrderedDict() for name in ('image', 'people', 'matches')}
            self.tracker = PersonContinuity()
            self.layout = None
            self.target_label = None
            self.pose_baselines = {}
            self.head = self.robot = None
            self.head_at = self.robot_at = 0.
            self.counts = Counter()
            self.samples = []
            self.last_state = None
            self.create_subscription(Image, '/camera/image_raw', lambda m: self.offer('image', m), qos_profile_sensor_data)
            self.create_subscription(Detection2DArray, '/perception/person_detections', lambda m: self.offer('people', m), 10)
            self.create_subscription(Detection2DArray, '/perception/target_matches', lambda m: self.offer('matches', m), 10)
            self.create_subscription(String, '/camera_head/status', lambda m: self.on_state('head', m), 10)
            self.create_subscription(String, '/robot_status', lambda m: self.on_state('robot', m), 10)
            self.create_subscription(String, '/perception/recognition_status', self.on_recognition, 10)

        def on_state(self, name, message):
            self.counts[name+'_messages'] += 1
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    self.counts[name+'_invalid_status'] += 1
                    return
                setattr(self, name, value)
                setattr(self, name+'_at', time.monotonic())
            except ValueError:
                self.counts[name+'_invalid_status'] += 1
                return
            signature = pose_signature(name, value)
            previous = self.pose_baselines.get(name)
            if previous is None or pose_changed(name, previous, signature,
                                                require_complete=not self.camera_only):
                self.reset()
                self.pose_baselines[name] = signature
            elif self.camera_only:
                # Start comparing an encoder once its first reading arrives;
                # keep existing baselines so small cumulative drift is caught.
                self.pose_baselines[name] = tuple(after if before is None else before
                                                 for before, after in zip(previous, signature))
            if not self.stationary():
                self.reset()

        def on_recognition(self, message):
            try:
                value = json.loads(message.data)
                label = value.get('target_label')
            except (ValueError, AttributeError):
                return
            if label != self.target_label:
                self.reset()
                self.target_label = label

        def stationary(self):
            if self.camera_only:
                # Missing/offline EV3 telemetry is not evidence of movement in
                # this explicitly stationary, subscription-only diagnostic.
                # A reported movement still invalidates the appearance track.
                head, robot = self.head or {}, self.robot or {}
                return (not any(head.get(key) is True for key in ('moving', 'homing'))
                        and robot.get('track_motion_active', robot.get('motion_active')) is not True)
            now = time.monotonic()
            return (self.head is not None and self.robot is not None
                    and now-self.head_at <= 1.5 and now-self.robot_at <= 1.5
                    and not self.head.get('moving', True) and not self.head.get('homing', True)
                    and not self.robot.get('track_motion_active', self.robot.get('motion_active', True)))

        def reset(self):
            self.tracker.reset()
            for cache in self.cache.values():
                cache.clear()

        def offer(self, name, message):
            self.counts[name+'_messages'] += 1
            if not self.stationary():
                self.reset()
                self.counts['not_stationary_or_stale_status'] += 1
                return
            key = source_key(message)
            own = self.cache[name]
            own[key] = message
            while len(own) > 24:
                own.popitem(last=False)
                self.counts[name+'_evicted'] += 1
            if not all(key in cache for cache in self.cache.values()):
                return
            frame, people, matches = (self.cache[n].pop(key) for n in ('image', 'people', 'matches'))
            self.counts['exact_frame_triplets'] += 1
            layout = (key[0], frame.width, frame.height, frame.encoding)
            if self.layout != layout:
                self.reset()
                self.layout = layout
            if frame.width <= 0 or frame.height <= 0 or frame.encoding not in ('rgb8', 'bgr8'):
                self.counts['invalid_image'] += 1
                self.reset()
                return
            try:
                pixels = np.frombuffer(frame.data, np.uint8).reshape(frame.height, frame.step)
                pixels = pixels[:, :frame.width*3].reshape(frame.height, frame.width, 3)
            except ValueError:
                self.counts['invalid_image'] += 1
                self.reset()
                return
            if frame.encoding == 'rgb8':
                pixels = pixels[:, :, ::-1]
            persons = []
            for box in normalized_boxes(people.detections, frame.width, frame.height, 'person'):
                descriptor = person_appearance(pixels, box)
                if descriptor is not None:
                    persons.append({'box': box, 'appearance': descriptor})
            face_boxes = normalized_boxes(matches.detections, frame.width, frame.height, 'target_person')
            stamp = key[1]+key[2]/1e9
            now = self.get_clock().now().nanoseconds/1e9
            result = self.tracker.observe(persons, face_boxes, stamp, now)
            if -.05 <= now-stamp <= .75:
                self.counts['usable_frame_triplets'] += 1
                if self.ready_at is None:
                    self.ready_at = time.monotonic()-self.started_at
                    print(json.dumps({'event': 'ready', 'camera_only': self.camera_only,
                                      'elapsed_seconds': self.ready_at, 'source_time': stamp}), flush=True)
            self.counts[result['state']] += 1
            sample = dict(result, source_time=stamp, person_count=len(persons), matched_face_count=len(face_boxes))
            self.samples.append(sample)
            state = (result['state'], result.get('reason'))
            if state != self.last_state:
                print(json.dumps(sample), flush=True)
                self.last_state = state

    rclpy.init()
    node = Observer(camera_only=args.camera_only)
    observe_for(node, rclpy, args.seconds, args.report, ExternalShutdownException)


if __name__ == '__main__':
    main()
