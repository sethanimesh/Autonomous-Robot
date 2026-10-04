#!/usr/bin/env python3
"""Record a bounded, opt-in live ROS run for portable offline replay.

This process only subscribes: start the mission separately in the normal UI.
It records camera images (including visible people) locally; never uploads them.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import queue
import sqlite3
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
SCHEMA = 'echora.mission_recording'
VERSION = 1
TOPICS = {
    '/camera/image_raw': 'sensor_msgs/msg/Image',
    '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/camera/status': 'std_msgs/msg/String',
    '/robot_status': 'std_msgs/msg/String',
    '/camera_head/status': 'std_msgs/msg/String',
    '/camera_head/command': 'std_msgs/msg/String',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/odom': 'nav_msgs/msg/Odometry',
    '/perception/person_detections': 'vision_msgs/msg/Detection2DArray',
    '/perception/face_detections': 'vision_msgs/msg/Detection2DArray',
    '/perception/people_tracks': 'std_msgs/msg/String',
    '/mission/target_observation': 'std_msgs/msg/String',
    '/perception/recognition_status': 'std_msgs/msg/String',
}
# Deliberately do not subscribe to family_matches or face_observations.
PRIVATE_KEYS = {'embedding', 'embeddings', 'face_embedding', 'face_embeddings',
                'template', 'templates', 'face_templates', 'face_image', 'face_images'}


def sanitize(value):
    """Preserve decision evidence without copying enrollment vectors into the run."""
    if isinstance(value, dict):
        return {k: sanitize(v) for k, v in value.items() if k.lower() not in PRIVATE_KEYS}
    if isinstance(value, (list, tuple)):
        return [sanitize(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def message_data(message, type_name, convert):
    """String JSON topics are decoded; typed messages retain their ROS field shape."""
    if type_name == 'std_msgs/msg/String':
        try:
            return sanitize(json.loads(message.data))
        except (ValueError, TypeError):
            return str(message.data)
    return sanitize(convert(message))


def read_profiles(path):
    """Snapshot only profile labels and revisions from the existing family store."""
    if path is None:
        return []
    connection = sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro', uri=True)
    try:
        columns = {r[1] for r in connection.execute('PRAGMA table_info(profiles)')}
        selected = [c for c in ('id', 'label', 'name', 'revision', 'enrollment_revision') if c in columns]
        if 'id' not in selected:
            raise ValueError('Family database does not contain profile metadata')
        return [dict(zip(selected, row)) for row in connection.execute(
            'SELECT ' + ','.join(selected) + ' FROM profiles')]
    finally:
        connection.close()


class RecordingWriter:
    """Single-worker writer. Receipt times are captured before queuing work."""
    def __init__(self, output, *, seconds=180., fps=5., max_bytes=512*1024*1024,
                 jpeg_quality=90, started_at_unix=None, started_monotonic_ns=None,
                 source_clock_unix=None, profiles=None):
        if not 0 < seconds <= 3600 or not 0 < fps <= 30:
            raise ValueError('Use 0 < duration <= 3600 seconds and 0 < FPS <= 30')
        if not isinstance(max_bytes, int) or max_bytes < 65536 or not 30 <= jpeg_quality <= 100:
            raise ValueError('Storage budget must be at least 64 KiB and JPEG quality 30–100')
        self.path = Path(output)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'frames').mkdir()
        self.events = (self.path / 'events.jsonl').open('wb')
        self.started_ns = time.monotonic_ns() if started_monotonic_ns is None else started_monotonic_ns
        self.started_at = time.time() if started_at_unix is None else started_at_unix
        self.seconds, self.fps, self.max_bytes = seconds, fps, max_bytes
        self.jpeg_quality = jpeg_quality
        self.bytes_written = 0
        self.sequence = 0
        self.frames = 0
        self.counts = collections.Counter()
        self.drops = collections.Counter()
        self.stop_reason = None
        self.closed = False
        self.manifest = dict(schema=SCHEMA, version=VERSION, started_at_unix=self.started_at,
            source_clock_unix=source_clock_unix, topics=TOPICS, image_topic='/camera/image_raw',
            image_encoding='jpeg', jpeg_quality=jpeg_quality, images_are_lossy=True,
            event_time='monotonic receipt elapsed_ns; header stamps preserved unchanged',
            limits=dict(seconds=seconds, fps=fps, max_bytes=max_bytes), profiles=profiles or [],
            commands_sent=False, cloud_requests_sent=False, biometric_embeddings_saved=False,
            raw_camera_images_saved=True, status='recording')
        self._write_manifest()

    def _write_manifest(self):
        self.manifest.update(events=self.sequence, frames=self.frames, counts=dict(self.counts),
            drops=dict(self.drops), bytes_written=self.bytes_written, stop_reason=self.stop_reason)
        temporary = self.path / 'manifest.tmp'
        temporary.write_text(json.dumps(self.manifest, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.path / 'manifest.json')

    def _append(self, topic, kind, data, received_at_unix, received_monotonic_ns, jpeg=None):
        if self.closed or self.stop_reason:
            return False
        elapsed = max(0, received_monotonic_ns - self.started_ns)
        if elapsed > int(self.seconds * 1e9):
            self.stop_reason = 'duration_limit'
            return False
        event = dict(sequence=self.sequence, elapsed_ns=elapsed,
            received_at_unix=received_at_unix, topic=topic, type=TOPICS[topic], kind=kind, data=sanitize(data))
        encoded = (json.dumps(event, separators=(',', ':'), allow_nan=False) + '\n').encode()
        required = len(encoded) + (len(jpeg) if jpeg is not None else 0)
        # Reserve 16 KiB for the final manifest, including drop counters.
        if self.bytes_written + required + 16384 > self.max_bytes:
            self.stop_reason = 'storage_limit'
            return False
        if jpeg is not None:
            (self.path / data['path']).write_bytes(jpeg)
            self.frames += 1
        self.events.write(encoded)
        self.events.flush()
        self.bytes_written += required
        self.counts[topic] += 1
        self.sequence += 1
        return True

    def record_message(self, topic, data, *, received_at_unix, received_monotonic_ns):
        if topic not in TOPICS or TOPICS[topic] == 'sensor_msgs/msg/Image':
            raise ValueError('Unsupported recording message topic')
        return self._append(topic, 'message', data, received_at_unix, received_monotonic_ns)

    def record_jpeg(self, jpeg, *, header, width, height, source_encoding,
                    received_at_unix, received_monotonic_ns):
        data = dict(path='frames/{:08d}.jpg'.format(self.frames), width=width, height=height,
                    encoding='jpeg', source_encoding=source_encoding, header=header)
        return self._append('/camera/image_raw', 'frame', data, received_at_unix,
                            received_monotonic_ns, jpeg=jpeg)

    def close(self, reason='completed'):
        if self.closed:
            return
        self.closed = True
        self.stop_reason = self.stop_reason or reason
        self.events.close()
        self.manifest.update(status='complete', finished_at_unix=time.time())
        self._write_manifest()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-camera', action='store_true', required=True,
                        help='Explicitly opt in to saving local camera images of this run')
    parser.add_argument('--output', type=Path, required=True, help='New recording directory')
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--fps', type=float, default=5.)
    parser.add_argument('--max-mb', type=float, default=512.)
    parser.add_argument('--jpeg-quality', type=int, default=90)
    parser.add_argument('--family-database', type=Path)
    return parser.parse_args(argv)


def record_live(args):
    # Import ROS only on an explicitly requested live capture; portable tests do not need it.
    from robot.jetson.perception.image_subscription import configure_perception_transport
    configure_perception_transport()
    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    profiles = read_profiles(args.family_database)
    rclpy.init()
    node = rclpy.create_node('echora_mission_recorder')
    writer = None
    worker = None
    work = queue.Queue(maxsize=64)
    done = threading.Event()
    errors = []
    subscriptions = []
    last_frame_ns = [None]
    try:
        writer = RecordingWriter(args.output, seconds=args.seconds, fps=args.fps,
            max_bytes=int(args.max_mb * 1024 * 1024), jpeg_quality=args.jpeg_quality,
            source_clock_unix=node.get_clock().now().nanoseconds / 1e9, profiles=profiles)
        bridge = CvBridge()

        def process():
            while not done.is_set() or not work.empty():
                try:
                    topic, message, unix, monotonic = work.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    if writer.stop_reason:
                        continue
                    if topic == '/camera/image_raw':
                        frame = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                        if not ok:
                            raise ValueError('JPEG compression failed')
                        writer.record_jpeg(jpeg.tobytes(), header=message_to_ordereddict(message.header),
                            width=message.width, height=message.height, source_encoding=message.encoding,
                            received_at_unix=unix, received_monotonic_ns=monotonic)
                    else:
                        writer.record_message(topic, message_data(message, TOPICS[topic], message_to_ordereddict),
                            received_at_unix=unix, received_monotonic_ns=monotonic)
                except OSError as exc:
                    errors.append(type(exc).__name__ + ': ' + str(exc))
                    writer.stop_reason = 'write_error'
                    done.set()
                except Exception as exc:
                    writer.drops['decode_error'] += 1
                    # Keep bad camera frames from losing the rest of the synchronized record.
                    if len(errors) < 10:
                        errors.append(type(exc).__name__ + ': ' + str(exc))
                finally:
                    work.task_done()

        def offer(topic, message):
            now = time.monotonic_ns()
            if done.is_set() or writer.stop_reason:
                return
            if topic == '/camera/image_raw':
                if last_frame_ns[0] is not None and now - last_frame_ns[0] < 1e9 / args.fps:
                    writer.drops['frame_rate_limit'] += 1
                    return
                last_frame_ns[0] = now
            try:
                work.put_nowait((topic, message, time.time(), now))
            except queue.Full:
                writer.drops['queue_full:' + topic] += 1

        worker = threading.Thread(target=process, name='mission-record-writer', daemon=True)
        worker.start()
        for topic, type_name in TOPICS.items():
            # Best effort accepts both reliable and camera best-effort publishers.
            subscriptions.append(node.create_subscription(get_message(type_name), topic,
                lambda message, topic=topic: offer(topic, message), qos_profile_sensor_data))
        print(json.dumps(dict(recording=str(args.output), seconds=args.seconds, fps=args.fps,
                              commands_sent=False)), flush=True)
        while rclpy.ok() and not writer.stop_reason and not done.is_set():
            if time.monotonic_ns() - writer.started_ns >= args.seconds * 1e9:
                break
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        if writer:
            writer.stop_reason = writer.stop_reason or 'interrupted'
    except Exception as exc:
        if writer:
            writer.stop_reason = writer.stop_reason or 'recorder_error'
            errors.append(type(exc).__name__ + ': ' + str(exc))
        raise
    finally:
        done.set()
        if worker:
            worker.join()
        if writer:
            writer.manifest['diagnostics'] = errors
            writer.close('duration_limit')
        node.destroy_node()
        # ROS may already have shut its context down when handling SIGINT.
        rclpy.try_shutdown()
    return writer.manifest


def main(argv=None):
    args = parse_args(argv)
    try:
        report = record_live(args)
    except (ImportError, ValueError, OSError) as exc:
        raise SystemExit(str(exc))
    print(json.dumps(dict(recording=str(args.output), frames=report['frames'],
                         events=report['events'], stop_reason=report['stop_reason'], commands_sent=False)))
    return 1 if report['stop_reason'] == 'write_error' else 0


if __name__ == '__main__':
    raise SystemExit(main())
