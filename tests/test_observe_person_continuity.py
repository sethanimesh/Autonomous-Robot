"""Adapter contracts without importing ROS or running camera inference."""

import ast
from collections import Counter, OrderedDict
from contextlib import redirect_stdout
import io
import itertools
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock

from robot.jetson.perception.person_continuity import PersonContinuity


def message(stamp=99.9, frame_id="camera", detections=()):
    sec = int(stamp)
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(sec=sec, nanosec=round((stamp - sec) * 1e9)),
        ),
        detections=list(detections),
        width=640, height=480, encoding="bgr8", step=1920, data=b"",
    )


def detection(x=320, y=240, width=200, height=300, class_id="person"):
    return SimpleNamespace(
        results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id=class_id))],
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
            size_x=width, size_y=height,
        ),
    )


def load_adapter(clock):
    # The executable defines its ROS class inside main. Extract its adapter
    # code, as existing mission tests do, so importing ROS cannot touch hardware.
    source = Path("scripts/diagnostics/observe_person_continuity.py").read_text()
    tree = ast.parse(source)
    selected = [node for node in ast.walk(tree)
                if ((isinstance(node, ast.FunctionDef)
                     and node.name in ("source_key", "normalized_boxes", "pose_signature", "pose_changed", "observe_for"))
                    or isinstance(node, ast.ClassDef) and node.name == "Observer")]

    class Node:
        def __init__(self, name):
            pass

        def create_subscription(self, *args):
            pass

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock["now"] * 1e9)))

    scope = dict(
        Node=Node, Image=object, String=object, Detection2DArray=object,
        qos_profile_sensor_data=object(), Counter=Counter, OrderedDict=OrderedDict,
        PersonContinuity=PersonContinuity, json=json, math=math,
        Path=Path,
        time=SimpleNamespace(monotonic=lambda: clock["now"]),
        # Pixel operations are irrelevant to empty-detection join tests.
        np=MagicMock(), person_appearance=lambda image, box: [1.0, 0.0],
    )
    exec(compile(ast.Module(body=selected, type_ignores=[]), "<continuity-adapter>", "exec"), scope)
    return scope


class ObservePersonContinuityTests(unittest.TestCase):
    def setUp(self):
        self.clock = {"now": 100.0}
        self.scope = load_adapter(self.clock)
        self.observer = self.scope["Observer"]()
        self.stop_status()

    def stop_status(self):
        self.head_status()
        self.robot_status()

    def head_status(self, position=10, reference_id="saved-reference", moving=False):
        self.observer.on_state("head", SimpleNamespace(data=json.dumps({
            "moving": moving, "homing": False, "position": position, "reference_id": reference_id,
        })))

    def robot_status(self, left=100, right=100):
        self.observer.on_state("robot", SimpleNamespace(data=json.dumps({
            "track_motion_active": False,
            "motors": {"left": {"position": left}, "right": {"position": right}},
        })))

    def triplet(self, stamp=99.9, frame_id="camera", order=("image", "people", "matches")):
        with redirect_stdout(io.StringIO()):
            for name in order:
                self.observer.offer(name, message(stamp, frame_id))

    def test_exact_triplet_joins_in_every_callback_order(self):
        for index, order in enumerate(itertools.permutations(("image", "people", "matches"))):
            self.triplet(99.80 + index * 0.01, order=order)
        self.assertEqual(6, self.observer.counts["exact_frame_triplets"])
        self.assertEqual(6, len(self.observer.samples))
        self.assertTrue(all(sample["state"] == "waiting_face" for sample in self.observer.samples))

    def test_matching_timestamp_from_other_camera_cannot_complete_triplet(self):
        self.observer.offer("image", message(frame_id="camera_a"))
        self.observer.offer("people", message(frame_id="camera_a"))
        self.observer.offer("matches", message(frame_id="camera_b"))
        self.assertEqual(0, self.observer.counts["exact_frame_triplets"])
        self.assertEqual([], self.observer.samples)

    def test_source_key_includes_camera_frame_identity(self):
        key = self.scope["source_key"]
        self.assertNotEqual(key(message(frame_id="camera_a")), key(message(frame_id="camera_b")))

    def test_nearby_timestamps_cannot_complete_triplet(self):
        self.observer.offer("image", message(99.80))
        self.observer.offer("people", message(99.81))
        self.observer.offer("matches", message(99.82))
        self.assertEqual(0, self.observer.counts["exact_frame_triplets"])

    def test_old_exact_triplet_does_not_become_fresh_on_delivery(self):
        self.triplet(99.0)
        self.assertEqual("lost", self.observer.samples[-1]["state"])
        self.assertEqual("stale_frame", self.observer.samples[-1]["reason"])
        self.assertIsNone(self.observer.ready_at)

    def test_ready_prints_once_on_fresh_triplet_even_without_a_face(self):
        output = io.StringIO()
        with redirect_stdout(output):
            for stamp in (99.8, 99.9):
                for name in ('image', 'people', 'matches'):
                    self.observer.offer(name, message(stamp))
        events = [json.loads(line) for line in output.getvalue().splitlines()]
        ready = [event for event in events if event.get('event') == 'ready']
        self.assertEqual(1, len(ready))
        self.assertEqual(99.8, ready[0]['source_time'])
        self.assertFalse(ready[0]['camera_only'])
        self.assertEqual(2, self.observer.counts['usable_frame_triplets'])

    def test_invalid_image_does_not_mark_ready(self):
        invalid = message()
        invalid.encoding = 'unsupported'
        self.observer.offer('image', invalid)
        self.observer.offer('people', message())
        self.observer.offer('matches', message())
        self.assertIsNone(self.observer.ready_at)
        self.assertEqual(1, self.observer.counts['invalid_image'])

    def test_camera_only_processes_without_ev3_status(self):
        self.observer = self.scope['Observer'](camera_only=True)
        self.triplet()
        self.assertEqual(1, self.observer.counts['usable_frame_triplets'])
        self.assertEqual('waiting_face', self.observer.samples[-1]['state'])

    def test_default_mode_requires_ev3_status(self):
        self.observer = self.scope['Observer']()
        self.triplet()
        self.assertIsNone(self.observer.ready_at)
        self.assertEqual(3, self.observer.counts['not_stationary_or_stale_status'])

    def test_camera_only_ignores_stale_stopped_telemetry(self):
        self.observer = self.scope['Observer'](camera_only=True)
        self.stop_status()
        self.clock['now'] = 105.0
        self.triplet(104.9)
        self.assertEqual(1, self.observer.counts['usable_frame_triplets'])

    def test_camera_only_reported_head_motion_blocks_and_clears_evidence(self):
        self.observer = self.scope['Observer'](camera_only=True)
        self.observer.offer('image', message())
        self.head_status(moving=True)
        self.assertTrue(all(not cache for cache in self.observer.cache.values()))
        self.triplet()
        self.assertIsNone(self.observer.ready_at)
        self.head_status(moving=False)
        self.triplet()
        self.assertEqual(1, self.observer.counts['usable_frame_triplets'])

    def test_camera_only_reported_track_motion_blocks(self):
        self.observer = self.scope['Observer'](camera_only=True)
        self.observer.on_state('robot', SimpleNamespace(data=json.dumps({'motion_active': True})))
        self.triplet()
        self.assertIsNone(self.observer.ready_at)

    def test_camera_only_resets_on_encoder_drift_and_reference_change(self):
        self.observer = self.scope['Observer'](camera_only=True)
        self.stop_status()
        self.observer.offer('image', message())
        self.robot_status(left=104)
        self.assertFalse(self.observer.cache['image'])
        self.observer.offer('image', message())
        self.head_status(reference_id='new-reference')
        self.assertFalse(self.observer.cache['image'])

    def test_camera_only_partial_status_still_learns_available_encoders(self):
        self.observer = self.scope['Observer'](camera_only=True)
        self.observer.on_state('robot', SimpleNamespace(data='{}'))
        self.robot_status()
        self.observer.offer('image', message())
        self.observer.on_state('robot', SimpleNamespace(data='{}'))
        self.assertTrue(self.observer.cache['image'])
        self.robot_status(left=104)
        self.assertFalse(self.observer.cache['image'])

    def test_report_records_readiness_and_saves_on_interrupt_without_double_shutdown(self):
        class ExternalShutdown(Exception):
            pass

        for error, context_alive in ((KeyboardInterrupt, True), (ExternalShutdown, False)):
            with self.subTest(error=error):
                self.triplet()
                self.observer.destroy_node = MagicMock()
                ros = MagicMock()
                ros.ok.side_effect = [True, context_alive]
                ros.spin_once.side_effect = error
                with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
                    path = Path(directory) / 'report.json'
                    self.scope['observe_for'](self.observer, ros, 15., path, ExternalShutdown)
                    report = json.loads(path.read_text())
                self.assertEqual('interrupted', report['completion'])
                self.assertTrue(report['readiness']['ready'])
                self.assertEqual(10., report['maximum_face_age_seconds'])
                self.assertGreaterEqual(report['counts']['head_messages'], 1)
                self.assertTrue(report['status_available']['robot'])
                self.observer.destroy_node.assert_called_once()
                self.assertEqual(int(context_alive), ros.shutdown.call_count)

    def test_motion_clears_incomplete_triplets(self):
        self.observer.offer("image", message())
        self.observer.offer("people", message())
        self.head_status(moving=True)
        self.assertTrue(all(not cache for cache in self.observer.cache.values()))
        self.stop_status()
        self.observer.offer("matches", message())
        self.assertEqual(0, self.observer.counts["exact_frame_triplets"])

    def test_stale_motor_status_blocks_triplet_processing(self):
        self.clock["now"] = 101.6
        self.triplet(101.5)
        self.assertEqual(0, self.observer.counts["exact_frame_triplets"])
        self.assertEqual(3, self.observer.counts["not_stationary_or_stale_status"])

    def test_stream_caches_remain_bounded_without_matching_detections(self):
        for index in range(40):
            self.observer.offer("image", message(99.0 + index / 100))
        self.assertEqual(24, len(self.observer.cache["image"]))
        self.assertEqual(16, self.observer.counts["image_evicted"])

    def test_target_change_clears_pending_old_target_evidence(self):
        self.observer.offer("image", message())
        self.observer.offer("people", message())
        self.observer.on_recognition(SimpleNamespace(data=json.dumps({"target_label": "new target"})))
        self.assertTrue(all(not cache for cache in self.observer.cache.values()))
        self.observer.offer("matches", message())
        self.assertEqual(0, self.observer.counts["exact_frame_triplets"])

    def test_cumulative_head_drift_resets_even_when_motion_flag_was_missed(self):
        self.observer.offer("image", message())
        for position in (11, 12, 13):
            self.head_status(position)
            self.assertEqual(1, len(self.observer.cache["image"]))
        self.head_status(14)
        self.assertTrue(all(not cache for cache in self.observer.cache.values()))

    def test_cumulative_track_encoder_change_resets_without_active_motion_flag(self):
        self.observer.offer("image", message())
        for position in (101, 102, 103):
            self.robot_status(left=position)
            self.assertEqual(1, len(self.observer.cache["image"]))
        self.robot_status(left=104)
        self.assertTrue(all(not cache for cache in self.observer.cache.values()))

    def test_referencing_camera_again_resets_even_when_position_is_unchanged(self):
        self.observer.offer("image", message())
        self.head_status(reference_id="replacement-reference")
        self.assertTrue(all(not cache for cache in self.observer.cache.values()))

    def test_pose_helpers_include_both_tracks_and_reference_identity(self):
        signature = self.scope["pose_signature"]
        changed = self.scope["pose_changed"]
        self.assertEqual((100, 200), signature("robot", {"motors": {
            "left": {"position": 100}, "right": {"position": 200},
        }}))
        self.assertFalse(changed("robot", (100, 200), (103, 197)))
        self.assertTrue(changed("robot", (100, 200), (100, 204)))
        self.assertTrue(changed("head", ("a", 10), ("b", 10)))
        self.assertTrue(changed("head", ("a", 10), ("a", float("nan"))))

    def test_detection_classes_are_filtered_before_association(self):
        boxes = self.scope["normalized_boxes"](
            [detection(class_id="target_person"), detection(class_id="person")],
            640, 480, "person",
        )
        self.assertEqual(1, len(boxes))
        self.assertAlmostEqual(220 / 640, boxes[0][0])

    def test_nonfinite_detection_cannot_be_clamped_into_a_real_person(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            for field in ("x", "y", "width", "height"):
                with self.subTest(value=value, field=field):
                    boxes = self.scope["normalized_boxes"]([detection(**{field: value})], 640, 480, "person")
                    self.assertEqual([], boxes)


if __name__ == "__main__":
    unittest.main()
