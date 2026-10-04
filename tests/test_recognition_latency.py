"""Exercise recognition event timing without ROS, models, or a camera."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock

from robot.jetson.mission.target_gate import (
    TargetGateError, build_target_observation, target_box_position,
)
from robot.jetson.perception.detections import Detection
from robot.jetson.perception.image_intake import frame_age_seconds, is_frame_too_old
from robot.jetson.perception.recognition_core import ConfirmationWindow, aggregate_similarity


def load_class(path, name, scope):
    tree = ast.parse(Path(path).read_text())
    node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), path, "exec"), scope)
    return scope[name]


def stamp(seconds):
    sec = int(seconds)
    return SimpleNamespace(sec=sec, nanosec=round((seconds - sec) * 1e9))


class RecognitionLatencyTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.events = []
        self.frame_number = 0
        self.os = SimpleNamespace(path=SimpleNamespace(getmtime=lambda _: 1))
        scope = dict(
            Node=object, String=SimpleNamespace, json=json, os=self.os,
            time=SimpleNamespace(monotonic=lambda: self.now),
            Detection=Detection, aggregate_similarity=aggregate_similarity,
            frame_age_seconds=frame_age_seconds, is_frame_too_old=is_frame_too_old,
            build_detection_array=lambda detections, frame_stamp, frame_id, factories: detections,
        )
        cls = load_class("robot/jetson/perception/target_recognizer.py", "TargetRecognizerNode", scope)
        self.node = cls.__new__(cls)
        self.node.config = SimpleNamespace(
            max_frame_age_sec=.5, max_faces=4, match_threshold=.45,
            target_store_path="unused", frame_timeout_sec=1.0,
            model_name="test", confirmation_required=3, confirmation_window=5,
        )
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9)))
        self.node.get_logger = MagicMock()
        self.node.target = {"label": "person", "created_utc": "enrollment-1",
                            "samples": [{"embedding": [1.0, 0.0]}]}
        self.node.store = SimpleNamespace(load=lambda: self.node.target)
        self.node._store_mtime = 1
        self.node.history = ConfirmationWindow(5, 3, .45)
        self.node.recognizer = SimpleNamespace(
            embedding=lambda image, landmarks: (landmarks, None),
            provider="offline", device_name="none")
        self.node.image_input = SimpleNamespace(reconnects=0, refresh_if_stale=lambda *args: False)
        self.node.observation_input = SimpleNamespace(reconnects=0, refresh_if_stale=lambda *args: False)
        self.node.last_observation_time = self.now
        self.node.status_publisher = SimpleNamespace(publish=lambda m: self.events.append(("status", json.loads(m.data))))
        self.node.matches_publisher = SimpleNamespace(publish=lambda m: self.events.append(("matches", m)))
        self.node.factories = None
        self.node.confirmed = False
        self.node.last_input_time = self.now
        self.node.last_image_time = self.now
        self.node.started = self.now
        self.node.last_score = self.node.last_latency_ms = self.node.last_error = None
        self.node.last_match_frame = None
        self.node.inferences = self.node.inference_errors = 0
        self.node.matches_published = self.node.pending_dropped = 0

    def infer(self, matching=True, source_time=None):
        self.frame_number += 1
        if source_time is None:
            source_time = 99.8 + self.frame_number * .01
        frame = SimpleNamespace(stamp=stamp(source_time), frame_id="camera", image=None)
        faces = [{"landmarks": [1.0, 0.0], "box": [100, 100, 180, 180]}] if matching else []
        self.node.pending = (frame, {"faces": faces})
        self.node.on_inference()

    def test_duplicate_frame_cannot_supply_second_household_vote(self):
        self.node.history = ConfirmationWindow(5, 2, .40)
        self.infer(source_time=99.9)
        self.infer(source_time=99.9)
        self.assertFalse(self.node.confirmed)
        self.assertEqual(1, self.node.matches_published)
        self.infer(source_time=99.95)
        self.assertTrue(self.node.confirmed)

    def test_confirmation_is_immediate_after_third_match_and_not_repeated(self):
        self.infer()
        self.infer()
        self.assertEqual(["matches", "matches"], [event[0] for event in self.events])
        self.infer(source_time=99.95)
        self.assertEqual(["matches", "status"], [event[0] for event in self.events[-2:]])
        payload = self.events[-1][1]
        self.assertEqual("target_confirmed", payload["state"])
        self.assertEqual(3, payload["matches_published"])
        self.assertEqual({"sec": 99, "nanosec": 950000000, "frame_id": "camera"}, payload["match_frame"])
        self.infer()
        self.assertEqual(1, len([event for event in self.events if event[0] == "status"]))

    def test_confirmation_loss_notifies_immediately_after_empty_match_batch(self):
        for _ in range(3):
            self.infer()
        for _ in range(3):
            self.infer(matching=False)
        self.assertEqual(("matches", []), self.events[-2])
        self.assertEqual("searching", self.events[-1][1]["state"])
        self.assertFalse(self.events[-1][1]["confirmation"]["confirmed"])
        self.infer(matching=False)
        self.assertEqual(2, len([event for event in self.events if event[0] == "status"]))

    def test_stale_batch_cannot_trigger_confirmation_or_publish_fresh_match(self):
        self.infer()
        self.infer()
        before = list(self.events)
        self.infer(source_time=98.0)
        self.assertEqual(before, self.events)
        self.assertFalse(self.node.confirmed)

    def test_periodic_health_still_marks_stale_input(self):
        for _ in range(3):
            self.infer()
        self.now += 2
        self.node.on_status()
        self.assertEqual("stale_input", self.events[-1][1]["state"])

    def test_stalled_observation_feed_reconnects_without_reloading_model(self):
        self.node.matcher = MagicMock()
        self.node.pending = 'old unmatched batch'
        self.node.get_logger = MagicMock()
        self.node.observation_input.refresh_if_stale = MagicMock(return_value=True)
        recognizer = self.node.recognizer
        self.now += 11
        self.node.on_status()
        self.node.observation_input.refresh_if_stale.assert_called_once_with(self.now,11.)
        self.node.matcher.clear.assert_called_once_with()
        self.assertIsNone(self.node.pending)
        self.assertIs(recognizer,self.node.recognizer)

    def test_reenrollment_same_label_invalidates_immediately_and_once(self):
        for _ in range(3):
            self.infer()
        replacement = dict(self.node.target, created_utc="enrollment-2")
        self.node.store.load = lambda: replacement
        self.os.path.getmtime = lambda _: 2
        before = len(self.events)
        self.node.on_status()
        self.assertEqual(before + 1, len(self.events))
        payload = self.events[-1][1]
        self.assertEqual("enrollment-2", payload["target_revision"])
        self.assertEqual("searching", payload["state"])
        self.assertIsNone(payload["match_frame"])
        self.assertEqual(0, self.node.history.hits)

    def test_initial_store_load_does_not_publish_before_publisher_exists(self):
        del self.node.status_publisher
        self.assertFalse(self.node.reload_target(force=True))
        self.assertEqual([], self.events)


class ObservationLatencyTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.messages = []

        class Node:
            def __init__(self, name):
                pass

            def create_publisher(node, *args):
                return SimpleNamespace(publish=lambda message: self.messages.append(json.loads(message.data)))

            def create_subscription(node, *args):
                pass

            def create_timer(node, *args):
                pass

            def get_clock(node):
                return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9)))

        scope = dict(
            Node=Node, String=SimpleNamespace, Detection2DArray=object,
            CameraInfo=object, qos_profile_sensor_data=object(), json=json,
            time=SimpleNamespace(monotonic=lambda: self.now),
            TargetGateError=TargetGateError, build_target_observation=build_target_observation,
            target_box_position=target_box_position,
        )
        cls = load_class("robot/jetson/mission/target_observer.py", "TargetObserverNode", scope)
        self.node = cls()
        self.node.on_info(SimpleNamespace(width=640, height=480))

    def status(self, confirmed=True, source_time=99.9, frame_id="camera", revision="enrollment-1"):
        frame_stamp = stamp(source_time)
        self.node.on_status(SimpleNamespace(data=json.dumps({
            "state": "target_confirmed" if confirmed else "searching",
            "confirmation": {"confirmed": confirmed},
            "target_label": "person", "target_revision": revision,
            "match_frame": {"sec": frame_stamp.sec, "nanosec": frame_stamp.nanosec, "frame_id": frame_id},
        })))

    def matches(self, source_time=99.9, present=True, frame_id="camera"):
        detection = SimpleNamespace(bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=160, y=120)),
            size_x=60, size_y=80))
        self.node.on_matches(SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(source_time), frame_id=frame_id),
            detections=[detection] if present else []))

    def test_confirmation_and_boxes_publish_without_timer_in_both_arrival_orders(self):
        for first, second in ((self.status, self.matches), (self.matches, self.status)):
            self.node.recognition_status = None
            self.node.match_frame_key = None
            self.node.box_heights = []
            first()
            self.assertFalse(self.messages[-1]["confirmed"])
            second()
            payload = self.messages[-1]
            self.assertTrue(payload["confirmed"])
            self.assertEqual(.1, payload["age_seconds"])
            self.assertEqual(.25, payload["center_x_fraction"])
            self.assertEqual("enrollment-1", payload["target_revision"])
            self.assertEqual("person", payload["target_label"])

    def test_confirmation_cannot_reuse_boxes_before_its_batch_or_from_other_camera(self):
        self.matches(source_time=99.8)
        self.status(source_time=99.9)
        self.assertFalse(self.messages[-1]["confirmed"])
        self.assertNotIn("center_x_fraction", self.messages[-1])
        self.matches(frame_id="other_camera")
        self.assertFalse(self.messages[-1]["confirmed"])
        self.matches()
        self.assertTrue(self.messages[-1]["confirmed"])
        self.matches(source_time=99.95)
        self.assertTrue(self.messages[-1]["confirmed"])

    def test_empty_batch_and_status_loss_are_immediate(self):
        self.status()
        self.matches()
        self.matches(present=False)
        self.assertFalse(self.messages[-1]["confirmed"])
        self.matches()
        self.status(confirmed=False, revision="enrollment-2")
        self.assertFalse(self.messages[-1]["confirmed"])
        self.assertEqual("enrollment-2", self.messages[-1]["target_revision"])

    def test_timer_ages_original_source_and_does_not_refresh_old_geometry(self):
        self.status()
        self.matches()
        self.now += 1
        self.node.publish_observation()
        self.assertEqual(1.1, self.messages[-1]["age_seconds"])
        self.now += 2
        self.node.publish_observation()
        self.assertFalse(self.messages[-1]["confirmed"])

    def test_invalid_status_does_not_trigger_an_observation(self):
        self.node.on_status(SimpleNamespace(data="[]"))
        self.node.on_status(SimpleNamespace(data="not json"))
        self.assertEqual([], self.messages)


if __name__ == "__main__":
    unittest.main()
