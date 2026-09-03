import json
import unittest

from robot.jetson.perception.face_detections import FaceDetection
from robot.jetson.perception.face_observations import FaceObservationError, parse_face_observations, serialize_face_observations


class Stamp(object):
    sec = 12
    nanosec = 345


class FaceObservationTests(unittest.TestCase):
    def test_round_trip_preserves_landmarks_and_stamp(self):
        face = FaceDetection(1, 2, 30, 40, 0.91, ((3, 4), (5, 4), (4, 6), (3, 8), (5, 8)))
        result = parse_face_observations(serialize_face_observations([face], Stamp(), "camera"))
        self.assertEqual((12, 345), result["key"])
        self.assertEqual("camera", result["frame_id"])
        self.assertEqual((1.0, 2.0, 30.0, 40.0), result["faces"][0]["box"])
        self.assertEqual(5, len(result["faces"][0]["landmarks"]))

    def test_non_finite_or_wrong_landmarks_are_rejected(self):
        payload = {"schema": 1, "stamp": {"sec": 1, "nanosec": 0}, "frame_id": "camera", "faces": [{"box": [0, 0, 2, 2], "score": 1, "landmarks": [[0, 0]]}]}
        with self.assertRaises(FaceObservationError):
            parse_face_observations(json.dumps(payload))

    def test_face_count_is_bounded(self):
        payload = {"schema": 1, "stamp": {"sec": 1, "nanosec": 0}, "frame_id": "camera", "faces": [{}] * 101}
        with self.assertRaises(FaceObservationError):
            parse_face_observations(json.dumps(payload))
