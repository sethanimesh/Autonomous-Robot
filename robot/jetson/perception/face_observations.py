#!/usr/bin/env python3
"""Lossless JSON transport for face boxes and five alignment landmarks."""

import json


class FaceObservationError(ValueError):
    pass


def _finite_number(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise FaceObservationError("{0} must be numeric".format(name))
    if value != value or value in (float("inf"), float("-inf")):
        raise FaceObservationError("{0} must be finite".format(name))
    return value


def serialize_face_observations(faces, stamp, frame_id):
    payload = {
        "schema": 1,
        "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
        "frame_id": str(frame_id),
        "faces": [],
    }
    for face in faces:
        payload["faces"].append(
            {
                "box": [face.x1, face.y1, face.x2, face.y2],
                "score": face.score,
                "landmarks": [[point[0], point[1]] for point in face.landmarks],
            }
        )
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def parse_face_observations(value):
    try:
        payload = json.loads(value)
    except (TypeError, ValueError) as exc:
        raise FaceObservationError("invalid JSON: {0}".format(exc))
    if not isinstance(payload, dict) or payload.get("schema") != 1:
        raise FaceObservationError("unsupported face-observation schema")
    stamp = payload.get("stamp")
    if not isinstance(stamp, dict):
        raise FaceObservationError("stamp is missing")
    try:
        sec = int(stamp["sec"])
        nanosec = int(stamp["nanosec"])
    except (KeyError, TypeError, ValueError):
        raise FaceObservationError("stamp is malformed")
    if sec < 0 or not 0 <= nanosec < 1000000000:
        raise FaceObservationError("stamp is out of range")
    frame_id = payload.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise FaceObservationError("frame_id is missing")
    raw_faces = payload.get("faces")
    if not isinstance(raw_faces, list) or len(raw_faces) > 100:
        raise FaceObservationError("faces must be a bounded list")
    faces = []
    for index, item in enumerate(raw_faces):
        if not isinstance(item, dict):
            raise FaceObservationError("face {0} is malformed".format(index))
        box = item.get("box")
        landmarks = item.get("landmarks")
        if not isinstance(box, list) or len(box) != 4:
            raise FaceObservationError("face {0} box is malformed".format(index))
        if not isinstance(landmarks, list) or len(landmarks) != 5:
            raise FaceObservationError("face {0} landmarks are malformed".format(index))
        clean_landmarks = []
        for point in landmarks:
            if not isinstance(point, list) or len(point) != 2:
                raise FaceObservationError("face {0} landmark is malformed".format(index))
            clean_landmarks.append(
                (_finite_number(point[0], "landmark x"), _finite_number(point[1], "landmark y"))
            )
        clean_box = tuple(
            _finite_number(number, "box") for number in box
        )
        if clean_box[2] <= clean_box[0] or clean_box[3] <= clean_box[1]:
            raise FaceObservationError("face {0} box is empty".format(index))
        faces.append(
            {
                "box": clean_box,
                "score": _finite_number(item.get("score"), "score"),
                "landmarks": tuple(clean_landmarks),
            }
        )
    return {
        "key": (sec, nanosec),
        "sec": sec,
        "nanosec": nanosec,
        "frame_id": frame_id,
        "faces": faces,
    }
