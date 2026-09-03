#!/usr/bin/env python3
"""Face alignment, quality, matching, and privacy-aware target storage."""

import json
import math
import os
import tempfile
import time
from collections import deque


STORE_SCHEMA = 1
SFACE_TEMPLATE = (
    (38.2946, 51.6963),
    (73.5318, 51.5014),
    (56.0252, 71.7366),
    (41.5493, 92.3655),
    (70.7299, 92.2041),
)
POSES = ("left", "center", "right")


class RecognitionError(RuntimeError):
    pass


def normalize_vector(values):
    clean = [float(value) for value in values]
    if not clean or any(not math.isfinite(value) for value in clean):
        raise RecognitionError("embedding must contain finite values")
    norm = math.sqrt(sum(value * value for value in clean))
    if norm <= 1e-12:
        raise RecognitionError("embedding norm is zero")
    return [value / norm for value in clean]


def cosine_similarity(left, right):
    if len(left) != len(right) or not left:
        raise RecognitionError("embedding dimensions do not match")
    return sum(float(a) * float(b) for a, b in zip(left, right))


def aggregate_similarity(probe, templates):
    """Blend the best view with the top-three consensus."""
    if not templates:
        return None
    scores = sorted((cosine_similarity(probe, item) for item in templates), reverse=True)
    top = scores[:3]
    return 0.70 * top[0] + 0.30 * (sum(top) / len(top))


def classify_pose(landmarks):
    if len(landmarks) != 5:
        return "unknown"
    left_eye, right_eye, nose, _left_mouth, _right_mouth = landmarks
    eye_distance = math.hypot(right_eye[0] - left_eye[0], right_eye[1] - left_eye[1])
    if eye_distance <= 1.0:
        return "unknown"
    midpoint = ((left_eye[0] + right_eye[0]) * 0.5, (left_eye[1] + right_eye[1]) * 0.5)
    yaw = (nose[0] - midpoint[0]) / eye_distance
    if yaw < -0.12:
        return "left"
    if yaw > 0.12:
        return "right"
    return "center"


def align_face(image, landmarks):
    import cv2
    import numpy

    if len(landmarks) != 5:
        raise RecognitionError("five landmarks are required for alignment")
    source = numpy.asarray(landmarks, dtype=numpy.float32)
    destination = numpy.asarray(SFACE_TEMPLATE, dtype=numpy.float32)
    transform, _inliers = cv2.estimateAffinePartial2D(
        source, destination, method=cv2.LMEDS
    )
    if transform is None or not numpy.isfinite(transform).all():
        raise RecognitionError("face alignment failed")
    return cv2.warpAffine(
        image,
        transform,
        (112, 112),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )


def assess_quality(image, face, minimum_face_pixels=72, minimum_blur=45.0):
    import cv2
    import numpy

    x1, y1, x2, y2 = [int(round(value)) for value in face["box"]]
    height, width = image.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    short_side = min(x2 - x1, y2 - y1)
    if short_side < int(minimum_face_pixels):
        return False, "move closer", {"face_pixels": short_side}
    if float(face.get("score", 0.0)) < 0.75:
        return False, "hold still", {"face_pixels": short_side}
    crop = image[y1:y2, x1:x2]
    if crop.size == 0:
        return False, "face is outside the image", {"face_pixels": short_side}
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    brightness = float(numpy.mean(gray))
    blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if brightness < 35.0:
        return False, "more light needed", {"face_pixels": short_side, "brightness": brightness, "blur": blur}
    if brightness > 225.0:
        return False, "too much light", {"face_pixels": short_side, "brightness": brightness, "blur": blur}
    if blur < float(minimum_blur):
        return False, "hold still", {"face_pixels": short_side, "brightness": brightness, "blur": blur}
    landmarks = face.get("landmarks", ())
    if len(landmarks) != 5:
        return False, "landmarks unavailable", {"face_pixels": short_side, "brightness": brightness, "blur": blur}
    eye_distance = math.hypot(
        landmarks[1][0] - landmarks[0][0], landmarks[1][1] - landmarks[0][1]
    )
    if eye_distance < short_side * 0.16 or eye_distance > short_side * 0.75:
        return False, "face angle is unclear", {"face_pixels": short_side, "brightness": brightness, "blur": blur}
    return True, "good", {"face_pixels": short_side, "brightness": brightness, "blur": blur}


class FaceEmbeddingRecognizer(object):
    def __init__(self, backend, input_mean=127.5, input_std=127.5):
        import numpy

        if (backend.input_height, backend.input_width) != (112, 112):
            raise RecognitionError("face embedding engine must accept 112x112 input")
        if len(backend.output_names) != 1:
            raise RecognitionError("face embedding engine must have one output")
        self.backend = backend
        self.provider = backend.provider
        self.device_name = backend.device_name
        self.output_name = backend.output_names[0]
        self.input_mean = float(input_mean)
        self.input_std = float(input_std)
        if self.input_std <= 0.0:
            raise RecognitionError("input_std must be positive")
        started = time.monotonic()
        backend.infer(numpy.zeros((1, 3, 112, 112), dtype=numpy.float32))
        self.warmup_seconds = time.monotonic() - started

    def embedding(self, image, landmarks):
        import numpy

        aligned = align_face(image, landmarks)
        rgb = aligned[:, :, ::-1]
        tensor = numpy.transpose(rgb, (2, 0, 1))[None].astype(numpy.float32)
        tensor = (tensor - self.input_mean) / self.input_std
        output = self.backend.infer(tensor)[self.output_name]
        return normalize_vector(numpy.asarray(output).reshape(-1).tolist()), aligned

    def close(self):
        self.backend.close()


class ConfirmationWindow(object):
    def __init__(self, size, required, threshold):
        self.size = int(size)
        self.required = int(required)
        self.threshold = float(threshold)
        if self.size < 1 or not 1 <= self.required <= self.size:
            raise RecognitionError("invalid confirmation window")
        self.values = deque(maxlen=self.size)

    def add(self, score):
        hit = score is not None and float(score) >= self.threshold
        self.values.append(bool(hit))
        return self.confirmed

    @property
    def hits(self):
        return sum(1 for value in self.values if value)

    @property
    def confirmed(self):
        return len(self.values) >= self.required and self.hits >= self.required

    def clear(self):
        self.values.clear()


class TargetStore(object):
    def __init__(self, path, model_id, model_sha256):
        self.path = os.path.abspath(path)
        self.model_id = str(model_id)
        self.model_sha256 = str(model_sha256)

    def load(self):
        if not os.path.isfile(self.path):
            return None
        with open(self.path, encoding="utf-8") as handle:
            payload = json.load(handle)
        if payload.get("schema") != STORE_SCHEMA:
            raise RecognitionError("unsupported target-store schema")
        if payload.get("model", {}).get("id") != self.model_id:
            raise RecognitionError("target was enrolled with a different model")
        if payload.get("model", {}).get("sha256") != self.model_sha256:
            raise RecognitionError("target model checksum does not match")
        label = payload.get("label")
        samples = payload.get("samples")
        if not isinstance(label, str) or not label.strip():
            raise RecognitionError("target label is missing")
        if not isinstance(samples, list) or not samples:
            raise RecognitionError("target has no samples")
        dimensions = None
        clean = []
        for sample in samples:
            if not isinstance(sample, dict):
                raise RecognitionError("target sample is malformed")
            embedding = normalize_vector(sample.get("embedding", []))
            if dimensions is None:
                dimensions = len(embedding)
            if len(embedding) != dimensions:
                raise RecognitionError("target embedding dimensions differ")
            clean.append(dict(sample, embedding=embedding))
        result = dict(payload)
        result["label"] = label.strip()
        result["samples"] = clean
        return result

    def save(self, label, samples, retention):
        label = str(label).strip()
        if not label or len(label) > 40:
            raise RecognitionError("target label must be 1-40 characters")
        clean_samples = []
        dimensions = None
        for sample in samples:
            embedding = normalize_vector(sample["embedding"])
            if dimensions is None:
                dimensions = len(embedding)
            if len(embedding) != dimensions:
                raise RecognitionError("embedding dimensions differ")
            clean_samples.append(
                {
                    "embedding": embedding,
                    "pose": str(sample.get("pose", "unknown")),
                    "source": str(sample.get("source", "unknown")),
                    "quality": dict(sample.get("quality", {})),
                    "photo": sample.get("photo"),
                }
            )
        if not clean_samples:
            raise RecognitionError("at least one sample is required")
        payload = {
            "schema": STORE_SCHEMA,
            "label": label,
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "model": {"id": self.model_id, "sha256": self.model_sha256},
            "retention": str(retention),
            "samples": clean_samples,
        }
        directory = os.path.dirname(self.path)
        os.makedirs(directory, mode=0o700, exist_ok=True)
        os.chmod(directory, 0o700)
        descriptor, temporary = tempfile.mkstemp(prefix=".target-", suffix=".json", dir=directory)
        try:
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, separators=(",", ":"), sort_keys=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
            os.chmod(self.path, 0o600)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return payload

    def delete(self):
        if os.path.exists(self.path):
            os.unlink(self.path)
            return True
        return False
