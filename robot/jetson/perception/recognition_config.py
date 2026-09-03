#!/usr/bin/env python3
"""Validated configuration for target-person recognition."""

try:
    from detector_config import DetectorConfigError, _bounded_float, _positive_int, _text, _topic
except ImportError:  # pragma: no cover
    from .detector_config import DetectorConfigError, _bounded_float, _positive_int, _text, _topic


PARAMETER_DEFAULTS = {
    "image_topic": "/camera/image_raw",
    "face_observations_topic": "/perception/face_observations",
    "matches_topic": "/perception/target_matches",
    "status_topic": "/perception/recognition_status",
    "model_name": "antelopev2_glintr100",
    "model_path": "/home/animesh/echora/models/glintr100_fp16.engine",
    "model_sha256": "",
    "target_store_path": "/home/animesh/echora/data/target_person.json",
    "input_mean": 127.5,
    "input_std": 127.5,
    "match_threshold": 0.45,
    "confirmation_window": 5,
    "confirmation_required": 3,
    "max_inference_rate_hz": 8.0,
    "max_faces": 3,
    "image_cache_size": 24,
    "max_frame_age_sec": 0.5,
    "frame_timeout_sec": 2.0,
    "status_interval_sec": 2.0,
}


class RecognitionConfig(object):
    def __init__(self, **overrides):
        unknown = sorted(set(overrides) - set(PARAMETER_DEFAULTS))
        if unknown:
            raise DetectorConfigError("unknown recognition parameters: {0}".format(", ".join(unknown)))
        values = dict(PARAMETER_DEFAULTS)
        values.update(overrides)
        for name in ("image_topic", "face_observations_topic", "matches_topic", "status_topic"):
            setattr(self, name, _topic(name, values[name]))
        if len({self.image_topic, self.face_observations_topic, self.matches_topic, self.status_topic}) != 4:
            raise DetectorConfigError("recognition topics must be unique")
        self.model_name = _text("model_name", values["model_name"])
        self.model_path = _text("model_path", values["model_path"])
        self.model_sha256 = _text("model_sha256", values["model_sha256"])
        if len(self.model_sha256) != 64 or any(character not in "0123456789abcdef" for character in self.model_sha256.lower()):
            raise DetectorConfigError("model_sha256 must be a lowercase SHA-256 digest")
        self.model_sha256 = self.model_sha256.lower()
        self.target_store_path = _text("target_store_path", values["target_store_path"])
        self.input_mean = _bounded_float("input_mean", values["input_mean"], -10000.0, 10000.0, True)
        self.input_std = _bounded_float("input_std", values["input_std"], 0.0, 10000.0)
        self.match_threshold = _bounded_float("match_threshold", values["match_threshold"], -1.0, 1.0, True)
        self.confirmation_window = _positive_int("confirmation_window", values["confirmation_window"], 100)
        self.confirmation_required = _positive_int("confirmation_required", values["confirmation_required"], 100)
        if self.confirmation_required > self.confirmation_window:
            raise DetectorConfigError("confirmation_required cannot exceed confirmation_window")
        self.max_inference_rate_hz = _bounded_float("max_inference_rate_hz", values["max_inference_rate_hz"], 0.0, 60.0)
        self.max_faces = _positive_int("max_faces", values["max_faces"], 20)
        self.image_cache_size = _positive_int("image_cache_size", values["image_cache_size"], 1000)
        self.max_frame_age_sec = _bounded_float("max_frame_age_sec", values["max_frame_age_sec"], 0.0, 60.0)
        self.frame_timeout_sec = _bounded_float("frame_timeout_sec", values["frame_timeout_sec"], 0.0, 600.0)
        if self.frame_timeout_sec < self.max_frame_age_sec:
            raise DetectorConfigError("frame_timeout_sec must be at least max_frame_age_sec")
        self.status_interval_sec = _bounded_float("status_interval_sec", values["status_interval_sec"], 0.0, 3600.0)

    @classmethod
    def from_mapping(cls, mapping):
        return cls(**dict(mapping))

    def inference_period_sec(self):
        return 1.0 / self.max_inference_rate_hz
