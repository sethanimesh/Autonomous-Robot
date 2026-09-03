#!/usr/bin/env python3
"""Validated configuration for the Echora face detector."""

try:
    from detector_config import (
        DetectorConfigError,
        _boolean,
        _bounded_float,
        _positive_int,
        _text,
        _topic,
    )
except ImportError:  # pragma: no cover - package import on the development Mac
    from .detector_config import (
        DetectorConfigError,
        _boolean,
        _bounded_float,
        _positive_int,
        _text,
        _topic,
    )


PARAMETER_DEFAULTS = {
    "image_topic": "/camera/image_raw",
    "person_detections_topic": "/perception/person_detections",
    "face_detections_topic": "/perception/face_detections",
    "annotated_image_topic": "/perception/face_image",
    "status_topic": "/perception/face_status",
    "model_name": "yunet_2023mar",
    "model_path": "/home/animesh/echora/models/yunet_2023mar_fp16.engine",
    "model_input_width": 640,
    "model_input_height": 640,
    "confidence_threshold": 0.75,
    "nms_iou_threshold": 0.30,
    "max_inference_rate_hz": 10.0,
    "max_person_rois": 3,
    "person_roi_padding": 0.08,
    "person_roi_height_fraction": 0.72,
    "minimum_person_roi_pixels": 24,
    "image_cache_size": 24,
    "max_frame_age_sec": 0.5,
    "frame_timeout_sec": 2.0,
    "person_timeout_sec": 2.0,
    "publish_annotated_image": True,
    "annotate_only_when_subscribed": True,
    "max_annotated_rate_hz": 3.0,
    "enable_timing_diagnostics": True,
    "status_interval_sec": 5.0,
    "max_faces": 20,
    "latency_window": 120,
}


class FaceDetectorConfig(object):
    def __init__(self, **overrides):
        unknown = sorted(set(overrides) - set(PARAMETER_DEFAULTS))
        if unknown:
            raise DetectorConfigError(
                "unknown face detector parameters: {0}".format(", ".join(unknown))
            )
        values = dict(PARAMETER_DEFAULTS)
        values.update(overrides)

        for name in (
            "image_topic",
            "person_detections_topic",
            "face_detections_topic",
            "annotated_image_topic",
            "status_topic",
        ):
            setattr(self, name, _topic(name, values[name]))
        outputs = (
            self.face_detections_topic,
            self.annotated_image_topic,
            self.status_topic,
        )
        if len(set(outputs)) != len(outputs):
            raise DetectorConfigError("face detector output topics must be unique")
        if self.image_topic == self.person_detections_topic:
            raise DetectorConfigError("image and person-detection topics must differ")
        if self.image_topic in outputs or self.person_detections_topic in outputs:
            raise DetectorConfigError("input topics must not also be output topics")

        self.model_name = _text("model_name", values["model_name"])
        self.model_path = _text("model_path", values["model_path"])
        self.model_input_width = _positive_int(
            "model_input_width", values["model_input_width"], 4096
        )
        self.model_input_height = _positive_int(
            "model_input_height", values["model_input_height"], 4096
        )
        for name in ("model_input_width", "model_input_height"):
            if getattr(self, name) % 32:
                raise DetectorConfigError("{0} must be divisible by 32".format(name))

        self.confidence_threshold = _bounded_float(
            "confidence_threshold", values["confidence_threshold"], 0.0, 1.0
        )
        self.nms_iou_threshold = _bounded_float(
            "nms_iou_threshold", values["nms_iou_threshold"], 0.0, 1.0
        )
        self.max_inference_rate_hz = _bounded_float(
            "max_inference_rate_hz", values["max_inference_rate_hz"], 0.0, 120.0
        )
        self.max_person_rois = _positive_int(
            "max_person_rois", values["max_person_rois"], 20
        )
        self.person_roi_padding = _bounded_float(
            "person_roi_padding", values["person_roi_padding"], 0.0, 0.5, True
        )
        self.person_roi_height_fraction = _bounded_float(
            "person_roi_height_fraction",
            values["person_roi_height_fraction"],
            0.25,
            1.0,
            True,
        )
        self.minimum_person_roi_pixels = _positive_int(
            "minimum_person_roi_pixels", values["minimum_person_roi_pixels"], 1000
        )
        self.image_cache_size = _positive_int(
            "image_cache_size", values["image_cache_size"], 1000
        )
        self.max_frame_age_sec = _bounded_float(
            "max_frame_age_sec", values["max_frame_age_sec"], 0.0, 60.0
        )
        self.frame_timeout_sec = _bounded_float(
            "frame_timeout_sec", values["frame_timeout_sec"], 0.0, 600.0
        )
        self.person_timeout_sec = _bounded_float(
            "person_timeout_sec", values["person_timeout_sec"], 0.0, 600.0
        )
        if self.frame_timeout_sec < self.max_frame_age_sec:
            raise DetectorConfigError(
                "frame_timeout_sec must be at least max_frame_age_sec"
            )
        self.publish_annotated_image = _boolean(
            "publish_annotated_image", values["publish_annotated_image"]
        )
        self.annotate_only_when_subscribed = _boolean(
            "annotate_only_when_subscribed", values["annotate_only_when_subscribed"]
        )
        self.max_annotated_rate_hz = _bounded_float(
            "max_annotated_rate_hz", values["max_annotated_rate_hz"], 0.0, 60.0
        )
        self.enable_timing_diagnostics = _boolean(
            "enable_timing_diagnostics", values["enable_timing_diagnostics"]
        )
        self.status_interval_sec = _bounded_float(
            "status_interval_sec", values["status_interval_sec"], 0.0, 3600.0
        )
        self.max_faces = _positive_int("max_faces", values["max_faces"], 1000)
        self.latency_window = _positive_int(
            "latency_window", values["latency_window"], 10000, minimum=2
        )

    @classmethod
    def from_mapping(cls, mapping):
        return cls(**dict(mapping))

    def inference_period_sec(self):
        return 1.0 / self.max_inference_rate_hz

    def model_input_shape(self):
        return (self.model_input_height, self.model_input_width)
