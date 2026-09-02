#!/usr/bin/env python3
"""Configuration for the Echora ROS 2 person detector.

Deliberately free of ROS, OpenCV, numpy, and TensorRT imports so every
validation rule can be tested on a development machine.
"""

PARAMETER_DEFAULTS = {
    "image_topic": "/camera/image_raw",
    "detections_topic": "/perception/person_detections",
    "annotated_image_topic": "/perception/person_image",
    "status_topic": "/perception/status",
    "model_name": "yolox_tiny",
    "model_path": "/home/animesh/echora/models/yolox_tiny_fp16.engine",
    "onnx_path": "/home/animesh/echora/models/yolox_tiny.onnx",
    "model_input_width": 416,
    "model_input_height": 416,
    "confidence_threshold": 0.45,
    "nms_iou_threshold": 0.45,
    "max_inference_rate_hz": 15.0,
    "inference_device": "auto",
    "publish_annotated_image": True,
    "annotate_only_when_subscribed": True,
    "enable_timing_diagnostics": True,
    "person_class_id": 0,
    "person_class_label": "person",
    "max_frame_age_sec": 0.5,
    "frame_timeout_sec": 2.0,
    "status_interval_sec": 5.0,
    "max_detections": 50,
    "latency_window": 120,
}

VALID_DEVICES = ("auto", "tensorrt", "cpu")

MAX_MODEL_DIMENSION = 4096
MAX_INFERENCE_RATE_HZ = 240.0
MAX_CLASS_ID = 10000


class DetectorConfigError(ValueError):
    """Raised when a parameter would produce an unusable detector."""


def _topic(name, value):
    if not isinstance(value, str) or not value.strip():
        raise DetectorConfigError("{0} must be a non-empty string".format(name))
    topic = value.strip()
    if " " in topic:
        raise DetectorConfigError(
            "{0} must not contain spaces, got {1!r}".format(name, topic)
        )
    return topic


def _text(name, value, allow_empty=False):
    if not isinstance(value, str):
        raise DetectorConfigError("{0} must be a string, got {1!r}".format(name, value))
    text = value.strip()
    if not text and not allow_empty:
        raise DetectorConfigError("{0} must not be empty".format(name))
    return text


def _positive_int(name, value, maximum, minimum=1):
    if isinstance(value, bool):
        raise DetectorConfigError("{0} must be an integer, got a bool".format(name))
    try:
        number = int(value)
    except (TypeError, ValueError):
        raise DetectorConfigError("{0} must be an integer, got {1!r}".format(name, value))
    if number < minimum:
        raise DetectorConfigError(
            "{0} must be at least {1}, got {2}".format(name, minimum, number)
        )
    if number > maximum:
        raise DetectorConfigError(
            "{0} must be {1} or less, got {2}".format(name, maximum, number)
        )
    return number


def _bounded_float(name, value, minimum, maximum, allow_minimum=False):
    if isinstance(value, bool):
        raise DetectorConfigError("{0} must be a number, got a bool".format(name))
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise DetectorConfigError("{0} must be a number, got {1!r}".format(name, value))
    if number != number:
        raise DetectorConfigError("{0} must be a real number, got NaN".format(name))
    if number in (float("inf"), float("-inf")):
        raise DetectorConfigError("{0} must be finite, got {1}".format(name, number))
    if number < minimum or (number == minimum and not allow_minimum):
        comparison = "at least" if allow_minimum else "greater than"
        raise DetectorConfigError(
            "{0} must be {1} {2}, got {3}".format(name, comparison, minimum, number)
        )
    if number > maximum:
        raise DetectorConfigError(
            "{0} must be {1} or less, got {2}".format(name, maximum, number)
        )
    return number


def _boolean(name, value):
    if not isinstance(value, bool):
        raise DetectorConfigError("{0} must be a bool, got {1!r}".format(name, value))
    return value


class DetectorConfig(object):
    """Validated settings for one person-detection node."""

    def __init__(self, **overrides):
        unknown = sorted(set(overrides) - set(PARAMETER_DEFAULTS))
        if unknown:
            raise DetectorConfigError(
                "unknown detector parameters: {0}".format(", ".join(unknown))
            )
        values = dict(PARAMETER_DEFAULTS)
        values.update(overrides)

        self.image_topic = _topic("image_topic", values["image_topic"])
        self.detections_topic = _topic("detections_topic", values["detections_topic"])
        self.annotated_image_topic = _topic(
            "annotated_image_topic", values["annotated_image_topic"]
        )
        self.status_topic = _topic("status_topic", values["status_topic"])

        published = [
            self.detections_topic,
            self.annotated_image_topic,
            self.status_topic,
        ]
        if len(set(published)) != len(published):
            raise DetectorConfigError("output topics must all differ from each other")
        if self.image_topic in published:
            raise DetectorConfigError(
                "image_topic {0} must not also be an output topic".format(self.image_topic)
            )

        self.model_name = _text("model_name", values["model_name"])
        self.model_path = _text("model_path", values["model_path"])
        self.onnx_path = _text("onnx_path", values["onnx_path"], allow_empty=True)

        self.model_input_width = _positive_int(
            "model_input_width", values["model_input_width"], MAX_MODEL_DIMENSION
        )
        self.model_input_height = _positive_int(
            "model_input_height", values["model_input_height"], MAX_MODEL_DIMENSION
        )
        for name, size in (
            ("model_input_width", self.model_input_width),
            ("model_input_height", self.model_input_height),
        ):
            if size % 32 != 0:
                raise DetectorConfigError(
                    "{0} must be a multiple of 32 for a YOLOX stride grid, got "
                    "{1}".format(name, size)
                )

        self.confidence_threshold = _bounded_float(
            "confidence_threshold", values["confidence_threshold"], 0.0, 1.0
        )
        self.nms_iou_threshold = _bounded_float(
            "nms_iou_threshold", values["nms_iou_threshold"], 0.0, 1.0
        )
        self.max_inference_rate_hz = _bounded_float(
            "max_inference_rate_hz", values["max_inference_rate_hz"], 0.0, MAX_INFERENCE_RATE_HZ
        )

        device = _text("inference_device", values["inference_device"]).lower()
        if device not in VALID_DEVICES:
            raise DetectorConfigError(
                "inference_device must be one of {0}, got {1!r}".format(
                    ", ".join(VALID_DEVICES), device
                )
            )
        self.inference_device = device

        self.publish_annotated_image = _boolean(
            "publish_annotated_image", values["publish_annotated_image"]
        )
        self.annotate_only_when_subscribed = _boolean(
            "annotate_only_when_subscribed", values["annotate_only_when_subscribed"]
        )
        self.enable_timing_diagnostics = _boolean(
            "enable_timing_diagnostics", values["enable_timing_diagnostics"]
        )

        self.person_class_id = _positive_int(
            "person_class_id", values["person_class_id"], MAX_CLASS_ID, minimum=0
        )
        self.person_class_label = _text("person_class_label", values["person_class_label"])

        self.max_frame_age_sec = _bounded_float(
            "max_frame_age_sec", values["max_frame_age_sec"], 0.0, 60.0
        )
        self.frame_timeout_sec = _bounded_float(
            "frame_timeout_sec", values["frame_timeout_sec"], 0.0, 600.0
        )
        if self.frame_timeout_sec < self.max_frame_age_sec:
            raise DetectorConfigError(
                "frame_timeout_sec ({0}) must be at least max_frame_age_sec ({1}); "
                "otherwise frames are reported missing while they are still "
                "arriving".format(self.frame_timeout_sec, self.max_frame_age_sec)
            )
        self.status_interval_sec = _bounded_float(
            "status_interval_sec", values["status_interval_sec"], 0.0, 3600.0
        )
        self.max_detections = _positive_int(
            "max_detections", values["max_detections"], 1000
        )
        self.latency_window = _positive_int("latency_window", values["latency_window"], 10000, minimum=2)

    @classmethod
    def from_mapping(cls, mapping):
        return cls(**dict(mapping))

    def inference_period_sec(self):
        """Timer period that caps how often inference may run."""
        return 1.0 / self.max_inference_rate_hz

    def model_input_shape(self):
        """Model input as (height, width), matching row-major image order."""
        return (self.model_input_height, self.model_input_width)

    def as_dict(self):
        return {name: getattr(self, name) for name in PARAMETER_DEFAULTS}
