import unittest

from robot.jetson.perception.detector_config import PARAMETER_DEFAULTS
from robot.jetson.perception.detector_config import DetectorConfig
from robot.jetson.perception.detector_config import DetectorConfigError


class DetectorConfigDefaultsTests(unittest.TestCase):
    def test_defaults_are_valid(self):
        config = DetectorConfig.from_mapping({})

        self.assertEqual("/camera/image_raw", config.image_topic)
        self.assertEqual("/perception/person_detections", config.detections_topic)
        self.assertEqual("/perception/person_image", config.annotated_image_topic)
        self.assertEqual("/perception/status", config.status_topic)
        self.assertEqual(0, config.person_class_id)

    def test_every_default_round_trips(self):
        config = DetectorConfig.from_mapping(dict(PARAMETER_DEFAULTS))

        self.assertEqual(PARAMETER_DEFAULTS, config.as_dict())

    def test_inference_period_matches_max_rate(self):
        config = DetectorConfig.from_mapping({"max_inference_rate_hz": 20.0})

        self.assertAlmostEqual(0.05, config.inference_period_sec())

    def test_model_input_shape_is_height_then_width(self):
        config = DetectorConfig.from_mapping(
            {"model_input_width": 640, "model_input_height": 384}
        )

        self.assertEqual((384, 640), config.model_input_shape())


class DetectorConfigValidationTests(unittest.TestCase):
    def assert_rejected(self, **overrides):
        with self.assertRaises(DetectorConfigError):
            DetectorConfig.from_mapping(overrides)

    def test_unknown_parameter_is_rejected(self):
        self.assert_rejected(enable_face_recognition=True)

    def test_empty_topic_is_rejected(self):
        self.assert_rejected(image_topic="   ")

    def test_topic_with_a_space_is_rejected(self):
        self.assert_rejected(detections_topic="/perception/person detections")

    def test_duplicate_output_topics_are_rejected(self):
        self.assert_rejected(
            detections_topic="/perception/same", status_topic="/perception/same"
        )

    def test_input_topic_may_not_also_be_an_output(self):
        self.assert_rejected(annotated_image_topic="/camera/image_raw")

    def test_empty_model_path_is_rejected(self):
        self.assert_rejected(model_path="")

    def test_model_size_must_be_a_multiple_of_32(self):
        self.assert_rejected(model_input_width=500)

    def test_model_size_must_be_positive(self):
        self.assert_rejected(model_input_height=0)

    def test_absurd_model_size_is_rejected(self):
        self.assert_rejected(model_input_width=99999)

    def test_unknown_device_is_rejected(self):
        self.assert_rejected(inference_device="tpu")

    def test_known_devices_are_accepted(self):
        for device in ("auto", "tensorrt", "cpu"):
            self.assertEqual(
                device, DetectorConfig.from_mapping({"inference_device": device}).inference_device
            )

    def test_device_is_case_insensitive(self):
        self.assertEqual(
            "tensorrt", DetectorConfig.from_mapping({"inference_device": "TensorRT"}).inference_device
        )

    def test_negative_class_id_is_rejected(self):
        self.assert_rejected(person_class_id=-1)

    def test_class_id_zero_is_accepted(self):
        self.assertEqual(0, DetectorConfig.from_mapping({"person_class_id": 0}).person_class_id)

    def test_booleans_must_be_booleans(self):
        self.assert_rejected(publish_annotated_image="yes")

    def test_integer_parameters_reject_bools(self):
        self.assert_rejected(person_class_id=True)

    def test_frame_timeout_must_not_be_shorter_than_max_frame_age(self):
        self.assert_rejected(max_frame_age_sec=2.0, frame_timeout_sec=0.5)

    def test_frame_timeout_equal_to_max_age_is_accepted(self):
        config = DetectorConfig.from_mapping(
            {"max_frame_age_sec": 1.0, "frame_timeout_sec": 1.0}
        )

        self.assertEqual(1.0, config.frame_timeout_sec)

    def test_latency_window_must_allow_a_percentile(self):
        self.assert_rejected(latency_window=1)


class ThresholdValidationTests(unittest.TestCase):
    """Confidence and IoU are probabilities and are validated as such."""

    def assert_rejected(self, **overrides):
        with self.assertRaises(DetectorConfigError):
            DetectorConfig.from_mapping(overrides)

    def test_confidence_above_one_is_rejected(self):
        self.assert_rejected(confidence_threshold=1.5)

    def test_negative_confidence_is_rejected(self):
        self.assert_rejected(confidence_threshold=-0.1)

    def test_zero_confidence_is_rejected(self):
        # A zero threshold would publish every anchor the model produces.
        self.assert_rejected(confidence_threshold=0.0)

    def test_confidence_of_one_is_accepted(self):
        self.assertEqual(
            1.0, DetectorConfig.from_mapping({"confidence_threshold": 1.0}).confidence_threshold
        )

    def test_non_numeric_confidence_is_rejected(self):
        self.assert_rejected(confidence_threshold="high")

    def test_nan_confidence_is_rejected(self):
        self.assert_rejected(confidence_threshold=float("nan"))

    def test_infinite_rate_is_rejected(self):
        self.assert_rejected(max_inference_rate_hz=float("inf"))

    def test_iou_above_one_is_rejected(self):
        self.assert_rejected(nms_iou_threshold=1.01)

    def test_negative_iou_is_rejected(self):
        self.assert_rejected(nms_iou_threshold=-0.5)

    def test_iou_of_one_is_accepted(self):
        self.assertEqual(
            1.0, DetectorConfig.from_mapping({"nms_iou_threshold": 1.0}).nms_iou_threshold
        )

    def test_zero_inference_rate_is_rejected(self):
        self.assert_rejected(max_inference_rate_hz=0.0)

    def test_absurd_inference_rate_is_rejected(self):
        self.assert_rejected(max_inference_rate_hz=10000.0)

    def test_negative_frame_age_is_rejected(self):
        self.assert_rejected(max_frame_age_sec=-1.0)
