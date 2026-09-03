import unittest

from robot.jetson.perception.detector_config import DetectorConfigError
from robot.jetson.perception.face_config import FaceDetectorConfig


class FaceDetectorConfigTests(unittest.TestCase):
    def test_defaults_are_safe_and_gpu_model_is_fixed_size(self):
        config = FaceDetectorConfig()

        self.assertEqual("/perception/person_detections", config.person_detections_topic)
        self.assertEqual((640, 640), config.model_input_shape())
        self.assertEqual(3, config.max_person_rois)
        self.assertAlmostEqual(0.1, config.inference_period_sec())
        self.assertAlmostEqual(3.0, config.max_annotated_rate_hz)

    def test_unknown_setting_is_rejected(self):
        with self.assertRaises(DetectorConfigError):
            FaceDetectorConfig(typo=True)

    def test_inputs_cannot_alias_outputs(self):
        with self.assertRaises(DetectorConfigError):
            FaceDetectorConfig(face_detections_topic="/camera/image_raw")

    def test_duplicate_outputs_are_rejected(self):
        with self.assertRaises(DetectorConfigError):
            FaceDetectorConfig(status_topic="/perception/face_detections")

    def test_model_dimensions_must_be_divisible_by_32(self):
        with self.assertRaises(DetectorConfigError):
            FaceDetectorConfig(model_input_width=641)

    def test_frame_timeout_cannot_precede_maximum_age(self):
        with self.assertRaises(DetectorConfigError):
            FaceDetectorConfig(max_frame_age_sec=1.0, frame_timeout_sec=0.5)

    def test_person_crop_fraction_is_bounded(self):
        with self.assertRaises(DetectorConfigError):
            FaceDetectorConfig(person_roi_height_fraction=0.2)
