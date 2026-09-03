import unittest

from robot.jetson.perception.detector_config import DetectorConfigError
from robot.jetson.perception.recognition_config import RecognitionConfig


SHA = "a" * 64


class RecognitionConfigTests(unittest.TestCase):
    def test_valid_configuration_is_conservative(self):
        config = RecognitionConfig(model_sha256=SHA)
        self.assertEqual("antelopev2_glintr100", config.model_name)
        self.assertEqual(3, config.confirmation_required)
        self.assertEqual(5, config.confirmation_window)
        self.assertAlmostEqual(0.45, config.match_threshold)

    def test_model_digest_is_required(self):
        with self.assertRaises(DetectorConfigError):
            RecognitionConfig()
        with self.assertRaises(DetectorConfigError):
            RecognitionConfig(model_sha256="not-a-digest")

    def test_confirmation_must_fit_window(self):
        with self.assertRaises(DetectorConfigError):
            RecognitionConfig(model_sha256=SHA, confirmation_required=6)

    def test_topics_cannot_alias(self):
        with self.assertRaises(DetectorConfigError):
            RecognitionConfig(model_sha256=SHA, status_topic="/camera/image_raw")
