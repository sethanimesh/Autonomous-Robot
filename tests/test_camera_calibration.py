import unittest

from robot.jetson.camera.camera_calibration import CalibrationError
from robot.jetson.camera.camera_calibration import calibration_from_mapping
from robot.jetson.camera.camera_calibration import resolve_calibration
from robot.jetson.camera.camera_calibration import uncalibrated


def calibration_document(width=640, height=480):
    return {
        "image_width": width,
        "image_height": height,
        "camera_name": "echora_usb",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": [0.1, -0.2, 0.0, 0.0, 0.0],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        },
    }


class UncalibratedTests(unittest.TestCase):
    def test_uncalibrated_zeroes_every_matrix(self):
        calibration = uncalibrated(640, 480)

        self.assertFalse(calibration.is_calibrated)
        self.assertEqual([0.0] * 9, calibration.camera_matrix)
        self.assertEqual([0.0] * 9, calibration.rectification_matrix)
        self.assertEqual([0.0] * 12, calibration.projection_matrix)
        self.assertEqual([], calibration.distortion_coefficients)
        self.assertEqual("", calibration.distortion_model)

    def test_uncalibrated_keeps_the_real_image_size(self):
        calibration = uncalibrated(1280, 720)

        self.assertEqual(1280, calibration.image_width)
        self.assertEqual(720, calibration.image_height)


class CalibrationParsingTests(unittest.TestCase):
    def test_a_complete_document_loads(self):
        calibration = calibration_from_mapping(calibration_document())

        self.assertTrue(calibration.is_calibrated)
        self.assertEqual(500.0, calibration.camera_matrix[0])
        self.assertEqual("plumb_bob", calibration.distortion_model)
        self.assertEqual(5, len(calibration.distortion_coefficients))
        self.assertEqual(12, len(calibration.projection_matrix))
        self.assertEqual("echora_usb", calibration.camera_name)

    def test_resolution_comparison(self):
        calibration = calibration_from_mapping(calibration_document())

        self.assertTrue(calibration.matches_resolution(640, 480))
        self.assertFalse(calibration.matches_resolution(1280, 720))

    def test_non_mapping_is_rejected(self):
        with self.assertRaises(CalibrationError):
            calibration_from_mapping("image_width: 640")

    def test_missing_image_size_is_rejected(self):
        document = calibration_document()
        del document["image_width"]

        with self.assertRaises(CalibrationError):
            calibration_from_mapping(document)

    def test_zeroed_camera_matrix_is_rejected_as_not_a_calibration(self):
        document = calibration_document()
        document["camera_matrix"]["data"] = [0.0] * 9

        with self.assertRaises(CalibrationError):
            calibration_from_mapping(document)

    def test_wrong_matrix_size_is_rejected(self):
        document = calibration_document()
        document["projection_matrix"]["data"] = [1.0, 2.0, 3.0]

        with self.assertRaises(CalibrationError):
            calibration_from_mapping(document)

    def test_missing_matrix_is_rejected(self):
        document = calibration_document()
        del document["rectification_matrix"]

        with self.assertRaises(CalibrationError):
            calibration_from_mapping(document)

    def test_missing_distortion_model_is_rejected(self):
        document = calibration_document()
        document["distortion_model"] = ""

        with self.assertRaises(CalibrationError):
            calibration_from_mapping(document)

    def test_non_numeric_matrix_data_is_rejected(self):
        document = calibration_document()
        document["camera_matrix"]["data"][4] = "five hundred"

        with self.assertRaises(CalibrationError):
            calibration_from_mapping(document)


class ResolveCalibrationTests(unittest.TestCase):
    def test_no_file_configured_reports_uncalibrated(self):
        calibration = resolve_calibration("", 640, 480)

        self.assertFalse(calibration.is_calibrated)
        self.assertEqual("no_calibration_file_configured", calibration.describe())

    def test_missing_file_reports_the_path(self):
        calibration = resolve_calibration(
            "/home/animesh/echora/missing.yaml", 640, 480, exists=lambda path: False
        )

        self.assertFalse(calibration.is_calibrated)
        self.assertIn("calibration_file_not_found", calibration.describe())
        self.assertIn("missing.yaml", calibration.describe())

    def test_unreadable_file_falls_back_without_raising(self):
        def broken_loader(path):
            raise CalibrationError("camera_matrix is missing or is not a mapping")

        calibration = resolve_calibration(
            "/tmp/bad.yaml", 640, 480, loader=broken_loader, exists=lambda path: True
        )

        self.assertFalse(calibration.is_calibrated)
        self.assertIn("calibration_file_invalid", calibration.describe())
        self.assertEqual([0.0] * 9, calibration.camera_matrix)

    def test_resolution_mismatch_is_refused_rather_than_reused(self):
        loaded = calibration_from_mapping(calibration_document(1280, 720))

        calibration = resolve_calibration(
            "/tmp/cal.yaml", 640, 480, loader=lambda path: loaded, exists=lambda path: True
        )

        self.assertFalse(calibration.is_calibrated)
        self.assertIn("calibration_resolution_mismatch", calibration.describe())
        self.assertIn("1280x720", calibration.describe())
        self.assertEqual(640, calibration.image_width)

    def test_matching_calibration_is_used_and_names_its_source(self):
        loaded = calibration_from_mapping(calibration_document())

        calibration = resolve_calibration(
            "/tmp/cal.yaml", 640, 480, loader=lambda path: loaded, exists=lambda path: True
        )

        self.assertTrue(calibration.is_calibrated)
        self.assertEqual("/tmp/cal.yaml", calibration.source)
        self.assertIn("calibration_file_loaded", calibration.describe())


if __name__ == "__main__":
    unittest.main()
