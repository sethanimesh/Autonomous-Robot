from pathlib import Path
import re
import unittest


class EnrollmentConsolePageContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            Path(__file__).resolve().parents[1]
            / "robot"
            / "jetson"
            / "perception"
            / "enrollment_console.py"
        ).read_text(encoding="utf-8")
        cls.page_source = cls.source.split('PAGE = r"""', 1)[1].split(
            '""".encode("utf-8")', 1
        )[0]

    def test_manual_car_controls_are_present(self):
        for identifier in (
            'id="driveForward"',
            'id="driveBack"',
            'id="driveLeft"',
            'id="driveRight"',
            'id="driveStop"',
        ):
            with self.subTest(identifier=identifier):
                self.assertIn(identifier, self.page_source)

    def test_manual_controls_use_press_release_and_disable_endpoints(self):
        self.assertIn("onpointerdown=\"beginDrive", self.page_source)
        self.assertIn("onlostpointercapture=\"releaseDrive", self.page_source)
        self.assertIn("setPointerCapture", self.page_source)
        self.assertIn("window.addEventListener('pointerup'", self.page_source)
        self.assertIn("'/api/drive/command'", self.page_source)
        self.assertIn("'/api/drive/disable'", self.page_source)

    def test_manual_controls_keep_dead_man_keyboard_behavior(self):
        self.assertIn("onkeydown=\"beginDrive", self.page_source)
        self.assertIn("onkeyup=\"releaseDrive", self.page_source)
        self.assertIn("window.addEventListener('keyup'", self.page_source)

    def test_manual_drive_requires_separate_cable_confirmation(self):
        self.assertIn('id="driveCableClear"', self.page_source)
        self.assertIn("'/api/drive/enable'", self.page_source)

    def test_global_stop_remains_prominent_and_immediate(self):
        self.assertIn('id="stopMission"', self.page_source)
        self.assertIn('onclick="stopMission()"', self.page_source)
        self.assertIn(
            'aria-label="Stop all robot motion immediately"', self.page_source
        )
        self.assertIn("'/api/mission/stop'", self.page_source)

    def test_find_mission_stays_locked_while_ev3_is_offline(self):
        self.assertIn(
            "!!s.target_label&&s.camera_ready&&s.robot_ready&&h.available",
            self.page_source,
        )
        self.assertIn(
            "missionMissing.push('EV3 connection')", self.page_source
        )

    def test_camera_controls_keep_semantic_bounded_steps(self):
        controls = (
            ('id="tiltUp"', "jogCamera('up',5)"),
            ('id="tiltDown"', "jogCamera('down',5)"),
            ('id="tiltUpCoarse"', "jogCamera('up',15)"),
            ('id="tiltDownCoarse"', "jogCamera('down',15)"),
        )
        for identifier, action in controls:
            with self.subTest(identifier=identifier):
                self.assertIn(identifier, self.page_source)
                self.assertIn(action, self.page_source)
        self.assertIn("'/api/camera/jog'", self.page_source)

    def test_mission_and_enrollment_controls_are_preserved(self):
        for identifier in (
            'id="cableClear"',
            'id="findPerson"',
            'id="label"',
            'id="retention"',
            'id="consent"',
            'id="files"',
            'id="start"',
            'id="upload"',
            'id="cancel"',
            'id="finish"',
            'id="deleteTarget"',
        ):
            with self.subTest(identifier=identifier):
                self.assertIn(identifier, self.page_source)
        for endpoint in (
            "'/api/mission/start'",
            "'/api/enrollment/start'",
            "'/api/enrollment/upload'",
            "'/api/enrollment/cancel'",
            "'/api/enrollment/finish'",
            "'/api/enrollment/delete'",
        ):
            with self.subTest(endpoint=endpoint):
                self.assertIn(endpoint, self.page_source)

    def test_readiness_and_accessibility_landmarks_are_present(self):
        for identifier in (
            'id="cameraHealth"',
            'id="robotHealth"',
            'id="odomHealth"',
            'id="headHealth"',
            'id="systemSummary"',
            'id="enrollmentProgress"',
        ):
            with self.subTest(identifier=identifier):
                self.assertIn(identifier, self.page_source)
        self.assertIn('aria-live="polite"', self.page_source)
        self.assertIn(':focus-visible', self.page_source)

    def test_page_element_ids_are_unique(self):
        identifiers = re.findall(r'id="([^"]+)"', self.page_source)
        duplicates = sorted(
            identifier
            for identifier in set(identifiers)
            if identifiers.count(identifier) > 1
        )
        self.assertEqual([], duplicates)


if __name__ == "__main__":
    unittest.main()
