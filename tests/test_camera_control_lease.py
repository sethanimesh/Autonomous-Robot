import os
import tempfile
import unittest
import subprocess
import sys

from robot.jetson.mission.camera_control_lease import CameraControlBusy
from robot.jetson.mission.camera_control_lease import CameraControlLease
from robot.jetson.mission.camera_control_lease import manual_camera_control_available
from robot.jetson.mission.camera_control_lease import subprocess_lease_options


class CameraControlLeaseTests(unittest.TestCase):
    def test_mission_and_calibrator_share_lock_without_releasing_parent_ownership(self):
        child = '''
import os, subprocess, sys
from robot.jetson.mission.camera_control_lease import CameraControlLease, camera_control_lease, subprocess_lease_options
path = sys.argv[1]
if sys.argv[2] == "mission":
    with camera_control_lease(exclusive=True, path=path):
        subprocess.run([sys.executable, "-c", sys.argv[3], path, "calibrator"], check=True,
                       timeout=5, **subprocess_lease_options())
else:
    with camera_control_lease(exclusive=True, path=path):
        pass
'''
        with CameraControlLease(exclusive=True, path=self.path) as parent:
            subprocess.run([sys.executable, "-c", child, self.path, "mission", child],
                           check=True, timeout=8, **subprocess_lease_options(parent))
            self.assertFalse(manual_camera_control_available(self.path))
        self.assertTrue(manual_camera_control_available(self.path))

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.directory.name, "camera.lock")

    def tearDown(self):
        self.directory.cleanup()

    def test_exclusive_calibration_lease_blocks_manual_control(self):
        with CameraControlLease(exclusive=True, path=self.path):
            self.assertFalse(manual_camera_control_available(self.path))
            with self.assertRaises(CameraControlBusy):
                CameraControlLease(exclusive=False, path=self.path).acquire()
        self.assertTrue(manual_camera_control_available(self.path))

    def test_multiple_manual_checks_can_share_the_lock(self):
        with CameraControlLease(exclusive=False, path=self.path):
            self.assertTrue(manual_camera_control_available(self.path))


if __name__ == "__main__":
    unittest.main()
