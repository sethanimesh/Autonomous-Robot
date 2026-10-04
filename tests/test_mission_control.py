import json
import os
import tempfile
import threading
import unittest

from robot.jetson.perception.mission_control import FindMissionController
from robot.jetson.perception.mission_control import MissionControlError
from robot.jetson.perception.mission_control import build_find_command
from robot.jetson.perception.mission_control import mission_result


class FakeLease(object):
    def __init__(self):
        self.acquired = False
        self.released = False

    def acquire(self):
        self.acquired = True
        return self

    def release(self):
        self.released = True


class WaitingProcess(object):
    pid = 1234

    def __init__(self):
        self.returncode = None
        self.done = threading.Event()

    def communicate(self):
        self.done.wait(2.0)
        return ("mission output", None)

    def poll(self):
        return self.returncode


class MissionControlTests(unittest.TestCase):
    def test_detected_person_is_distinct_from_confirmed_identity(self):
        result = mission_result({'outcome': 'person_found_unidentified'})
        self.assertEqual('person_seen', result['state'])
        self.assertIn('identity is not confirmed', result['message'])

    def test_unknown_or_contradictory_pose_enters_calibration_never_skips_it(self):
        for head in (
            {"available": True, "homed": False, "calibrated": False},
            {"available": True, "homed": True, "calibrated": True, "position": 185},
        ):
            FindMissionController.validate_preflight(True, "Me", True, head)
        command = build_find_command("mission.py", "/tmp/report.json", "http://route")
        self.assertNotIn("--skip-camera-calibration", command)

    def test_command_calibrates_before_cable_neutral_mission(self):
        command = build_find_command("mission.py", "/tmp/report.json", "http://route")

        self.assertEqual("/usr/bin/python3", command[0])
        self.assertIn("--execute", command)
        self.assertNotIn("--skip-camera-calibration", command)
        self.assertEqual("right", command[command.index("--first-direction") + 1])
        self.assertIn("--cable-zero-confirmed", command)
        self.assertEqual("0", command[command.index("--initial-cable-heading-degrees") + 1])

    def test_preflight_requires_tether_camera_and_calibrated_head(self):
        good_head = {
            "available": True,
            "homed": True,
            "calibrated": True,
            "moving": False,
            "homing": False,
        }
        invalid = (
            (False, "Me", True, good_head),
            (True, None, True, good_head),
            (True, "Me", False, good_head),
            (True, "Me", True, dict(good_head, moving=True)),
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(MissionControlError):
                FindMissionController.validate_preflight(*values)

    def test_result_reports_found_standoff(self):
        result = mission_result(
            {
                "outcome": "target_found_at_standoff",
                "cable_heading_degrees": 24.5,
                "target_observation": {"confirmed": True},
            }
        )

        self.assertEqual("found", result["state"])
        self.assertEqual(24.5, result["cable_heading_degrees"])

    def test_missing_child_report_is_failure(self):
        result = mission_result({}, returncode=0)

        self.assertEqual("failed", result["state"])
        self.assertIn("without a valid status report", result["message"])

    def test_start_is_nonblocking_and_stop_terminates_process_group(self):
        with tempfile.TemporaryDirectory() as directory:
            report = os.path.join(directory, "mission.json")
            process = WaitingProcess()
            lease = FakeLease()
            calls = []

            def factory(command, **options):
                calls.append((command, options))
                return process

            controller = FindMissionController(
                "mission.py",
                report,
                "http://route",
                process_factory=factory,
                lease_factory=lambda: lease,
                group_killer=lambda pgid, sig: calls.append((pgid, sig)),
                pgid_getter=lambda pid: pid + 1,
            )
            head = {
                "available": True,
                "homed": True,
                "calibrated": True,
                "moving": False,
                "homing": False,
            }

            started = controller.start(True, "Me", True, head,
                                       initial_cable_heading_degrees=-80.34)
            stopped = controller.stop()

            self.assertEqual("running", started["state"])
            self.assertEqual("stopping", stopped["state"])
            self.assertTrue(lease.acquired)
            self.assertTrue(calls[0][1]["start_new_session"])
            command = calls[0][0]
            self.assertEqual('-80.34', command[command.index('--initial-cable-heading-degrees') + 1])
            self.assertIn('--preserve-initial-heading', command)
            self.assertEqual(-80.34, started['cable_heading_degrees'])
            self.assertEqual('0', controller.command[controller.command.index('--initial-cable-heading-degrees') + 1])
            self.assertEqual(1235, calls[1][0])
            process.returncode = -15
            process.done.set()

    def test_invalid_recovered_heading_does_not_launch(self):
        controller = FindMissionController('mission.py', '/tmp/report.json', 'http://route',
                                          process_factory=lambda *_a, **_k: self.fail('must not launch'))
        for heading in (float('nan'), float('inf'), 91., -91., '24', True, None):
            with self.subTest(heading=heading), self.assertRaises(MissionControlError):
                controller.start(True, 'Me', True, {'available': True},
                                 initial_cable_heading_degrees=heading)

    def test_completed_process_loads_report_and_releases_lease(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = os.path.join(directory, "mission.json")
            process = WaitingProcess()
            lease = FakeLease()
            finished = threading.Event()
            controller = FindMissionController(
                "mission.py",
                report_path,
                "http://route",
                process_factory=lambda *_args, **_kwargs: process,
                lease_factory=lambda: lease,
                on_finished=finished.set,
            )
            head = {
                "available": True,
                "homed": True,
                "calibrated": True,
                "moving": False,
                "homing": False,
            }
            controller.start(True, "Me", True, head)
            with open(report_path, "w", encoding="utf-8") as handle:
                json.dump(
                    {
                        "outcome": "target_found_at_standoff",
                        "cable_heading_degrees": 12.0,
                    },
                    handle,
                )
            process.returncode = 0
            process.done.set()

            self.assertTrue(finished.wait(1.0))
            self.assertEqual("found", controller.status()["state"])
            self.assertTrue(lease.released)


if __name__ == "__main__":
    unittest.main()
