#!/usr/bin/env python3
"""Run the selected robotics contract checks without hardware or services."""

import argparse
import os
from pathlib import Path
import platform
import sys
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
MODULES = (
    "tests.test_recognition_core",
    "tests.test_person_continuity",
    "tests.test_target_gate",
    "tests.test_cloud_view_calibration",
    "tests.test_detour_route_binding",
    "tests.test_navigation_reasoning",
    "tests.test_ev3_client_serialization",
    "tests.test_head_command_worker",
    "tests.test_ev3_server",
    "tests.test_single_room_mission",
    "tests.test_mission_control",
)


class CheckResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.passed = 0

    def addSuccess(self, test):
        self.passed += 1
        super().addSuccess(test)


def prohibit_live_operations(event, args):
    """Catch accidental live I/O if a selected test changes in the future."""
    if event in {
        "socket.connect", "socket.bind", "socket.getaddrinfo",
        "subprocess.Popen", "os.system", "os.exec", "os.posix_spawn",
    }:
        raise RuntimeError("Hardware-free checks prohibit live operation: " + event)
    if event == "open" and isinstance(args[0], (str, bytes, os.PathLike)):
        path = Path(os.path.realpath(os.fsdecode(args[0])))
        if path == Path("/sys") or Path("/sys") in path.parents:
            raise RuntimeError("Hardware-free checks prohibit system-device access: " + str(path))
        if path == Path("/dev") or Path("/dev") in path.parents:
            raise RuntimeError("Hardware-free checks prohibit device access: " + str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verbose", action="store_true", help="print each test name")
    parser.add_argument("--list", action="store_true", help="list selected modules without running tests")
    args = parser.parse_args()
    if sys.version_info < (3, 11):
        parser.error("Python 3.11 or newer is required; no third-party dependencies are needed")
    if args.list:
        print("\n".join(MODULES))
        return 0

    # Several existing replay tests read production source relative to the root.
    os.chdir(ROOT)
    sys.path.insert(0, str(ROOT))
    sys.dont_write_bytecode = True
    sys.addaudithook(prohibit_live_operations)
    print("Python {} on {}; {} selected modules".format(
        platform.python_version(), platform.system(), len(MODULES)), flush=True)
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromNames(MODULES)
    if loader.errors:
        print("Required checks could not load. Missing modules or dependencies are failures.",
              file=sys.stderr)
        print("\n".join(loader.errors), file=sys.stderr)
        return 1
    if suite.countTestCases() == 0:
        print("No checks loaded; refusing to report success.", file=sys.stderr)
        return 1

    started = time.perf_counter()
    result = unittest.TextTestRunner(
        verbosity=2 if args.verbose else 1, resultclass=CheckResult).run(suite)
    elapsed = time.perf_counter() - started
    successful = result.wasSuccessful() and not result.skipped and not result.expectedFailures
    print("Result: {} tests, {} passed, {} skipped, {} failures, {} errors; {:.3f} s".format(
        result.testsRun, result.passed, len(result.skipped), len(result.failures),
        len(result.errors), elapsed), flush=True)
    if result.skipped:
        print("Skipped required checks count as failure; restore their dependencies or fixtures.",
              file=sys.stderr)
    if result.expectedFailures:
        print("Expected failures still represent unmet required checks and count as failure.",
              file=sys.stderr)
    return 0 if successful else 1


if __name__ == "__main__":
    sys.exit(main())
