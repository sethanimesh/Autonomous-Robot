#!/usr/bin/env python3
"""Background control for the single-room find-person browser action."""

import json
import math
import os
import signal
import subprocess
import threading
import time

try:
    from camera_control_lease import CameraControlBusy, CameraControlLease, subprocess_lease_options
except ImportError:  # pragma: no cover - package import on the development Mac
    from robot.jetson.mission.camera_control_lease import (
        CameraControlBusy,
        CameraControlLease,
        subprocess_lease_options,
    )


RUNNING_STATES = ("starting", "running", "stopping")
FOUND_OUTCOMES = ("target_found_at_standoff", "target_found_at_estimated_standoff", "target_found_not_at_standoff")


class MissionControlError(RuntimeError):
    """Raised when a browser mission request cannot be started safely."""


def build_find_command(
    mission_script,
    report_path,
    route_url,
    python_executable="/usr/bin/python3",
    cable_limit_degrees=90.0,
    cable_margin_degrees=10.0,
):
    """Build the fixed, cable-neutral single-room mission command."""

    return [
        str(python_executable),
        str(mission_script),
        "--execute",
        "--first-direction",
        "right",
        "--cable-zero-confirmed",
        "--initial-cable-heading-degrees",
        "0",
        "--cable-limit-degrees",
        str(float(cable_limit_degrees)),
        "--cable-margin-degrees",
        str(float(cable_margin_degrees)),
        "--route-url",
        str(route_url),
        "--report",
        str(report_path),
    ]


def mission_result(report, returncode=0, stop_requested=False):
    """Turn one child report into concise browser state."""

    report = report if isinstance(report, dict) else {}
    outcome = report.get("outcome")
    if stop_requested:
        state = "stopped"
        message = "Mission stopped. All motors were commanded to stop."
    elif outcome == 'paused':
        state = 'paused'
        message = str(report.get('error') or 'Waiting for the route or robot connection. Progress saved.')
    elif returncode != 0 or outcome == "failure":
        state = "failed"
        message = str(report.get("error") or "Find-person mission failed safely.")
    elif not outcome:
        state = "failed"
        message = "Find-person mission finished without a valid status report."
    elif outcome in FOUND_OUTCOMES:
        state = "found"
        if outcome == "target_found_at_standoff":
            message = "Target found. Robot stopped at a safe distance."
        elif outcome == 'target_found_at_estimated_standoff':
            message = 'Target found. Stopped nearby using an approximate camera distance; gap is unverified.'
        else:
            message = str(report.get('message') or 'Target found. Approach remains incomplete.')
    elif outcome == "target_not_found":
        state = "not_found"
        message = "Target was not found in the bounded room search."
    elif outcome == "target_lost":
        state = "target_lost"
        message = "Target was lost. Robot stopped safely."
    elif outcome == "person_found_unidentified":
        state = "person_seen"
        message = "Person found. Stopped with them in view; identity is not confirmed yet."
    elif outcome == "step_complete_target_reacquired":
        state = "found"
        message = "Short approach completed. Target identified again; robot stopped."
    else:
        state = "complete"
        message = "Find-person mission finished."
    return {
        "state": state,
        "message": message,
        "outcome": outcome,
        "cable_heading_degrees": report.get("cable_heading_degrees"),
        "target_observation": report.get("target_observation"),
        "error": report.get("error"),
    }


class FindMissionController(object):
    """Own one non-blocking mission process and its camera-control lease."""

    def __init__(
        self,
        mission_script,
        report_path,
        route_url,
        python_executable="/usr/bin/python3",
        cable_limit_degrees=90.0,
        cable_margin_degrees=10.0,
        process_factory=None,
        lease_factory=None,
        group_killer=None,
        pgid_getter=None,
        clock=None,
        on_finished=None,
    ):
        self.command = build_find_command(
            mission_script,
            report_path,
            route_url,
            python_executable,
            cable_limit_degrees,
            cable_margin_degrees,
        )
        self.report_path = os.path.abspath(report_path)
        self.process_factory = process_factory or subprocess.Popen
        self.lease_factory = lease_factory or (
            lambda: CameraControlLease(exclusive=True)
        )
        self.group_killer = group_killer or os.killpg
        self.pgid_getter = pgid_getter or os.getpgid
        self.clock = clock or time.time
        self.on_finished = on_finished
        self.lock = threading.RLock()
        self.process = None
        self.lease = None
        self.stop_requested = False
        self.state = "idle"
        self.message = "Ready to find the enrolled person."
        self.started_at = None
        self.finished_at = None
        self.outcome = None
        self.cable_heading_degrees = None
        self.target_observation = None
        self.profile_id = None
        self.profile_revision = None
        self.error = None
        self.output_tail = ""

    @staticmethod
    def validate_preflight(
        cable_zero_confirmed,
        target_label,
        camera_ready,
        camera_head,
    ):
        if cable_zero_confirmed is not True:
            raise MissionControlError(
                "Center the tether at its marked neutral position, keep it clear "
                "of both tracks, then tick the cable confirmation."
            )
        if not target_label:
            raise MissionControlError("Enroll a target person before starting.")
        if not camera_ready:
            raise MissionControlError("The live camera is not ready.")
        if not isinstance(camera_head, dict) or not camera_head.get("available"):
            raise MissionControlError("Camera-head status is unavailable.")
        # The mission validates this boot against saved operator endpoints, or
        # performs optional visual setup when no complete endpoints are saved.
        if camera_head.get("moving") or camera_head.get("homing"):
            raise MissionControlError("Wait for the camera head to stop first.")

    def _snapshot_locked(self):
        return {
            "state": self.state,
            "running": self.state in RUNNING_STATES,
            "message": self.message,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "outcome": self.outcome,
            "cable_heading_degrees": self.cable_heading_degrees,
            "target_observation": self.target_observation,
            "profile_id": self.profile_id,
            "profile_revision": self.profile_revision,
            "error": self.error,
        }

    def status(self):
        with self.lock:
            snapshot = self._snapshot_locked()
            if self.state == 'running':
                progress = self._load_report()
                if progress.get('state') in ('running', 'recovering'):
                    snapshot['message'] = progress.get('message', self.message)
                snapshot['recoveries'] = progress.get('recoveries', [])
            return snapshot

    def start(
        self,
        cable_zero_confirmed,
        target_label,
        camera_ready,
        camera_head,
        initial_cable_heading_degrees=0.0,
        profile_id=None,
        profile_revision=None,
    ):
        self.validate_preflight(
            cable_zero_confirmed,
            target_label,
            camera_ready,
            camera_head,
        )
        # A supervising caller can retain a heading recovered from unchanged
        # encoders. The browser's ordinary neutral start still defaults to zero.
        heading = initial_cable_heading_degrees
        limit = float(self.command[self.command.index('--cable-limit-degrees') + 1])
        if (isinstance(heading, bool) or not isinstance(heading, (int, float))
                or not math.isfinite(heading) or abs(heading) > limit):
            raise MissionControlError('Current cable heading is invalid or outside the saved range.')
        command = list(self.command)
        if profile_id:
            command.extend(['--profile-id',profile_id])
        if profile_revision:
            command.extend(['--profile-revision',profile_revision])
        command[command.index('--initial-cable-heading-degrees') + 1] = str(heading)
        if heading != 0:
            command.append('--preserve-initial-heading')
        with self.lock:
            if self.state in RUNNING_STATES:
                raise MissionControlError("A find-person mission is already running.")
            lease = self.lease_factory()
            try:
                lease.acquire()
            except CameraControlBusy:
                raise MissionControlError(
                    "Camera control is busy; wait for the current action to finish."
                )
            try:
                directory = os.path.dirname(self.report_path)
                if directory:
                    os.makedirs(directory, exist_ok=True)
                try:
                    os.remove(self.report_path)
                except FileNotFoundError:
                    pass
                process = self.process_factory(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                    **subprocess_lease_options(lease),
                )
            except Exception as exc:
                lease.release()
                raise MissionControlError(
                    "Could not start the find-person mission: {0}".format(exc)
                )
            self.process = process
            self.lease = lease
            self.stop_requested = False
            self.state = "running"
            self.message = "Calibrating the camera, then searching for {0}…".format(target_label)
            self.started_at = self.clock()
            self.finished_at = None
            self.outcome = None
            self.cable_heading_degrees = float(heading)
            self.target_observation = None
            self.profile_id = profile_id
            self.profile_revision = profile_revision
            self.error = None
            self.output_tail = ""
            watcher = threading.Thread(
                target=self._watch,
                args=(process,),
                name="echora-find-mission",
                daemon=True,
            )
            watcher.start()
            return self._snapshot_locked()

    def _load_report(self):
        try:
            with open(self.report_path, "r", encoding="utf-8") as handle:
                value = json.load(handle)
        except (OSError, ValueError):
            return {}
        return value if isinstance(value, dict) else {}

    def _watch(self, process):
        output, _ = process.communicate()
        report = self._load_report()
        with self.lock:
            if self.process is not process:
                return
            result = mission_result(
                report,
                process.returncode,
                stop_requested=self.stop_requested,
            )
            self.state = result["state"]
            self.message = result["message"]
            self.outcome = result["outcome"]
            self.cable_heading_degrees = result["cable_heading_degrees"]
            self.target_observation = result["target_observation"]
            self.error = result["error"]
            self.output_tail = (output or "")[-4000:]
            self.finished_at = self.clock()
            self.process = None
            lease, self.lease = self.lease, None
        if lease is not None:
            lease.release()
        if self.on_finished is not None:
            try:
                self.on_finished()
            except Exception:
                # Mission state and lease cleanup must survive a ROS shutdown
                # racing the best-effort final stop publication.
                pass

    def stop(self):
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                self.state = "stopped"
                self.message = "Stop command sent. Robot is stationary."
                self.finished_at = self.clock()
                return self._snapshot_locked()
            self.stop_requested = True
            self.state = "stopping"
            self.message = "Stopping robot…"
            pid = process.pid
        try:
            self.group_killer(self.pgid_getter(pid), signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass
        return self.status()

    def close(self):
        self.stop()
