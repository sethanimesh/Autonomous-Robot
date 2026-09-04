#!/usr/bin/env python3
"""Run the complete bounded single-room find-and-approach mission."""

import argparse
import json
import math
import os
import subprocess
import sys
import time


def target_height(scan_report):
    observation = scan_report.get("target_observation")
    if not isinstance(observation, dict) or not observation.get("confirmed", False):
        return None
    try:
        height = float(observation["box_height_fraction"])
    except (KeyError, TypeError, ValueError):
        return None
    return height if math.isfinite(height) and height >= 0.0 else None


def body_height(scan_report):
    observations = scan_report.get("body_guided_tilts", [])
    if not observations:
        return None
    try:
        height = float(observations[-1]["body"]["height_fraction"])
    except (KeyError, TypeError, ValueError):
        return None
    return height if math.isfinite(height) and height >= 0.0 else None


def target_is_at_standoff(
    scan_report, minimum_height=0.15, minimum_body_height=0.70
):
    height = target_height(scan_report)
    if height is None:
        return False
    close_body = body_height(scan_report)
    return height >= minimum_height or (
        close_body is not None and close_body >= minimum_body_height
    )


def heading_after_relative_scan(current_heading, scan_report):
    return float(current_heading) + float(
        scan_report.get("target_heading_degrees", 0.0)
    )


def heading_after_detour(current_heading, detour_report):
    # Detour reports ROS yaw (positive left); cable headings use image-space
    # convention (positive right), so the sign is inverted.
    return float(current_heading) - float(detour_report.get("turned_degrees", 0.0))


def run_child(command, report_path, timeout_seconds):
    completed = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout_seconds,
        check=False,
    )
    try:
        with open(report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
    except (OSError, ValueError) as exc:
        raise RuntimeError("mission child produced no valid report: {0}".format(exc))
    if completed.returncode not in (0, 2):
        raise RuntimeError(
            "mission child exited {0}: {1}".format(
                completed.returncode, completed.stdout[-500:].strip()
            )
        )
    return report


def calibration_command(args, report_path):
    return [
        sys.executable,
        args.calibration_script,
        "--execute",
        "--route-url",
        args.route_url,
        "--report",
        report_path,
    ]


def run(args):
    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "events": [],
        "steps": [],
        "cable_heading_degrees": float(args.initial_cable_heading_degrees),
    }
    if not args.execute:
        report["outcome"] = "dry_run_success"
        report["events"].append("no motors commanded")
        report["finished_at_unix"] = time.time()
        return report

    log_dir = os.path.dirname(args.report) or "."
    os.makedirs(log_dir, exist_ok=True)
    cable_heading = float(args.initial_cable_heading_degrees)
    scan_number = 0
    move_number = 0

    def scan(local=False, unwind=False):
        nonlocal scan_number, cable_heading
        scan_number += 1
        path = os.path.join(log_dir, "mission-scan-{0:02d}.json".format(scan_number))
        command = [
            sys.executable,
            args.scan_script,
            "--execute",
            "--first-direction",
            "left",
            "--report",
            path,
            "--dwell-seconds",
            str(args.dwell_seconds),
        ]
        if local:
            command.extend(["--step-degrees", "15", "--sweep-limit-degrees", "45"])
        else:
            command.extend(["--step-degrees", "30", "--sweep-limit-degrees", "180"])
            if unwind and abs(cable_heading) >= 1.0:
                command.extend(
                    ["--initial-cable-heading-degrees", str(int(round(cable_heading)))]
                )
                cable_heading = 0.0
        if args.try_up:
            command.extend(["--try-up", "--search-up"])
        if args.camera_only:
            command.append("--vertical-only")
        scan_origin_heading = cable_heading
        child = run_child(command, path, args.child_timeout_seconds)
        if child.get("outcome") == "target_found":
            cable_heading = heading_after_relative_scan(scan_origin_heading, child)
        elif child.get("observed_headings"):
            cable_heading = scan_origin_heading + float(
                child["observed_headings"][-1]
            )
        report["steps"].append({"kind": "scan", "report": child})
        report["cable_heading_degrees"] = round(cable_heading, 2)
        return child

    def move(kind):
        nonlocal move_number, cable_heading
        move_number += 1
        path = os.path.join(log_dir, "mission-move-{0:02d}.json".format(move_number))
        child = run_child(
            [
                sys.executable,
                args.approach_script,
                "--route-url",
                args.route_url,
                "--execute",
                "--report",
                path,
            ],
            path,
            args.child_timeout_seconds,
        )
        if child.get("outcome") == "success":
            cable_heading = heading_after_detour(cable_heading, child)
        report["steps"].append({"kind": kind, "report": child})
        report["cable_heading_degrees"] = round(cable_heading, 2)
        return child

    try:
        if not args.skip_camera_calibration:
            calibration_path = os.path.join(log_dir, "mission-head-calibration.json")
            calibration = run_child(
                calibration_command(args, calibration_path),
                calibration_path,
                args.child_timeout_seconds,
            )
            report["steps"].append(
                {"kind": "camera_head_calibration", "report": calibration}
            )
            if calibration.get("outcome") != "calibrated":
                raise RuntimeError(
                    calibration.get("error", "camera-head calibration failed")
                )
            report["events"].append("camera_head_calibrated")

        target_scan = None
        for search_index in range(args.maximum_search_moves + 1):
            target_scan = scan(local=False, unwind=True)
            if target_scan.get("outcome") == "target_found":
                report["events"].append("target_found")
                break
            valid_scan_outcomes = (
                "scan_complete_no_target",
                "vertical_scan_complete_no_target",
            )
            if target_scan.get("outcome") not in valid_scan_outcomes:
                raise RuntimeError(target_scan.get("error", "search scan failed"))
            if args.camera_only:
                report["outcome"] = "target_not_found"
                return report
            if search_index >= args.maximum_search_moves:
                report["outcome"] = "target_not_found"
                return report
            search_move = move("search_reposition")
            if search_move.get("outcome") != "success":
                raise RuntimeError(search_move.get("error", "search reposition failed"))
            report["events"].append("search_reposition_complete")

        for approach_index in range(args.maximum_approach_steps + 1):
            if target_is_at_standoff(target_scan, args.found_height_fraction):
                report["outcome"] = "target_found_at_standoff"
                report["target_observation"] = target_scan["target_observation"]
                report["events"].append("safe_standoff_confirmed")
                return report
            if args.camera_only:
                report["outcome"] = "target_found_not_at_standoff"
                report["target_observation"] = target_scan["target_observation"]
                return report
            if approach_index >= args.maximum_approach_steps:
                raise RuntimeError("maximum approach steps reached")
            approach = move("target_approach")
            if approach.get("outcome") != "success":
                raise RuntimeError(approach.get("error", "target approach failed"))
            report["events"].append("approach_step_complete")
            target_scan = scan(local=True)
            if target_scan.get("outcome") != "target_found":
                report["outcome"] = "target_lost"
                return report
            report["events"].append("target_reacquired")
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        report["finished_at_unix"] = time.time()
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--camera-only",
        action="store_true",
        help="forbid all chassis scans and movement",
    )
    parser.add_argument(
        "--scan-script", default="/home/animesh/echora/bounded_target_scan.py"
    )
    parser.add_argument(
        "--calibration-script",
        default="/home/animesh/echora/camera_head_calibration.py",
    )
    parser.add_argument(
        "--skip-camera-calibration",
        action="store_true",
        help="reuse the current boot's already verified camera calibration",
    )
    parser.add_argument(
        "--approach-script", default="/home/animesh/echora/closed_loop_detour.py"
    )
    parser.add_argument("--route-url", default="http://192.168.1.26:8091/route")
    parser.add_argument("--initial-cable-heading-degrees", type=float, default=0.0)
    parser.add_argument("--maximum-search-moves", type=int, default=2)
    parser.add_argument("--maximum-approach-steps", type=int, default=8)
    parser.add_argument("--found-height-fraction", type=float, default=0.15)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--try-up", action="store_true", default=True)
    parser.add_argument("--child-timeout-seconds", type=float, default=180.0)
    parser.add_argument(
        "--report", default="/home/animesh/echora/logs/latest_find_mission.json"
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = run(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.execute:
        os.makedirs(os.path.dirname(args.report), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
    return 0 if report["outcome"] in (
        "dry_run_success",
        "target_found_at_standoff",
        "target_not_found",
        "target_lost",
        "target_found_not_at_standoff",
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
