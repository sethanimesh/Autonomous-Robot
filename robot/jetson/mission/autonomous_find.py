#!/usr/bin/env python3
"""Run the complete bounded single-room find-and-approach mission."""

import argparse
import json
import math
import os
import subprocess
import sys
import time

try:
    from robot.jetson.mission.recovery_policy import recovery_kind
except ImportError:
    from recovery_policy import recovery_kind


try:
    from robot.jetson.mission.person_approach import approach_decision
except ImportError:
    from person_approach import approach_decision

class MissionPaused(RuntimeError):
    pass


def write_progress(path, report):
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    os.replace(temporary, path)

try:
    from robot.jetson.mission.camera_control_lease import camera_control_lease, subprocess_lease_options
except ImportError:
    from camera_control_lease import camera_control_lease, subprocess_lease_options

try:
    from robot.jetson.navigation.cable_guard import validate_measured_cable_heading
except ImportError:
    from cable_guard import validate_measured_cable_heading


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


def target_is_at_standoff(scan_report, minimum_height=0.28):
    """Use the confirmed face size; body cropping is not a distance signal."""
    observation = scan_report.get('target_observation') or {}
    if observation.get('clothing_approach_enabled'):
        return approach_decision(observation)['action'] in ('arrived','arrived_estimate')
    height = target_height(scan_report)
    if height is None:
        return False
    return height >= minimum_height


def record_arrival(report, observation):
    """Keep the estimated stopping outcome distinct all the way to the UI."""
    estimated=(observation.get('clothing_approach_enabled')
               and approach_decision(observation)['action']=='arrived_estimate')
    report.update(outcome='target_found_at_estimated_standoff' if estimated else 'target_found_at_standoff',
                  target_observation=observation)
    report['events'].append('estimated_standoff_reached' if estimated else 'safe_standoff_confirmed')
    if estimated:
        report['message']='Stopped near the person using estimated camera distance; front gap is unverified.'


def heading_after_relative_scan(current_heading, scan_report):
    if "final_cable_heading_degrees" in scan_report:
        return float(scan_report["final_cable_heading_degrees"])
    return float(current_heading) + float(
        scan_report.get(
            "target_relative_heading_degrees",
            scan_report.get("target_heading_degrees", 0.0),
        )
    )


def heading_after_detour(current_heading, detour_report):
    if "final_cable_heading_degrees" in detour_report:
        return float(detour_report["final_cable_heading_degrees"])
    return float(current_heading) + float(detour_report.get("turned_degrees", 0.0))


def run_child(command, report_path, timeout_seconds):
    try:
        os.remove(report_path)
    except FileNotFoundError:
        pass
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_seconds,
            check=False,
            **subprocess_lease_options(),
        )
    except subprocess.TimeoutExpired:
        return dict(outcome='failure', cable_heading_known=False,
                    error='Mission worker timed out; its final position is unavailable')
    try:
        with open(report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
    except (OSError, ValueError) as exc:
        return dict(outcome='failure', cable_heading_known=False,
                    error='Mission worker produced no valid final position: {0}'.format(exc))
    if not isinstance(report, dict):
        return dict(outcome='failure', cable_heading_known=False,
                    error='Mission worker produced an invalid report')
    if completed.returncode not in (0, 2):
        report.update(outcome='failure', cable_heading_known=False,
                      error='Mission worker exited {0}: {1}'.format(
                          completed.returncode, completed.stdout[-500:].strip()))
    return report


def calibration_command(args, report_path):
    command = [
        sys.executable,
        args.calibration_script,
        "--execute",
        "--route-url",
        args.route_url,
        "--report",
        report_path,
    ]
    if getattr(args, 'visual_camera_setup', False):
        command.extend(['--visual-setup', '--setup-advice-url', args.setup_advice_url])
    else:
        command.insert(3, '--auto-setup')
    return command


def run(args):
    report = {
        "outcome": "running",
        "state": "running",
        "started_at_unix": time.time(),
        "events": [],
        "steps": [],
        "cable_heading_degrees": float(args.initial_cable_heading_degrees),
    }
    if not args.execute:
        report["outcome"] = "dry_run_success"
        report['state'] = report['outcome']
        report["events"].append("no motors commanded")
        report["finished_at_unix"] = time.time()
        return report
    if not args.camera_only and not args.cable_zero_confirmed:
        report['outcome'] = 'failure'
        report['state'] = report['outcome']
        report["error"] = (
            "chassis motion locked: align the robot with the marked cable-neutral "
            "pose and pass --cable-zero-confirmed"
        )
        report["finished_at_unix"] = time.time()
        return report

    log_dir = os.path.dirname(args.report) or "."
    os.makedirs(log_dir, exist_ok=True)
    cable_heading = float(args.initial_cable_heading_degrees)
    scan_number = 0
    move_number = 0
    preferred_head = None
    approach_step = .05
    range_lower_view = False

    def beginning_step(message):
        report.update(state='running', message=message)
        write_progress(args.report, report)

    def scan_once(local=False, continue_past=False):
        nonlocal range_lower_view
        nonlocal scan_number, cable_heading, preferred_head
        beginning_step('Looking for the target person.')
        scan_number += 1
        path = os.path.join(log_dir, "mission-scan-{0:02d}.json".format(scan_number))
        command = [
            sys.executable,
            args.scan_script,
            "--execute",
            "--first-direction",
            args.first_direction,
            "--report",
            path,
            "--dwell-seconds",
            str(args.dwell_seconds),
            "--initial-cable-heading-degrees",
            str(cable_heading),
            "--cable-limit-degrees",
            str(args.cable_limit_degrees),
            "--cable-margin-degrees",
            str(args.cable_margin_degrees),
        ]
        if local:
            command.extend(["--step-degrees", "15", "--sweep-limit-degrees", "45"])
            command.append("--preserve-initial-heading")
            if preferred_head:
                command.extend(['--preferred-head-position', str(preferred_head[0]),
                                '--preferred-head-reference', preferred_head[1]])
        else:
            command.extend(["--step-degrees", "30", "--sweep-limit-degrees", "90"])
            if args.one_step_test or args.preserve_initial_heading:
                command.append('--preserve-initial-heading')
        if continue_past:
            command.append('--continue-past-unidentified')
        if args.try_up and not range_lower_view:
            command.extend(["--try-up", "--search-up"])
        if args.camera_only:
            command.append("--vertical-only")
        else:
            command.extend(["--cable-zero-confirmed", "--center-target"])
        command.append('--fast-search')
        scan_origin_heading = cable_heading
        if range_lower_view:
            command.append('--range-lower-view')
            range_lower_view=False
        if args.profile_id:
            command.extend(['--profile-id',args.profile_id])
        if args.profile_revision:
            command.extend(['--profile-revision',args.profile_revision])
        child = run_child(command, path, args.scan_timeout_seconds)
        if (child.get('outcome') in ('target_found', 'person_found_unidentified')
                and type(child.get('target_head_position')) is int
                and isinstance(child.get('target_head_reference'), str)):
            preferred_head = (child['target_head_position'], child['target_head_reference'])
        if child.get('cable_heading_known') is False:
            report['steps'].append({'kind': 'scan', 'report': child})
            report['cable_heading_degrees'] = None
            raise MissionPaused(child.get('error', 'Turn interrupted') + '; restore cable neutral before another scan')
        if "final_cable_heading_degrees" in child:
            cable_heading = float(child["final_cable_heading_degrees"])
        elif child.get("outcome") == "target_found":
            cable_heading = heading_after_relative_scan(scan_origin_heading, child)
        report["steps"].append({"kind": "scan", "report": child})
        report["cable_heading_degrees"] = round(cable_heading, 2)
        write_progress(args.report, report)
        return child

    def move_once(kind):
        nonlocal move_number, cable_heading
        beginning_step('Approaching the person in a route-checked short step.'
                       if kind == 'target_approach' else
                       'Checking the floor and taking a short step.')
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
                "--initial-cable-heading-degrees",
                str(cable_heading),
                "--cable-limit-degrees",
                str(args.cable_limit_degrees),
                "--cable-margin-degrees",
                str(args.cable_margin_degrees),
                "--cable-zero-confirmed",
                "--maximum-step-distance",
                str(approach_step if kind == "target_approach" else .05),
            ],
            path,
            args.child_timeout_seconds,
        )
        report["steps"].append({"kind": kind, "report": child})
        if child.get('cable_heading_known') is False:
            report['cable_heading_degrees'] = None
            raise MissionPaused(child.get('error', 'Movement interrupted') + '; restore cable neutral before another run')
        if 'final_cable_heading_degrees' in child:
            cable_heading = float(child['final_cable_heading_degrees'])
        elif child.get("outcome") == "success":
            cable_heading = heading_after_detour(cable_heading, child)
        report["cable_heading_degrees"] = round(cable_heading, 2)
        write_progress(args.report, report)
        return child

    def recovering(kind, child, attempt):
        report['state'] = 'recovering'
        report['message'] = {'identity': 'Person seen; trying another view.',
                             'route': 'Checking another route.',
                             'feedback': 'Waiting for fresh feedback, then continuing.'}[kind]
        report['events'].append('recovering_' + kind)
        report.setdefault('recoveries', []).append(dict(kind=kind, attempt=attempt,
                                                       reason=child.get('error') or child.get('outcome')))
        write_progress(args.report, report)
        time.sleep(args.retry_delay_seconds)

    def scan(local=False):
        candidate_attempts = 0
        failures = 0
        continue_past = False
        while True:
            child = scan_once(local=local, continue_past=continue_past)
            outcome = child.get('outcome')
            if outcome == 'person_found_unidentified':
                if candidate_attempts < args.candidate_retries:
                    candidate_attempts += 1
                    recovering('identity', child, candidate_attempts)
                    local = True
                    continue
                if not continue_past:
                    # Retain the candidate in the report and look for other
                    # family members after giving this view several chances.
                    continue_past, local = True, False
                    continue
                return child
            if outcome == 'failure' and recovery_kind(child) and failures < args.recovery_attempts:
                failures += 1
                recovering(recovery_kind(child), child, failures)
                local = True
                continue
            return child

    def move(kind):
        for attempt in range(args.recovery_attempts + 1):
            child = move_once(kind)
            recovery = recovery_kind(child)
            if recovery and child.get('drive_started') and child.get('outcome') != 'success':
                # A partially completed drive needs a new target observation,
                # not another full-distance command at the old target bearing.
                report['events'].append('interrupted_drive_requires_reacquisition')
                return dict(child, outcome='interrupted')
            if child.get('outcome') == 'success' or not recovery:
                return child
            if attempt == args.recovery_attempts:
                raise MissionPaused(child.get('error') or 'Route is still unavailable')
            recovering(recovery, child, attempt+1)
        return child

    try:
        if not args.skip_camera_calibration:
            beginning_step('Preparing the camera view.')
            calibration_path = os.path.join(log_dir, "mission-head-calibration.json")
            calibration = run_child(
                calibration_command(args, calibration_path),
                calibration_path,
                args.calibration_timeout_seconds,
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
            target_scan = scan(local=False)
            if target_scan.get("outcome") == "target_found":
                report["events"].append("target_found")
                break
            if target_scan.get('outcome') == 'person_found_unidentified':
                report['outcome'] = 'person_found_unidentified'
                report['person_observation'] = target_scan.get('person_observation')
                report['events'].append('stopped_with_person_candidate')
                return report
            valid_scan_outcomes = (
                "scan_complete_no_target",
                "vertical_scan_complete_no_target",
            )
            if target_scan.get("outcome") not in valid_scan_outcomes:
                raise MissionPaused(target_scan.get("error", "search scan is temporarily unavailable"))
            if args.camera_only:
                report["outcome"] = "target_not_found"
                return report
            if search_index >= args.maximum_search_moves:
                report["outcome"] = "target_not_found"
                return report
            search_move = move("search_reposition")
            if search_move.get("outcome") not in ('success', 'interrupted'):
                raise MissionPaused(search_move.get("error", "search route is unavailable"))
            report["events"].append("search_reposition_complete")

        for approach_index in range(args.maximum_approach_steps + 1):
            if target_is_at_standoff(target_scan, args.found_height_fraction):
                record_arrival(report,target_scan["target_observation"])
                return report
            if args.camera_only:
                report["outcome"] = "target_found_not_at_standoff"
                report["target_observation"] = target_scan["target_observation"]
                return report
            if approach_index >= args.maximum_approach_steps:
                report['outcome'] = 'target_found_not_at_standoff'
                report['target_observation'] = target_scan['target_observation']
                report['events'].append('approach_step_limit_reached')
                return report
            if (target_scan.get('target_observation') or {}).get('clothing_approach_enabled'):
                decision = approach_decision(target_scan['target_observation'])
                # Reacquiring identity must not spend the floor-view attempts
                # needed immediately after a face becomes visible.
                view_attempts = {'reacquire': 2, 'inspect': 2, 'look_lower': 2}
                inspecting_lower = False
                while True:
                    if decision['action'] in ('approach', 'arrived', 'arrived_estimate', 'close'): break
                    action = decision['action']
                    if view_attempts.get(action, 0) <= 0: break
                    view_attempts[action] -= 1
                    if decision['action'] == 'look_lower':
                        inspecting_lower = True
                    if inspecting_lower:
                        preferred_head = None
                        range_lower_view = True
                    # The local scan starts at the useful lower view and the
                    # persistent observer keeps the person's appearance memory.
                    target_scan = scan(local=True)
                    decision = approach_decision(target_scan.get('target_observation') or {})
                if decision['action'] in ('arrived','arrived_estimate'):
                    record_arrival(report,target_scan['target_observation'])
                    return report
                if decision['action'] != 'approach':
                    report.update(outcome='target_found_not_at_standoff',
                        target_observation=target_scan.get('target_observation'),
                        message=decision.get('reason','Person located; approach incomplete'))
                    return report
                approach_step = decision['distance_m']
            approach = move("target_approach")
            if approach.get("outcome") not in ('success', 'interrupted'):
                raise MissionPaused(approach.get("error", "target approach is temporarily unavailable"))
            report["events"].append('approach_step_interrupted' if approach.get('outcome') == 'interrupted'
                                    else 'approach_step_complete')
            target_scan = scan(local=True)
            for retry in range(args.reacquire_retries):
                if target_scan.get('outcome') == 'target_found':
                    break
                recovering('identity', target_scan, retry+1)
                target_scan = scan(local=(retry == 0))
            if target_scan.get("outcome") != "target_found":
                raise MissionPaused('Target temporarily lost; search progress saved')
            report["events"].append("target_reacquired")
            if args.one_step_test:
                report['outcome'] = 'step_complete_target_reacquired'
                report['target_observation'] = target_scan['target_observation']
                return report
    except MissionPaused as exc:
        report['outcome'] = 'paused'
        report['error'] = str(exc)
    except Exception as exc:
        report['outcome'] = 'failure'
        report["error"] = str(exc)
    finally:
        report['state'] = report['outcome']
        report["finished_at_unix"] = time.time()
        write_progress(args.report, report)
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile-id')
    parser.add_argument('--profile-revision')
    parser.add_argument("--execute", action="store_true")
    parser.add_argument('--recovery-attempts', type=int, default=3)
    parser.add_argument('--candidate-retries', type=int, default=1)
    parser.add_argument('--reacquire-retries', type=int, default=2)
    parser.add_argument('--retry-delay-seconds', type=float, default=.5)
    parser.add_argument("--one-step-test", action="store_true",
                        help="stop after one route-checked movement and target reacquisition")
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
    parser.add_argument('--visual-camera-setup', action='store_true',
                        help='opt in to Groq useful-view selection inside the approved camera range')
    parser.add_argument('--setup-advice-url', default='http://127.0.0.1:18091/camera-setup')
    parser.add_argument(
        "--approach-script", default="/home/animesh/echora/closed_loop_detour.py"
    )
    parser.add_argument("--route-url", default="http://127.0.0.1:18091/route")
    parser.add_argument("--initial-cable-heading-degrees", type=float, default=0.0)
    parser.add_argument("--cable-limit-degrees", type=float, default=120.0)
    parser.add_argument("--cable-margin-degrees", type=float, default=5.0)
    parser.add_argument(
        "--cable-zero-confirmed",
        action="store_true",
        help="confirm the chassis is aligned with the marked tether-neutral pose",
    )
    parser.add_argument("--maximum-search-moves", type=int, default=2)
    parser.add_argument('--preserve-initial-heading', action='store_true',
                        help='begin searching at the current measured cable heading')
    parser.add_argument("--maximum-approach-steps", type=int, default=8)
    parser.add_argument("--found-height-fraction", type=float, default=0.28)
    parser.add_argument("--dwell-seconds", type=float, default=0.8)
    parser.add_argument(
        "--first-direction",
        choices=("right", "left"),
        default="right",
        help="direction used first during each bounded visual scan",
    )
    parser.add_argument("--try-up", action="store_true", default=True)
    parser.add_argument("--scan-timeout-seconds", type=float, default=900.0,
                        help="allow small camera steps across the bounded room scan")
    parser.add_argument("--child-timeout-seconds", type=float, default=180.0)
    parser.add_argument("--calibration-timeout-seconds", type=float, default=600.0,
                        help="allow the bounded camera sweep, recovery and Groq capacity waits")
    parser.add_argument(
        "--report", default="/home/animesh/echora/logs/latest_find_mission.json"
    )
    args = parser.parse_args(argv)
    if args.visual_camera_setup and args.skip_camera_calibration:
        parser.error('--visual-camera-setup cannot be combined with --skip-camera-calibration')
    for name in ('recovery_attempts', 'candidate_retries', 'reacquire_retries'):
        if not 0 <= getattr(args, name) <= 10:
            parser.error(name + ' must be between zero and ten')
    if not math.isfinite(args.retry_delay_seconds) or not 0 <= args.retry_delay_seconds <= 10:
        parser.error('retry delay must be between zero and ten seconds')
    if args.one_step_test:
        args.maximum_search_moves = 0
        args.maximum_approach_steps = 1
    if args.maximum_search_moves < 0 or args.maximum_approach_steps < 0:
        parser.error("movement counts must be non-negative")
    if not math.isfinite(args.scan_timeout_seconds) or args.scan_timeout_seconds <= 0:
        parser.error('scan timeout must be positive and finite')
    if not math.isfinite(args.calibration_timeout_seconds) or args.calibration_timeout_seconds <= 0:
        parser.error('calibration timeout must be positive and finite')
    try:
        validate_measured_cable_heading(
            args.initial_cable_heading_degrees,
            args.cable_limit_degrees,
            args.cable_margin_degrees,
        )
    except Exception as exc:
        parser.error(str(exc))
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.execute:
        with camera_control_lease(exclusive=True):
            report = run(args)
    else:
        report = run(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.execute:
        os.makedirs(os.path.dirname(args.report) or '.', exist_ok=True)
        write_progress(args.report, report)
    return 0 if report["outcome"] in (
        "dry_run_success",
        "target_found_at_standoff",
        "target_found_at_estimated_standoff",
        "target_not_found",
        "target_lost",
        "target_found_not_at_standoff",
        "step_complete_target_reacquired",
        "person_found_unidentified",
        "paused",
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
