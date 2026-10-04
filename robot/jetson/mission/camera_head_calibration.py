#!/usr/bin/env python3
"""Reference the useful floor view without seeking a mechanical stop."""

import argparse
import json
import math
import os
import re
import time
from urllib.request import Request, urlopen

try:
    from robot.jetson.mission.camera_motion_feedback import (
        CameraMotionFeedbackError,
        fingerprint_quality,
        frame_fingerprint,
        verify_camera_step,
    )
    from robot.jetson.mission.camera_control_lease import camera_control_lease
    from robot.jetson.mission.camera_visual_setup import (
        SetupStateChanged, approved_setup_range, request_setup_view, run_visual_setup, setup_tolerance,
        discovery_setup_range, run_visual_discovery)
except ImportError:
    from camera_motion_feedback import (
        CameraMotionFeedbackError,
        fingerprint_quality,
        frame_fingerprint,
        verify_camera_step,
    )
    from camera_control_lease import camera_control_lease
    from camera_visual_setup import (
        SetupStateChanged, approved_setup_range, request_setup_view, run_visual_setup, setup_tolerance,
        discovery_setup_range, run_visual_discovery)


class CameraCalibrationError(RuntimeError):
    pass


class CameraLimitReached(CameraCalibrationError):
    pass


def usable_motion_feedback(report, allow_textureless=False):
    """Image change is diagnostic; fresh usable views can be classified later."""
    if not isinstance(report,dict):
        return False
    for key in ('after','settled'):
        quality=report.get(key,{})
        if not (quality.get('usable') is True or
                (allow_textureless and 12 <= quality.get('median_luma',0) <= 245)):
            return False
    return True


def next_upward_position(current, lower_limit=-54, step_degrees=15):
    """Step toward smaller encoder counts within the current reference bounds."""

    current = int(current)
    lower_limit = int(lower_limit)
    step_degrees = abs(int(step_degrees))
    if step_degrees <= 0:
        raise ValueError("camera calibration step must be positive")
    if current <= lower_limit:
        return None
    return max(lower_limit, current - step_degrees)


# Compatibility for old diagnostic imports. Live image comparison on
# 2026-09-05 proved that the current linkage's negative sweep raises the view.
next_lowering_position = next_upward_position


def track_motion_active(status):
    if not isinstance(status, dict):
        return True
    if "track_motion_active" in status:
        return bool(status["track_motion_active"])
    return bool(status.get("motion_active", True)) and not bool(
        status.get("tool_motion_active", False)
    )


def stable_reference_tail(positions, tolerance=2):
    """Return the final consecutive encoder samples within one stable band."""

    tolerance = abs(int(tolerance))
    stable = []
    for value in positions:
        value = int(value)
        candidate = stable + [value]
        if stable and max(candidate) - min(candidate) > tolerance:
            stable = [value]
        else:
            stable = candidate
    return stable


def center_floor_fraction(route):
    if not isinstance(route, dict):
        return None
    values = []
    for item in route.get("evidence", []):
        try:
            heading = float(item["heading_degrees"])
            fraction = float(item["floor_fraction"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(heading) and math.isfinite(fraction) and abs(heading) <= 10:
            values.append((abs(heading), max(0.0, min(1.0, fraction))))
    return None if not values else min(values)[1]


def cloud_reference_verified(sample, role):
    return role in ('floor', 'room', 'overhead', 'raised') and sample.get('cloud_view_role') == role and bool(sample.get('cloud_advice_id'))


def select_cloud_frames(samples):
    usable=[s for s in samples if s.get('frame_sha256')]
    if len(usable)<3:
        raise CameraCalibrationError('Cloud calibration needs three captured camera views')
    first,last=usable[0],usable[-1]
    middle=min(usable[1:-1],key=lambda s:abs(s['position']-(first['position']+last['position'])/2))
    if not first['position']>middle['position']>last['position']:
        raise CameraCalibrationError('Cloud calibration requires an ordered upward sweep')
    return [first,middle,last]


def select_revisited_cloud_frames(rechecks):
    latest = {sample['role']: sample for sample in rechecks}
    if not all(role in latest for role in ('down','forward','up')):
        raise CameraCalibrationError('Capture all three functional camera views before checking them')
    selected = [latest[role] for role in ('down','forward','up')]
    if (not all(sample.get('frame_sha256') and sample.get('view_usable') for sample in selected)
            or not selected[0]['position'] > selected[1]['position'] > selected[2]['position']):
        raise CameraCalibrationError('Revisited scene evidence is incomplete or camera poses are out of order')
    return selected


def bound_cloud_observations(selected, result, reference):
    if (result.get('ok') is not True or result.get('provider') not in ('gemini','groq')
            or not result.get('advice_id')
            or result.get('reference_id')!=reference
            or result.get('frame_sha256')!=[s['frame_sha256'] for s in selected]
            or result.get('positions')!=[s['position'] for s in selected]):
        raise CameraCalibrationError('Cloud vision could not verify a consistent floor, room and overhead sequence')
    observations=result.get('observations',[])
    if not isinstance(observations,list) or len(observations)!=3:
        raise CameraCalibrationError('Cloud vision did not describe all three views')
    if any(not isinstance(obs,dict) for obs in observations):
        raise CameraCalibrationError('Cloud scene observations are invalid')
    return observations


def apply_cloud_advice(selected, result, reference):
    observations=bound_cloud_observations(selected,result,reference)
    floor,room,upper=observations
    if not (result.get('upper_verified') is True
            and floor.get('view')=='floor_room' and floor.get('floor_visible')=='yes'
            and room.get('view') in ('room','floor_room')
            and upper.get('view')=='ceiling' and upper.get('ceiling_visible')=='yes'
            and upper.get('floor_visible')=='no'):
        raise CameraCalibrationError('Cloud view labels do not establish useful floor, room and overhead views')
    for sample,role in zip(selected,('floor','room','overhead')):
        sample['cloud_view_role']=role;sample['cloud_advice_id']=result['advice_id']


def cloud_capacity_delay(advice):
    """Pace a new check after success; HTTP quota errors are never retried."""
    if advice.get('provider') == 'gemini':
        return 0.0  # Successful Vertex requests have no Groq input-token window.
    quota = advice.get('quota') or {}
    reset = str(quota.get('x-ratelimit-reset-tokens', ''))
    if not reset:
        return 65.0  # Include a small margin for Groq's separate input-token window.
    parts = re.findall(r'(\d+(?:\.\d+)?)(ms|h|m|s)', reset)
    if not parts or ''.join(number+unit for number,unit in parts) != reset:
        return 65.0
    delay = sum(float(number)*{'h':3600,'m':60,'s':1,'ms':.001}[unit] for number,unit in parts)
    if not math.isfinite(delay) or delay > 120:
        raise CameraCalibrationError('Groq capacity reset is too far away for this prepared head test')
    return max(65.0, delay + 5.0)


def recover_cloud_room(selected, reference, query, move_observe, minimum, maximum,
                       events, maximum_adjustments=3, initial_advice=None, allow_raised_room=False):
    """Correct a positively observed overhead middle view in small lower steps.

    The caller owns stopped-state/reference checks and bounded motor execution.
    Model text never supplies a command. Unknown endpoints cannot start recovery.
    """
    selected = list(selected)
    if len(selected)!=3 or any(s.get('view_usable') is not True for s in selected):
        raise CameraCalibrationError('Recovery requires three usable captured views')
    advice = initial_advice if initial_advice is not None else query(selected)
    for attempt in range(maximum_adjustments + 1):
        floor,room,upper = bound_cloud_observations(selected,advice,reference)
        events.append(dict(attempt=attempt,positions=[s['position'] for s in selected],advice=advice))
        if advice.get('upper_verified') is True:
            apply_cloud_advice(selected,advice,reference)
            return selected,advice
        if (allow_raised_room and floor.get('view') == 'floor_room'
                and floor.get('floor_visible') == 'yes'
                and room.get('view') in ('room', 'floor_room')
                and upper.get('view') == 'room' and upper.get('floor_visible') == 'no'):
            # The user chose the highest comfortable pose. A useful raised
            # room view is sufficient; do not relabel it as a verified ceiling.
            for sample, role in zip(selected, ('floor', 'room', 'raised')):
                sample.update(cloud_view_role=role, cloud_advice_id=advice['advice_id'])
            events[-1]['upper_acceptance'] = 'user_limit_with_usable_raised_room_view'
            return selected, advice
        if not (floor.get('view')=='floor_room' and floor.get('floor_visible')=='yes'
                and upper.get('view')=='ceiling' and upper.get('ceiling_visible')=='yes'
                and upper.get('floor_visible')=='no'
                and room.get('view')=='ceiling' and room.get('ceiling_visible')=='yes'
                and room.get('floor_visible')=='no'):
            raise CameraCalibrationError('Scene uncertainty does not establish a useful camera correction')
        if attempt == maximum_adjustments:
            raise CameraCalibrationError('Middle view remained overhead after three bounded lower adjustments')
        current = int(selected[1]['position'])
        floor_target = int(selected[0].get('target_position',selected[0]['position']))
        target = min(current + 5, int(maximum), floor_target - 5)
        if target < minimum or target - current < 3:
            raise CameraCalibrationError('No room for another middle-view adjustment inside the approved range')
        events[-1]['correction_target'] = target
        observed = move_observe(target)
        if (not observed.get('view_usable') or not observed.get('frame_sha256')
                or abs(int(observed['position'])-target)>3
                or int(observed['position']) <= current
                or not selected[2]['position'] < observed['position'] < selected[0]['position']):
            raise CameraCalibrationError('Middle-view correction did not produce a usable forward-progressing capture')
        selected[1] = observed
        advice = query(selected)
    raise CameraCalibrationError('Camera recovery ended without a verified view')


def floor_reference_visible(sample):
    if sample.get("view_usable") is True and cloud_reference_verified(sample, "floor"):
        return True
    try:
        floor, known = float(sample["floor_fraction"]), float(sample["known_fraction"])
        bottom = float(sample.get("bottom_floor_fraction", 0))
        bottom_known = float(sample.get("bottom_known_fraction", 0))
    except (KeyError, TypeError, ValueError):
        return False
    if not (sample.get("view_usable") is True and all(math.isfinite(v) for v in
            (floor, known, bottom, bottom_known)) and 0 <= floor <= known <= 1):
        return False
    # Whole-image coverage alone penalizes an otherwise useful low camera view
    # when furniture occupies the room. Require strong floor evidence at bottom.
    if floor >= .30 or (floor >= .12 and .45 <= bottom <= bottom_known <= 1 and bottom_known >= .70):
        return True
    # A central obstacle must not erase a known lower camera reference. Require
    # a confidently visible floor patch beside it; navigation still checks the
    # complete path independently, including the obstructed center.
    for side in ("left", "right"):
        try:
            side_floor = float(sample.get("bottom_" + side + "_floor_fraction", 0))
            side_known = float(sample.get("bottom_" + side + "_known_fraction", 0))
        except (TypeError, ValueError):
            continue
        if (floor >= .12 and math.isfinite(side_floor) and math.isfinite(side_known)
                and .70 <= side_floor <= side_known <= 1 and side_known >= .80):
            return True
    return False


def retained_floor_position(head, require_at_floor=True):
    """Validate the operator's physical reference; never infer or reset zero."""
    reference = head.get("reference_id")
    if (not head.get("require_approved_reference") or not reference
            or reference != head.get("approved_reference_id")
            or not head.get("homed") or head.get("moving") or head.get("homing")):
        raise CameraCalibrationError("Approved physical camera limits are unavailable for this encoder reference; head stayed stopped")
    position, target = int(head["position"]), int(head["down_position"])
    maximum = int(head["maximum_target_position"])
    physical_maximum = int(head.get("maximum_position", maximum))
    minimum = int(head["minimum_position"])
    if not (minimum <= position <= physical_maximum and minimum <= target <= maximum):
        raise CameraCalibrationError("Camera position or floor target is outside the approved physical range")
    if require_at_floor and abs(position - target) > 3:
        raise CameraCalibrationError("Camera has not reached the retained floor view")
    return target


def use_operator_limits(node, report):
    """Saved physical endpoints authorize search; route checks remain separate."""
    head = node.head
    limits = head.get('saved_limits')
    if limits is None:
        return False
    retained_floor_position(head, require_at_floor=False)
    reference = head['reference_id']
    if (head.get('manual_override') or limits.get('reference_id') != reference
            or limits.get('lower') != head.get('maximum_position')
            or limits.get('upper') != head.get('minimum_position')):
        raise CameraCalibrationError('Save both camera limits for this boot first')
    quality = fingerprint_quality(node.frame_fingerprint)
    # Operator endpoints already define the physical range. A plain wall at
    # startup does not invalidate them; floor clearance is checked before drive.
    if quality['mean_luma'] < 12:
        raise CameraCalibrationError('Camera view is too dark')
    previous = node.head_at or 0.0
    node.command_head({'action': 'use_saved_limits'})
    node.spin_until(lambda: node.head_at > previous
                    and node.head.get('calibrated') is True
                    and node.head.get('calibration_source') == limits.get('source', 'operator_limits')
                    and node.head.get('reference_id') == reference,
                    3.0, 'Bridge did not accept saved camera limits')
    report.update(outcome='calibrated', calibration_method=limits.get('source', 'operator_limits'),
                  reference_id=reference, mechanical_homing_attempted=False,
                  calibration={k: node.head[k] for k in
                               ('down_position', 'forward_position', 'up_position')},
                  final_position=node.head['position'],
                  route_verified=False, cloud_calls=0)
    return True


def probe_return_step(position, minimum_progress=3, maximum_step=15, floor_position=0):
    """A probe must return even when the raised pose is inside settling tolerance."""
    position = int(position) - int(floor_position)
    if abs(position) < minimum_progress:
        return None
    return max(-maximum_step, min(maximum_step, -position))


def choose_runtime_positions(samples, fallback_forward=None, fallback_down=None, floor_position=0):
    """Select floor/forward/upper views from confident whole-image semantics.

    Motor direction is a measured hardware property: negative counts raise
    this linkage. Scene labels select useful views inside that travel range;
    they never authorize extending it. No person is needed for calibration.
    """
    cloud = {}
    for sample in samples:
        role=sample.get('cloud_view_role')
        if role in ('floor','room','overhead','raised') and cloud_reference_verified(sample,role) and sample.get('view_usable'):
            position=int(sample.get('target_position',sample['position']))
            if abs(position-int(sample['position']))<=3:
                cloud['overhead' if role == 'raised' else role]=(position,sample)
    if len(cloud)==3 and len({s['cloud_advice_id'] for _,s in cloud.values()})==1:
        down,forward,up=(cloud[role][0] for role in ('floor','room','overhead'))
        if down==floor_position and down>forward>up:
            return dict(down_position=down,forward_position=forward,up_position=up,
                        floor_position=down,person_position=forward,face_position=up,
                        observed_range=[up,down],selection_reason='cloud_multi_view',floor_selection='cloud_multi_view')
    clean = []
    for sample in samples:
        try:
            actual = int(sample["position"])
            position = int(sample.get("target_position", actual))
            if abs(actual - position) > 3:
                continue
            floor = float(sample["floor_fraction"])
            known = float(sample["known_fraction"])
        except (KeyError, TypeError, ValueError):
            continue
        if (sample.get("view_usable") is True
                and math.isfinite(floor) and math.isfinite(known)
                and 0.0 <= floor <= known <= 1.0
                and (known >= 0.70 or sample.get("overhead_reference_verified") is True
                     or (position == floor_position and floor_reference_visible(sample)))):
            clean.append((position, floor, sample))
    # An explicit retained encoder pose, never a floor-pixel-derived zero.
    floors = [item for item in clean if item[0] == floor_position and floor_reference_visible(item[2])]
    if not floors:
        raise CameraCalibrationError(
            "No stable floor view was verified. Check the live image and floor "
            "segmentation; encoder positions alone cannot calibrate the camera."
        )
    down = max(floors, key=lambda item: (item[1], item[0]))
    upper_candidates = [item for item in clean if item[0] < down[0]]
    if not upper_candidates:
        raise CameraCalibrationError("No usable upper view inside the approved travel range")
    ceiling_candidates = [item for item in upper_candidates
                          if (float(item[2].get("ceiling_fraction", 0)) >= .20
                              or item[2].get("overhead_reference_verified") is True) and item[1] <= .05]
    if not ceiling_candidates:
        raise CameraCalibrationError("No confident upper ceiling view was verified")
    upper = min(ceiling_candidates, key=lambda item: item[0])
    forward_candidates = [
        item for item in clean
        if upper[0] < item[0] < down[0] and item[1] <= down[1] - .05
        and float(item[2].get("ceiling_fraction", 0)) <= .15
    ]
    if not forward_candidates:
        raise CameraCalibrationError(
            "No forward room view between the floor and ceiling views was verified"
        )
    forward = min(forward_candidates, key=lambda item: abs(item[1] - down[1] / 2.0))
    if down[1] - upper[1] < 0.10:
        raise CameraCalibrationError("Floor coverage did not change enough across the head sweep")
    return {
        "down_position": down[0], "forward_position": forward[0],
        "up_position": upper[0], "floor_position": down[0],
        "person_position": forward[0], "face_position": upper[0],
        "observed_range": [min(item[0] for item in clean), max(item[0] for item in clean)],
        "selection_reason": "semantic_floor_transition",
        "floor_selection": "semantic_floor",
    }


def verify_view_role(role, sample):
    """Recheck what a named pose sees; this does not establish route clearance."""
    if role == "down":
        if not floor_reference_visible(sample):
            raise CameraCalibrationError("Revisited down view did not match its semantic role")
        return True
    expected_cloud = {"up":"overhead", "forward":"room"}.get(role)
    if role == 'up' and sample.get('view_usable') is True and cloud_reference_verified(sample, 'raised'):
        return True
    if sample.get("view_usable") is True and cloud_reference_verified(sample, expected_cloud):
        return True
    known = float(sample.get("known_fraction", 0))
    ceiling = float(sample.get("ceiling_fraction", 0))
    if not sample.get("view_usable") or not math.isfinite(known) or (known < .70 and not (role == "up" and sample.get("overhead_reference_verified") is True)):
        raise CameraCalibrationError("Revisited camera view is uncertain")
    if role == "down":
        valid = floor_reference_visible(sample)
    elif role == "up":
        valid = (ceiling >= .20 or sample.get("overhead_reference_verified") is True) and float(sample.get("floor_fraction", 1)) <= .05
    elif role == "forward":
        valid = ceiling <= .15 and float(sample.get("floor_fraction", 1)) < .30
    else:
        valid = False
    if not valid:
        raise CameraCalibrationError("Revisited {0} view did not match its semantic role".format(role))
    return True


def validate_runtime_positions(samples, calibration, minimum_span_degrees=10, floor_position=0):
    """Require actual scene evidence as well as distinct, settled positions."""
    expected = choose_runtime_positions(samples, floor_position=floor_position)
    selected = {name: int(calibration[key]) for name, key in (
        ("floor", "floor_position"), ("person", "person_position"), ("face", "face_position")
    )}
    for key in ("floor_position", "person_position", "face_position",
                "down_position", "forward_position", "up_position"):
        if int(calibration[key]) != expected[key]:
            raise CameraCalibrationError("Selected camera poses do not match the scene evidence")
    positions = set(selected.values())
    if len(positions) != 3 or max(positions) - min(positions) < minimum_span_degrees:
        raise CameraCalibrationError(
            "Three distinct settled views spanning at least {0} encoder counts "
            "are required inside the approved range".format(minimum_span_degrees)
        )
    return {"verified": True, "selected_positions": selected,
            "view_span_degrees": max(positions) - min(positions)}


def scene_sample(result, position, reference_id, elapsed_seconds, maximum_age=2.0, require_homed=True):
    """Bind semantics to the stopped head pose, reference and fresh response."""
    if not isinstance(result, dict) or result.get("ok") is not True:
        raise CameraCalibrationError("Scene perception is unavailable")
    try:
        age = float(result["result_age_seconds"])
        head = result["camera_head"]
        scene = result["scene"]
        floor, ceiling, known = (float(scene[key]) for key in
                                 ("floor_fraction", "ceiling_fraction", "known_fraction"))
        matches = (head.get("available") and (head.get("homed") or not require_homed)
                   and not head.get("moving") and not head.get("homing")
                   and abs(head["position"] - position) <= 3
                   and head["reference_id"] == reference_id and bool(reference_id))
    except (KeyError, TypeError, ValueError):
        raise CameraCalibrationError("Scene evidence is incomplete; update the Mac perception service")
    if not all(math.isfinite(value) for value in (age, elapsed_seconds, floor, ceiling, known)):
        raise CameraCalibrationError("Scene evidence contains non-finite values")
    if not (0 <= age <= maximum_age and 0 <= elapsed_seconds <= maximum_age):
        raise CameraCalibrationError("Scene evidence is stale")
    if not matches:
        raise CameraCalibrationError("Scene evidence belongs to a different camera pose or reference")
    if not (0 <= floor <= known <= 1 and 0 <= ceiling <= known and floor + ceiling <= known + 1e-6):
        raise CameraCalibrationError("Scene coverage fractions are invalid")
    evidence = {"floor_fraction": floor, "ceiling_fraction": ceiling, "known_fraction": known,
                "scene_position": head['position']}
    for key in ("bottom_floor_fraction", "bottom_known_fraction", "bottom_left_floor_fraction", "bottom_left_known_fraction", "bottom_right_floor_fraction", "bottom_right_known_fraction"):
        if key in scene:
            value = float(scene[key])
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise CameraCalibrationError("Bottom scene coverage is invalid")
            evidence[key] = value
    reference_match = result.get("view_reference", {})
    evidence["overhead_reference_verified"] = bool(
        reference_match.get("verified") is True and reference_match.get("role") == "overhead"
        and reference_match.get("label_source") == "operator"
        and reference_match.get("scope") == "prepared_room")
    evidence['frame_sha256'] = result.get('frame_sha256')
    cloud = result.get('cloud_view_reference', {})
    if (cloud.get('verified') is True and cloud.get('provider') in ('gemini','groq')
            and cloud.get('label_source')==cloud.get('provider') and cloud.get('reference_id')==reference_id
            and cloud.get('role') in ('floor','room','overhead') and cloud.get('advice_id')):
        evidence['cloud_view_role']=cloud['role'];evidence['cloud_advice_id']=cloud['advice_id']
    return evidence


def fetch_route(url, timeout_seconds=8.0):
    if not url:
        return None
    request = Request(url, headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=timeout_seconds) as response:
        value = json.loads(response.read().decode("utf-8"))
    if not isinstance(value, dict) or not value.get("ok", False):
        raise CameraCalibrationError("route perception did not return a valid result")
    return value


def fetch_cloud_advice(url, samples, reference):
    request=Request(url,data=json.dumps(dict(reference_id=reference,
        frame_sha256=[s['frame_sha256'] for s in samples])).encode(),headers={'Content-Type':'application/json'})
    try:
        with urlopen(request,timeout=45) as response:
            return json.load(response)
    except Exception as exc:
        detail='Cloud vision quota/rate limit reached' if getattr(exc,'code',None)==429 else str(exc)
        if hasattr(exc,'close'):
            exc.close()
        raise CameraCalibrationError('Cloud scene calibration unavailable: '+detail)


def three_view_targets(head, floor_position, upper_limit):
    """Use the approved endpoints and a useful interior pose; never widen travel."""
    floor = int(floor_position)
    approved_upper = int(head['minimum_target_position'])
    upper = approved_upper if upper_limit is None else max(int(upper_limit), approved_upper)
    if floor - upper < 10:
        raise CameraCalibrationError('Approved camera range is too small for three views')
    middle = int(head.get('forward_position', round((floor + upper) / 2)))
    if not upper + 5 <= middle <= floor - 5:
        middle = round((floor + upper) / 2)
    return [floor, middle, upper]


def calibrate_three_views(node, upper_limit, report, minimum_span=10):
    """One lower/middle/upper capture sequence, one cloud decision, no revisits."""
    targets = three_view_targets(node.head, node.floor_position, upper_limit)
    report['planned_positions'] = targets
    report['calibration_method'] = 'three_views_cloud'
    samples = []
    for target in targets:
        node.move_to(target)
        node.last_view_target = target
        sample = node.observe_scene()
        if abs(int(sample['position']) - target) > 3:
            raise CameraCalibrationError('Camera did not settle near the requested view')
        samples.append(sample)
        report['samples'] = list(samples)
    selected, advice = node.recover_room(samples, 'three_views')
    report['cloud_scene_advice'] = advice
    report['selected_views'] = selected
    calibration = choose_runtime_positions(selected, floor_position=node.floor_position)
    report['calibration_readiness'] = validate_runtime_positions(
        selected, calibration, minimum_span, floor_position=node.floor_position)
    report['calibration'] = calibration
    return calibration


def dry_run_report(args):
    if args.visual_setup:
        return dict(outcome='dry_run_success', calibration_method='cloud_useful_views',
                    chassis_locked=True, mechanical_homing_attempted=False,
                    physical_limits_changed=False, maximum_observations=20 if args.discover_views else 16,
                    step_counts=10 if args.discover_views else 5, discovery=args.discover_views,
                    note='Reuse current saved views or discover them automatically after a reference change.' if args.auto_setup else
                         'Discover useful views from the current pose.' if args.discover_views else
                         'Opt-in only; requires approved current-reference bounds. Cloud uncertainty retains valid saved limits.')
    preview_upper = -48 if args.lower_limit is None else args.lower_limit
    targets = three_view_targets(dict(minimum_target_position=preview_upper), 0, preview_upper)
    return {
        'outcome': 'dry_run_success', 'chassis_locked': True,
        'floor_reference_position': 0, 'mechanical_homing_attempted': False,
        'planned_positions': targets, 'finish_at': targets[1],
        'calibration_method': 'three_views_cloud',
        'normal_speed': 'bridge configured speed',
        'single_retry_boost_speed': args.boost_speed,
        'note': 'Preview only; live targets use the approved current-reference bounds.',
    }


def run(args):
    if not args.execute:
        return dry_run_report(args)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray

    class CalibrationNode(Node):
        def __init__(self):
            super().__init__("echora_camera_head_calibration")
            self.robot = None
            self.robot_at = None
            self.camera = None
            self.camera_at = None
            self.frame_at = None
            self.frame_sequence = 0
            self.frame_fingerprint = None
            self.frame_error = None
            self.setup_frame = None
            self.setup_expected_range = None
            self.setup_origin = None
            self.head = None
            self.head_at = None
            self.expected_reference_id = None
            self.cloud_next_at = 0.0
            self.body = None
            self.body_at = None
            self.target_score = None
            self.target_at = None
            self.target_confirmed = False
            self.confirmed_at = None
            self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head_publisher = self.create_publisher(
                String, "/camera_head/command", 1
            )
            self.create_subscription(String, "/robot_status", self.on_robot, 10)
            self.create_subscription(String, "/camera/status", self.on_camera, 10)
            self.create_subscription(String, "/camera_head/status", self.on_head, 10)
            self.create_subscription(
                String, "/mission/target_observation", self.on_confirmation, 10
            )
            qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(Image, "/camera/image_raw", self.on_frame, qos)
            self.create_subscription(
                Detection2DArray,
                "/perception/person_detections",
                self.on_people,
                10,
            )
            self.create_subscription(
                Detection2DArray,
                "/perception/target_matches",
                self.on_matches,
                10,
            )

        @staticmethod
        def parse(message):
            try:
                value = json.loads(message.data)
            except ValueError:
                return None
            return value if isinstance(value, dict) else None

        def on_robot(self, message):
            value = self.parse(message)
            if value is not None:
                self.robot, self.robot_at = value, time.monotonic()

        def on_camera(self, message):
            value = self.parse(message)
            if value is not None:
                self.camera, self.camera_at = value, time.monotonic()

        def on_head(self, message):
            value = self.parse(message)
            if value is not None:
                self.head, self.head_at = value, time.monotonic()

        def on_frame(self, message):
            if args.visual_setup:
                self.setup_frame = message
            try:
                self.frame_fingerprint = frame_fingerprint(
                    message.data,
                    message.width,
                    message.height,
                    message.step,
                    message.encoding,
                )
                self.frame_error = None
            except (TypeError, ValueError) as exc:
                self.frame_fingerprint = None
                self.frame_error = str(exc)
            self.frame_sequence += 1
            self.frame_at = time.monotonic()

        def on_people(self, message):
            heights = [float(item.bbox.size_y) / 480.0 for item in message.detections]
            self.body = max(heights) if heights else None
            self.body_at = time.monotonic()

        def on_matches(self, message):
            scores = [
                float(result.hypothesis.score)
                for item in message.detections
                for result in item.results
            ]
            self.target_score = max(scores) if scores else None
            self.target_at = time.monotonic()

        def on_confirmation(self, message):
            value = self.parse(message)
            if value is None:
                return
            self.target_confirmed = bool(value.get("confirmed", False))
            self.confirmed_at = time.monotonic()

        def spin_until(self, predicate, timeout, reason):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if predicate():
                    return
            raise CameraCalibrationError(reason)

        def ready(self):
            now = time.monotonic()
            return (
                self.robot_at is not None
                and now - self.robot_at <= 1.0
                and not track_motion_active(self.robot)
                and self.head_at is not None
                and now - self.head_at <= 1.0
                and self.camera_at is not None
                and now - self.camera_at <= 6.5
                and self.camera.get("state") == "streaming"
                and self.frame_at is not None
                and now - self.frame_at <= 0.75
                and self.frame_fingerprint is not None
            )

        def stop_chassis(self):
            message = Twist()
            for _ in range(5):
                self.velocity_publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.05)
            self.spin_until(
                lambda: self.robot is not None and not track_motion_active(self.robot),
                2.0,
                "chassis stop was not confirmed",
            )

        def command_head(self, value):
            message = String()
            message.data = (
                value
                if isinstance(value, str)
                else json.dumps(value, separators=(",", ":"))
            )
            self.head_publisher.publish(message)

        def invalidate_calibration(self):
            previous = self.head_at or 0.0
            self.command_head("invalidate_calibration")
            self.spin_until(
                lambda: self.head_at is not None
                and self.head_at > previous
                and not self.head.get("calibrated", True),
                3.0,
                "bridge did not lock old camera calibration",
            )

        def wait_camera(self):
            self.spin_until(
                lambda: self.camera is not None
                and self.camera.get("state") == "streaming"
                and self.frame_at is not None
                and time.monotonic() - self.frame_at <= 0.75,
                8.0,
                "camera did not recover after head movement",
            )

        def capture_frame_after(self, sequence, observed_after, timeout=3.0):
            self.spin_until(
                lambda: self.frame_sequence > sequence
                and self.frame_at is not None
                and self.frame_at >= observed_after
                and self.frame_fingerprint is not None,
                timeout,
                self.frame_error or "no fresh frame arrived for camera feedback",
            )
            return (
                self.frame_sequence,
                self.frame_at,
                self.frame_fingerprint,
            )

        def setup_pause(self, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.setup_ready()

        def setup_prepare_discovery(self):
            from uuid import uuid4
            self.spin_until(lambda: self.ready() and not self.head.get('moving')
                            and not self.head.get('homing'), 15.0, 'Waiting for camera feedback')
            reference, position = self.head['reference_id'], self.head['position']
            request_id = uuid4().hex
            self.command_head(dict(action='begin_visual_setup', request_id=request_id,
                                   expected_reference_id=reference, expected_position=position))
            self.spin_until(lambda: (self.head.get('setup_reference_result') or {}).get('request_id') == request_id,
                            5.0, 'Camera setup reference confirmation did not arrive')
            result = self.head['setup_reference_result']
            if (not result.get('ok') or result.get('position') != position
                    or result.get('previous_reference_id') != reference
                    or result.get('reference_id') != self.head.get('reference_id')):
                raise SetupStateChanged(result.get('error', 'Camera reference changed while starting setup'))

        def setup_range(self):
            if args.discover_views:
                if self.setup_origin is None:
                    self.setup_origin = (self.head.get('reference_id'), self.head.get('position'))
                return discovery_setup_range(self.head, self.setup_origin)
            return approved_setup_range(self.head)

        def setup_ready(self):
            self.spin_until(lambda: self.ready() and not self.head.get('moving')
                            and not self.head.get('homing'), 15.0 if args.discover_views else 6.0,
                            'Waiting for fresh stopped camera and robot feedback')
            current = self.setup_range()
            if self.setup_expected_range is None:
                self.setup_expected_range = current
            elif current != self.setup_expected_range:
                raise SetupStateChanged('Camera reference or travel range changed during setup')

        def setup_anchor(self):
            """Bind evidence to the head reference and stopped track encoders."""
            anchor = [self.setup_range(), int(self.head['position'])]
            for side in ('left', 'right'):
                motor = self.robot.get('motors', {}).get(side, {})
                if (not motor.get('generation') or type(motor.get('position')) is not int
                        or motor.get('speed') != 0 or motor.get('commanded_speed') != 0):
                    raise SetupStateChanged('Tracks are not confirmed stopped for camera setup')
                anchor.append((motor['generation'], motor['position']))
            return anchor

        def setup_observe(self):
            from cv_bridge import CvBridge
            import cv2
            self.setup_ready()
            before = self.setup_anchor()
            tolerance = setup_tolerance(self.head)
            previous = self.frame_sequence
            started = time.monotonic()
            self.capture_frame_after(previous, started, timeout=4.0)
            self.setup_ready()
            frame = self.setup_frame
            if frame is None:
                raise SetupStateChanged('No source image available for camera setup')
            stamp = frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not -0.25 <= age <= 2.0:
                raise SetupStateChanged('Camera source image is stale')
            captured = self.setup_anchor()
            if (captured[0] != before[0] or abs(captured[1] - before[1]) > tolerance
                    or captured[2:] != before[2:]):
                raise SetupStateChanged('Camera or tracks changed during capture')
            pixels = CvBridge().imgmsg_to_cv2(frame, desired_encoding='bgr8')
            # Functional floor/room classification needs the whole view, not
            # face-level detail. Reduce cloud image tokens without cropping.
            scale = min(1.0, 448.0 / max(pixels.shape[:2]))
            if scale < 1:
                pixels = cv2.resize(pixels, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise SetupStateChanged('Camera setup image could not be encoded')
            try:
                result = request_setup_view(encoded.tobytes(), args.setup_advice_url)
            finally:
                # Inference may take seconds. Require newly received state after
                # the response; cached feedback cannot authorize the next move.
                after = time.monotonic()
                self.spin_until(lambda: self.ready() and self.head_at > after
                                and self.robot_at > after and not self.head.get('moving')
                                and not self.head.get('homing'), 6.0,
                                'Fresh robot feedback unavailable after cloud view check')
                fresh = self.setup_anchor()
                if (fresh[0] != before[0] or abs(fresh[1] - before[1]) > tolerance
                        or fresh[2:] != before[2:]):
                    raise SetupStateChanged('Camera reference, pose or tracks changed during cloud check')
            return dict(result, reference_id=before[0][0], position=before[1],
                        source_stamp_ns=stamp)

        def setup_move(self, target):
            self.setup_ready()
            reference, low, high = self.setup_range()
            tolerance = setup_tolerance(self.head)
            if type(target) is not int or not low <= target <= high:
                raise SetupStateChanged('Setup target is outside the current search range')
            if low <= self.head['position'] <= high and abs(self.head['position'] - target) <= tolerance:
                return
            if args.discover_views:
                # Search and return travel use the same normal-speed steps.
                # Motor acceptance uses the deployed tolerance, not an extra
                # three-count test that rejects a valid four-count landing.
                while abs(self.head['position'] - target) > tolerance:
                    origin = self.head['position']
                    step = max(origin - 10, min(origin + 10, target))
                    previous = self.head_at
                    self.command_head(dict(action='setup_jog', target=step,
                        expected_reference_id=reference, expected_position=origin))
                    self.spin_until(lambda: self.head_at > previous and self.ready()
                        and self.head.get('reference_id') == reference
                        and self.head.get('target_position') == step
                        and not self.head.get('moving') and not self.head.get('homing')
                        and abs(self.head['position'] - step) <= tolerance,
                        args.move_timeout, 'Camera discovery move did not settle')
                    self.setup_ready()
                    if abs(self.head['position'] - origin) < 1:
                        raise SetupStateChanged('Camera did not make progress')
                return
            previous = self.head_at
            self.command_head(dict(action='move_to', target=target))
            self.spin_until(lambda: self.head_at > previous and self.ready()
                            and self.head.get('reference_id') == reference
                            and not self.head.get('moving') and not self.head.get('homing')
                            and abs(self.head['position'] - target) <= tolerance,
                            args.move_timeout, 'Camera setup move did not reach its target')
            self.setup_ready()

        def setup_save_discovery(self, chosen):
            from uuid import uuid4
            self.setup_ready()
            request_id = uuid4().hex
            self.command_head(dict(action='save_visual_limits', request_id=request_id,
                expected_reference_id=self.head['reference_id'], expected_position=self.head['position'],
                lower=chosen['lower'], upper=chosen['upper']))
            self.spin_until(lambda: (self.head.get('limit_save_result') or {}).get('request_id') == request_id,
                            5.0, 'Visual limit save confirmation did not arrive')
            result = self.head['limit_save_result']
            if not result.get('ok') or not self.head.get('calibrated'):
                raise SetupStateChanged(result.get('error', 'Visual limits were not applied'))

        def setup_commit(self, calibration):
            reference = calibration['reference_id']
            previous = self.head_at
            self.command_head(dict(action='set_runtime_positions', reference_id=reference,
                                   forward=calibration['forward_position'], down=calibration['down_position'],
                                   up=calibration['up_position']))
            self.spin_until(lambda: self.head_at > previous
                            and self.head.get('calibrated') is True
                            and self.head.get('reference_id') == reference
                            and all(self.head.get(key) == calibration[key] for key in
                                    ('forward_position', 'down_position', 'up_position')),
                            4.0, 'Bridge did not confirm selected camera views')

        def setup_retain(self):
            reference, _, _ = approved_setup_range(self.head)
            limits = self.head.get('saved_limits') or {}
            if (limits.get('reference_id') != reference
                    or limits.get('lower') != self.head['maximum_position']
                    or limits.get('upper') != self.head['minimum_position']):
                raise SetupStateChanged('No matching operator limits available to retain')
            previous = self.head_at
            self.command_head(dict(action='use_saved_limits'))
            self.spin_until(lambda: self.head_at > previous and self.head.get('calibrated') is True
                            and self.head.get('reference_id') == reference
                            and self.head.get('calibration_source') == limits.get('source', 'operator_limits'),
                            4.0, 'Bridge did not confirm retained camera limits')

        def reference_floor(self):
            # Restore only a pose inside the SAME explicitly approved reference.
            # An unknown boot reference still cannot authorize any movement.
            self.floor_position = retained_floor_position(self.head, require_at_floor=False)
            self.expected_reference_id = self.head["reference_id"]
            self.invalidate_calibration()
            self.move_to(self.floor_position)
            self.last_view_target = self.floor_position
            report["retained_floor_position"] = self.floor_position
            self.wait_camera()
            positions = []
            stable_positions = []
            last_update = self.head_at or 0.0
            minimum_deadline = time.monotonic() + args.reference_hold_seconds
            deadline = minimum_deadline + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.head_at is not None and self.head_at > last_update:
                    last_update = self.head_at
                    positions.append(int(self.head.get("position", 999)))
                    stable_positions = stable_reference_tail(positions)
                if (
                    time.monotonic() >= minimum_deadline
                    and len(stable_positions) >= 3
                ):
                    break
            reference = {
                "position": int(self.head.get("position", 999)),
                "held_positions": positions,
                "stable_tail": stable_positions,
                "verified": len(stable_positions) >= 3,
            }
            report["floor_reference"] = reference
            if not positions:
                raise CameraCalibrationError(
                    "camera floor reference produced no hold samples"
                )
            if len(stable_positions) < 3 or abs(int(self.head["position"]) - self.floor_position) > 3:
                raise CameraCalibrationError(
                    "camera floor reference did not settle; observed span {0} counts".format(
                        max(positions) - min(positions)
                    )
                )

        def move_to(self, target):
            current = int(self.head["position"])
            moves = 0
            while abs(current - target) > args.tolerance:
                moves += 1
                if moves > 24:
                    raise CameraCalibrationError(
                        "camera head could not converge on {0}".format(target)
                    )
                delta = target - current
                step = max(-args.step_degrees, min(args.step_degrees, delta))
                current = self.jog_once(step)
            return current

        def query_cloud(self, samples):
            self.spin_until(self.ready,5.0,'Fresh stopped state required for cloud view check')
            reference = self.expected_reference_id
            position = int(self.head['position'])
            def unchanged():
                return (self.ready() and self.head.get('homed')
                        and not self.head.get('moving') and not self.head.get('homing')
                        and self.head.get('reference_id')==reference
                        and abs(int(self.head['position'])-position)<=3)
            delay=max(0.0,self.cloud_next_at-time.monotonic())
            if delay:
                report.setdefault('cloud_capacity_waits',[]).append(round(delay,2))
                print('Camera held stopped while waiting {0:.0f}s for Groq capacity.'.format(delay),flush=True)
            while time.monotonic() < self.cloud_next_at:
                rclpy.spin_once(self,timeout_sec=.05)
                if not unchanged():
                    raise CameraCalibrationError('Camera or robot state changed while waiting for cloud capacity')
            if not unchanged():
                raise CameraCalibrationError('Fresh unchanged camera reference required for cloud interpretation')
            advice=fetch_cloud_advice(args.scene_advice_url,samples,reference)
            after=time.monotonic()
            self.spin_until(lambda: unchanged() and self.head_at>after and self.robot_at>after,
                            5.0,'Camera changed or fresh stopped state was unavailable after cloud interpretation')
            self.cloud_next_at=time.monotonic()+cloud_capacity_delay(advice)
            return advice

        def recover_room(self, selected, context):
            def capture(target):
                self.move_to(target)
                self.last_view_target=target
                return self.observe_scene()
            events=[]
            report.setdefault('semantic_recovery',[]).append(dict(context=context,attempts=events))
            limits = self.head.get('saved_limits') or {}
            manual_upper = (limits.get('reference_id') == self.expected_reference_id
                and limits.get('upper') == self.head['minimum_position']
                and limits.get('lower') == self.head['maximum_position']
                and selected[2].get('target_position') == self.head['minimum_target_position'])
            return recover_cloud_room(selected,self.expected_reference_id,self.query_cloud,capture,
                int(self.head.get('minimum_target_position',self.head['minimum_position'])),
                int(self.head['maximum_target_position']),events,allow_raised_room=manual_upper)

        def jog_once(self, step):
            self.spin_until(self.ready, 3.0, "fresh stopped robot and camera state is required")
            if self.head.get("reference_id") != self.expected_reference_id:
                raise CameraCalibrationError("Camera reference changed before movement")
            minimum = int(self.head.get("minimum_position", args.lower_limit))
            minimum_target = int(self.head.get("minimum_target_position", minimum))
            maximum = int(self.head.get("maximum_target_position", self.head.get("maximum_position", 0)))
            physical_maximum = int(self.head["maximum_position"])
            step = int(step)
            if step == 0 or abs(step) > args.step_degrees:
                raise CameraCalibrationError(
                    "camera calibration accepts only one bounded 15-count step"
                )
            origin = int(self.head["position"])
            target = min(maximum, max(minimum_target, origin + step))
            if target == origin:
                raise CameraLimitReached("camera head is already at its limit")
            direction = 1 if target > origin else -1
            speeds = [None]
            # Negative counts lift the current heavy linkage, so the upward
            # step is the one allowed a single near-rated retry.
            if step < 0 and args.boost_speed:
                speeds.append(int(args.boost_speed))
            for attempt_index, speed in enumerate(speeds):
                current = int(self.head["position"])
                remaining = target - current
                if remaining * direction <= 0:
                    return current
                if abs(remaining) > args.step_degrees:
                    raise CameraCalibrationError(
                        "camera back-drove beyond the bounded retry range"
                    )
                before_sequence = self.frame_sequence
                before_frame = self.frame_fingerprint
                if before_frame is None:
                    raise CameraCalibrationError(
                        self.frame_error or "camera feedback frame is unavailable"
                    )
                previous = self.head_at or 0.0
                request = {"action": "jog_to", "target": target}
                if speed is not None:
                    request["speed"] = speed
                command_started = time.monotonic()
                self.command_head(request)
                deadline = command_started + args.move_timeout
                last_update = previous
                last_progress_at = command_started
                furthest_progress = 0
                stable_position = None
                stable_updates = 0
                actual = current
                acknowledged = False
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    if self.head_at is None or self.head_at <= last_update:
                        continue
                    last_update = self.head_at
                    acknowledged = acknowledged or self.head.get('target_position') == target
                    if not acknowledged:
                        continue
                    actual = int(self.head.get("position", current))
                    attempt_progress = max(0, (actual - current) * direction)
                    total_progress = max(0, (actual - origin) * direction)
                    if attempt_progress > furthest_progress:
                        furthest_progress = attempt_progress
                        last_progress_at = time.monotonic()
                    if self.head.get("moving", True) or self.head.get("homing", True):
                        stable_position = None
                        stable_updates = 0
                        progress_timeout = (
                            args.boost_no_progress_timeout
                            if speed is not None
                            else args.no_progress_timeout
                        )
                        if time.monotonic() - last_progress_at >= progress_timeout:
                            break
                        continue
                    if stable_position is not None and abs(actual - stable_position) <= 2:
                        stable_updates += 1
                    else:
                        stable_position = actual
                        stable_updates = 1
                    if stable_updates >= 2:
                        if total_progress < args.minimum_progress:
                            if time.monotonic()-last_progress_at >= args.no_progress_timeout:
                                break
                            continue
                        if (
                            actual < minimum
                            or actual > physical_maximum
                        ):
                            raise CameraCalibrationError(
                                "camera head moved outside its software range"
                            )
                        self.last_view_target = target
                        stopped_at = time.monotonic()
                        self.wait_camera()
                        if not args.probe_only:
                            # Transit checks motor progress; semantic images are
                            # captured only at the three requested views.
                            report.setdefault('motion_feedback', []).append(dict(
                                start_position=current, bounded_target=target,
                                actual_position=actual, encoder_progress=total_progress))
                            return actual
                        # Encoder stop precedes the end of visible flex in this
                        # loaded linkage.  Discard that settling interval, then
                        # record image change for diagnostics. Scene contents
                        # and small camera shifts need not repeat pixel-for-pixel.
                        settle_deadline = stopped_at + args.visual_settle_seconds
                        while time.monotonic() < settle_deadline:
                            rclpy.spin_once(self, timeout_sec=0.05)
                        after_sequence, after_at, after_frame = self.capture_frame_after(
                            before_sequence, settle_deadline
                        )
                        hold_deadline = after_at + args.visual_hold_seconds
                        while time.monotonic() < hold_deadline:
                            rclpy.spin_once(self, timeout_sec=0.05)
                        _, _, settled_frame = self.capture_frame_after(
                            after_sequence, hold_deadline
                        )
                        feedback = {
                            "start_position": current,
                            "origin_position": origin,
                            "requested_step": remaining,
                            "bounded_target": target,
                            "actual_position": actual,
                            "encoder_progress": total_progress,
                            "speed": int(speed or self.head.get("speed", 0)),
                            "boosted": speed is not None,
                        }
                        try:
                            ceiling = self.observe_scene().get("ceiling_fraction", 0) >= 0.65
                            feedback["visual"] = verify_camera_step(
                                before_frame, after_frame, settled_frame,
                                allow_textureless_view=ceiling,
                            )
                        except CameraMotionFeedbackError as exc:
                            if exc.report is not None:
                                feedback["visual"] = exc.report
                            if not usable_motion_feedback(exc.report,ceiling):
                                feedback["visual_error"] = str(exc)
                                report.setdefault("motion_feedback", []).append(feedback)
                                raise CameraCalibrationError(str(exc))
                            feedback['visual_note']=str(exc)
                            feedback['acceptance']='encoder_stop_and_usable_images_pending_semantic_check'
                        report.setdefault("motion_feedback", []).append(feedback)
                        return actual

                self.command_head("stop")
                stopped_after = self.head_at or 0.0
                self.spin_until(
                    lambda: self.head_at is not None
                    and self.head_at > stopped_after
                    and not self.head.get("moving", True)
                    and not self.head.get("homing", True),
                    3.0,
                    "camera head stop/hold was not confirmed",
                )
                report.setdefault("motion_attempts", []).append(
                    {
                        "start_position": current,
                        "origin_position": origin,
                        "requested_step": remaining,
                        "bounded_target": target,
                        "actual_position": int(self.head.get("position", actual)),
                        "encoder_progress": max(
                            0,
                            (
                                int(self.head.get("position", actual)) - origin
                            )
                            * direction,
                        ),
                        "attempt_progress": furthest_progress,
                        "speed": int(speed or self.head.get("speed", 0)),
                        "boosted": speed is not None,
                        "outcome": "no_progress",
                    }
                )
                # The brick already boosts a stalled upward move once. Do not
                # reset its watchdog by starting another Jetson retry cycle.
                retry_limit = int(self.head.get("stall_retry_limit", 0))
                if retry_limit and int(self.head.get("stall_retry_count", 0)) >= retry_limit:
                    raise CameraLimitReached("camera head stalled after its bounded EV3 retry")
                if attempt_index + 1 < len(speeds):
                    continue
                raise CameraLimitReached(
                    "camera head could not complete one 15-count step"
                )
            raise CameraLimitReached("camera head step had no safe attempt")

        def observe_scene(self, require_homed=True):
            sample = self.observe()
            reference = self.head.get("reference_id")
            if self.expected_reference_id is not None and reference != self.expected_reference_id:
                raise CameraCalibrationError("Camera reference changed during observation")
            views = []
            for _ in range(2):
                started = time.monotonic()
                result = fetch_route(args.route_url, args.route_timeout)
                views.append(scene_sample(result, sample["position"], reference,
                                          time.monotonic() - started,
                                          require_homed=require_homed))
            for key in ("floor_fraction", "ceiling_fraction", "known_fraction", "bottom_floor_fraction", "bottom_known_fraction", "bottom_left_floor_fraction", "bottom_left_known_fraction", "bottom_right_floor_fraction", "bottom_right_known_fraction"):
                if all(key in view for view in views):
                    sample[key] = min(view[key] for view in views)
            sample["overhead_reference_verified"] = all(v.get("overhead_reference_verified") for v in views)
            sample['frame_sha256']=views[-1].get('frame_sha256')
            sample['position']=views[-1]['scene_position']
            if (views[0].get('cloud_view_role') and views[0].get('cloud_view_role')==views[1].get('cloud_view_role')
                    and views[0].get('cloud_advice_id')==views[1].get('cloud_advice_id')):
                sample['cloud_view_role']=views[0]['cloud_view_role'];sample['cloud_advice_id']=views[0]['cloud_advice_id']
            # A confidently observed, bright ceiling can be a valid upper
            # view despite lacking texture. Encoder stop, image freshness and
            # the useful scene are checked separately; pixel drift is diagnostic.
            if (sample["ceiling_fraction"] >= 0.65 and sample["known_fraction"] >= 0.70
                    and 12 <= sample["image_mean_luma"] <= 245):
                sample["view_usable"] = True
            return sample

        def observe(self):
            deadline = time.monotonic() + args.dwell_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            quality = fingerprint_quality(self.frame_fingerprint)
            return {
                "position": int(self.head["position"]),
                "target_position": getattr(self, "last_view_target", int(self.head["position"])),
                "image_mean_luma": quality["mean_luma"],
                "image_contrast": quality["mean_absolute_contrast"],
                "view_usable": quality["usable"],
                "body_height_fraction": (
                    round(self.body, 4)
                    if self.body is not None
                    and self.body_at is not None
                    and now - self.body_at <= 1.0
                    else None
                ),
                "target_score": (
                    round(self.target_score, 4)
                    if self.target_score is not None
                    and self.target_at is not None
                    and now - self.target_at <= 1.0
                    else None
                ),
                "target_confirmed": bool(
                    self.target_confirmed
                    and self.confirmed_at is not None
                    and now - self.confirmed_at <= 1.0
                ),
            }

    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "chassis_locked": True,
        "samples": [],
    }
    with camera_control_lease(exclusive=True):
        rclpy.init()
        node = CalibrationNode()
        try:
            node.spin_until(node.ready, 10.0, "camera or EV3 state is unavailable")
            node.stop_chassis()
            if args.auto_setup:
                try:
                    approved_setup_range(node.head)
                except SetupStateChanged:
                    args.discover_views = True
                else:
                    if use_operator_limits(node, report):
                        return report
                    args.discover_views = True
            if args.visual_setup:
                return run_visual_discovery(node, report) if args.discover_views else run_visual_setup(node, report)
            if not args.probe_only and not args.verify_views and use_operator_limits(node, report):
                return report
            node.reference_floor()
            report["reference_method"] = "retained_operator_boundary_no_zero"
            report["mechanical_homing_attempted"] = False
            reference_id = node.head.get("reference_id")
            if not reference_id:
                raise CameraCalibrationError("EV3 camera reference is unavailable; update its service")
            # The bridge's current approved limit always bounds the requested sweep.
            approved_upper = int(node.head.get("minimum_target_position", node.head["minimum_position"]))
            args.lower_limit = approved_upper if args.lower_limit is None else max(args.lower_limit, approved_upper)
            if args.probe_only:
                floor_before = node.observe_scene()
                if not floor_reference_visible(floor_before):
                    raise CameraCalibrationError("Retained floor view changed")
                node.jog_once(-args.step_degrees)
                report["raised_view"] = node.observe_scene()
                # The outbound move may end exactly one settling tolerance
                # from the retained pose. A probe must still issue a return.
                position = int(node.head["position"])
                return_step = probe_return_step(position, args.minimum_progress, maximum_step=args.step_degrees, floor_position=node.floor_position)
                if return_step is not None:
                    node.jog_once(return_step)
                report["returned_view"] = node.observe_scene()
                if not floor_reference_visible(report["returned_view"]):
                    raise CameraCalibrationError("The camera did not recover the approved floor view")
                report["final_position"] = int(node.head["position"])
                report["outcome"] = "probe_complete"
                return report
            calibration = calibrate_three_views(node, args.lower_limit, report,
                                                args.minimum_calibration_span_degrees)

            node.command_head(
                {
                    "action": "set_runtime_positions",
                    "forward": calibration["forward_position"],
                    "down": calibration["down_position"],
                    "up": calibration["up_position"],
                    "reference_id": reference_id,
                }
            )
            accepted_after = node.head_at or 0.0
            node.spin_until(
                lambda: node.head is not None
                and node.head_at > accepted_after
                and node.head.get("calibrated") is True
                and node.head.get("reference_id") == reference_id
                and int(node.head.get("forward_position", 999))
                == calibration["forward_position"]
                and int(node.head.get("down_position", 999))
                == calibration["down_position"]
                and int(node.head.get("up_position", 999))
                == calibration["up_position"],
                3.0,
                "bridge did not accept runtime camera positions",
            )
            node.move_to(calibration["forward_position"])
            report["final_position"] = int(node.head["position"])
            report["outcome"] = "calibrated"
        except Exception as exc:
            report["error"] = str(exc)
            if args.visual_setup:
                report['failure_head'] = {key: (node.head or {}).get(key) for key in
                    ('reference_id', 'approved_reference_id', 'manual_override', 'homed',
                     'moving', 'homing', 'position', 'target_position', 'calibrated',
                     'calibration_error', 'minimum_target_position', 'maximum_target_position')}
            try:
                node.command_head("stop")
                node.stop_chassis()
                if not args.visual_setup:
                    node.invalidate_calibration()
                    report["calibration_locked_after_failure"] = True
            except Exception as stop_exc:
                report["stop_error"] = str(stop_exc)
        finally:
            report["finished_at_unix"] = time.time()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument('--visual-setup', action='store_true',
                        help='opt-in Gemini useful-view setup within approved bounds; retains valid limits on cloud uncertainty')
    parser.add_argument('--discover-views', action='store_true',
                        help='automatically discover and save useful views from the current pose; no lower-reference restore')
    parser.add_argument('--auto-setup', action='store_true',
                        help='reuse matching saved views, otherwise discover views automatically')
    parser.add_argument('--setup-advice-url', default='http://127.0.0.1:18091/camera-setup')
    parser.add_argument("--verify-views", action="store_true",
                        help="optionally verify saved camera views using Gemini")
    parser.add_argument("--probe-only", action="store_true",
                        help="verify retained floor pose and one bounded up/return cycle; never drive tracks")
    parser.add_argument(
        "--lower-limit",
        "--upper-limit",
        dest="lower_limit",
        type=int,
        default=None,
        help="optional stricter upper-view bound; defaults to this reference's approved target limit",
    )
    parser.add_argument("--step-degrees", type=int, default=15)
    parser.add_argument("--tolerance", type=int, default=3)
    parser.add_argument("--minimum-progress", type=int, default=2)
    parser.add_argument("--move-timeout", type=float, default=10.0)
    parser.add_argument("--no-progress-timeout", type=float, default=2.5)
    parser.add_argument("--boost-speed", type=int, default=1500)
    parser.add_argument("--boost-no-progress-timeout", type=float, default=1.5)
    parser.add_argument("--reference-hold-seconds", type=float, default=1.0)
    parser.add_argument("--visual-settle-seconds", type=float, default=5.0)
    parser.add_argument("--visual-hold-seconds", type=float, default=1.0)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--match-score", type=float, default=0.45)
    parser.add_argument("--minimum-calibration-span-degrees", type=int, default=10)
    parser.add_argument(
        "--route-url", default="http://127.0.0.1:18091/view"
    )
    parser.add_argument("--route-timeout", type=float, default=8.0)
    parser.add_argument("--scene-advice-url", default="http://127.0.0.1:18091/scene-advice")
    parser.add_argument(
        "--report",
        default="/home/animesh/echora/logs/camera_head_calibration.json",
    )
    args = parser.parse_args(argv)
    if args.discover_views or args.auto_setup:
        args.visual_setup = True
    if args.visual_setup and (args.probe_only or args.verify_views):
        parser.error('--visual-setup cannot be combined with --probe-only or --verify-views')
    if not 5 <= args.step_degrees <= 15:
        parser.error("camera calibration steps must be 5 to 15 counts")
    if args.tolerance < 1 or args.tolerance >= args.step_degrees:
        parser.error("tolerance must be positive and smaller than the step")
    if args.minimum_progress < 1 or args.minimum_progress > args.step_degrees:
        parser.error("minimum-progress must be between 1 and the step size")
    if (
        args.move_timeout <= 0
        or args.no_progress_timeout <= 0
        or args.boost_no_progress_timeout <= 0
        or args.dwell_seconds <= 0
        or args.reference_hold_seconds <= 0
        or args.visual_settle_seconds <= 0
        or args.visual_hold_seconds <= 0
    ):
        parser.error("timeouts and dwell must be positive")
    if args.no_progress_timeout >= args.move_timeout:
        parser.error("no-progress timeout must be shorter than move timeout")
    if args.boost_no_progress_timeout >= args.no_progress_timeout:
        parser.error("boost no-progress timeout must be shorter than normal")
    if args.boost_speed < 300 or args.boost_speed > 1500:
        parser.error("boost-speed must be between 300 and 1500")
    if args.minimum_calibration_span_degrees < 10:
        parser.error("minimum calibration span must be at least 10 counts")
    return args


def main(argv=None):
    args = parse_args(argv)
    report = run(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.execute:
        os.makedirs(os.path.dirname(args.report), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
    return 0 if report["outcome"] in ("calibrated", "probe_complete", "dry_run_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
