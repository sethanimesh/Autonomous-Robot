"""Choose functional views inside an independently approved camera range.

No ROS or motor access at import. The runner supplies fresh bound observations
and movements; this module never derives a physical stop from an image.
"""
import base64
import hashlib
import json
import re
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from uuid import uuid4


class SetupUnavailable(RuntimeError):
    """Visual selection can be deferred while valid operator limits remain."""


class SetupStateChanged(RuntimeError):
    """The current pose/reference cannot authorize another setup action."""


class SetupRetryLater(SetupUnavailable):
    def __init__(self, quota):
        text = str(quota.get('retry-after') or quota.get('x-ratelimit-reset-tokens') or '15s')
        try:
            seconds = float(text)
        except ValueError:
            seconds = sum(float(value) * dict(ms=.001, s=1, m=60, h=3600)[unit]
                          for value, unit in re.findall(r'([0-9.]+)(ms|s|m|h)', text)) or 15
        self.retry_after = max(1, min(60, seconds + 1))
        super().__init__('Cloud vision temporarily unavailable; continuing after a short cooldown')


class SetupRateLimited(SetupRetryLater):
    """Provider quota cooldown, distinct from a temporary service failure."""


def setup_tolerance(head):
    """Match the deployed bridge's small settling allowance, capped below a step."""
    value = head.get('settle_tolerance', 3)
    return max(3, min(4, value)) if type(value) is int else 3


def approved_setup_range(head):
    reference = head.get('reference_id')
    if (not reference or reference != head.get('approved_reference_id')
            or not head.get('require_approved_reference') or not head.get('homed')
            or head.get('available') is False or head.get('manual_override')):
        raise SetupStateChanged('Restore the saved range from a confirmed lower view first')
    names = ('minimum_target_position', 'maximum_target_position', 'position',
             'minimum_position', 'maximum_position')
    if any(type(head.get(name)) is not int for name in names):
        raise SetupStateChanged('Camera range or position is unavailable')
    low, high = head['minimum_target_position'], head['maximum_target_position']
    if not (head['minimum_position'] <= low < high <= head['maximum_position']
            and high - low >= 10
            and head['minimum_position'] <= head['position'] <= head['maximum_position']):
        raise SetupStateChanged('Camera position or saved range is invalid')
    return reference, low, high


def discovery_setup_range(head, origin, travel=60):
    """A finite relative search window, independent of obsolete saved angles."""
    reference, start = origin
    if (not reference or head.get('reference_id') != reference
            or head.get('available') is False or type(head.get('position')) is not int):
        raise SetupStateChanged('Camera reference changed during visual discovery')
    if abs(head['position'] - start) > travel + setup_tolerance(head):
        raise SetupStateChanged('Camera left the visual discovery window')
    return reference, start - travel, start + travel


def run_visual_discovery(node, report, maximum_observations=20):
    """Find, verify and save useful views in one run with no per-step review.

    Gemini classifies images; these rules choose directions and ten-count steps.
    The adapter supplies stopped, current-reference observations and normal-speed
    movement. The search budget is not a claimed mechanical travel measurement.
    """
    node.setup_prepare_discovery()
    node.setup_ready()
    expected = node.setup_range()
    reference, low, high = expected
    samples = report.setdefault('visual_setup_samples', [])
    cooldown = 0.0
    report.update(calibration_method='cloud_useful_views', mechanical_homing_attempted=False,
                  physical_limits_changed=False, operating_limits_changed=False,
                  route_verified=False, discovery_window=[low, high])
    minimum_span = max(20, node.head.get('lower_target_margin', 6)
                       + node.head.get('upper_target_margin', 3) + 10)
    chosen = {}
    visited = set()

    def observe(goal):
        nonlocal cooldown
        attempt = 0
        while attempt < 2:
            node.setup_ready()
            if node.setup_range() != expected:
                raise SetupStateChanged('Camera reference changed during discovery')
            if len(samples) >= maximum_observations:
                raise SetupUnavailable('Useful-view discovery reached its observation budget')
            try:
                result = node.setup_observe()
            except SetupRetryLater as exc:
                if cooldown + exc.retry_after > 120:
                    raise SetupUnavailable('Cloud vision did not recover within two minutes')
                cooldown += exc.retry_after
                report['cloud_cooldown_seconds'] = round(cooldown, 2)
                print('Visual setup: cloud cooldown, resuming in {0:.0f}s'.format(exc.retry_after), flush=True)
                node.setup_pause(exc.retry_after)
                continue
            except SetupUnavailable:
                if attempt == 0:
                    attempt += 1
                    continue
                raise
            node.setup_ready()
            if (node.setup_range() != expected or result.get('reference_id') != reference
                    or type(result.get('position')) is not int
                    or abs(result['position'] - node.head['position']) > setup_tolerance(node.head)):
                raise SetupStateChanged('View no longer matches the stopped camera')
            samples.append(dict(goal=goal, **result))
            action = result.get('decisions', {}).get(goal, {}).get('action')
            observations = result.get('observations') or [{}]
            observation = observations[0]
            if (action == 'observe_again' and observation.get('quality') == 'clear'
                    and observation.get('scene') in ('room', 'upper_room', 'floor_room')):
                # A readable furniture/curtain view can be explored even when
                # the model dislikes its room context. Only actual candidate
                # classifications may become saved endpoints.
                action = 'look_down' if goal == 'lower' else 'look_up'
                samples[-1]['discovery_action'] = action
            print('Visual setup: {0}, position {1}, {2}'.format(goal, result['position'], action), flush=True)
            if action not in ('observe_again', 'recover_visibility'):
                return result, action
            attempt += 1
        raise SetupUnavailable('Camera view remains unreadable; retry setup when visibility recovers')

    for goal in ('lower', 'upper'):
        if goal == 'upper':
            # Reuse an upper view already seen while finding the floor. This
            # avoids scanning the same positions again or insisting on ceiling.
            candidates = [s['position'] for s in samples
                          if s.get('decisions', {}).get('upper', {}).get('action') == 'candidate_view'
                          and chosen['lower'] - s['position'] >= minimum_span]
            if candidates:
                chosen['upper'] = min(candidates)
                break
            # Both suitability decisions already came with the lower image.
            # Move to a distinct view before making another paid cloud call.
            target = chosen['lower'] - minimum_span
            if target < low:
                raise SetupUnavailable('No room for a distinct upper view in the discovery window')
            node.setup_move(target)
        while True:
            result, action = observe(goal)
            position = result['position']
            if action == 'candidate_view':
                if goal == 'lower' or chosen['lower'] - position >= minimum_span:
                    chosen[goal] = position
                    break
                action = 'look_up'
            if action not in ('look_up', 'look_down'):
                raise SetupUnavailable('No usable camera direction from this view')
            target = position + (-10 if action == 'look_up' else 10)
            key = (goal, round(position / 5), action)
            if not low <= target <= high or key in visited:
                raise SetupUnavailable('No useful endpoint found in this discovery window')
            visited.add(key)
            node.setup_move(target)

    # Verify the real commandable views after applying the existing small
    # settling margins, then save both endpoints together (never a half range).
    for goal in ('upper', 'lower'):
        margin = (-node.head.get('lower_target_margin', 6) if goal == 'lower'
                  else node.head.get('upper_target_margin', 3))
        for attempt in range(3):
            node.setup_move(chosen[goal] + margin)
            _, action = observe(goal)
            if action == 'candidate_view':
                break
            # A settling margin can lose the thin strip of floor that made
            # the candidate useful. Adjust and recheck automatically instead
            # of failing the entire setup on this first return mismatch.
            if attempt == 2 or action not in ('look_up', 'look_down'):
                raise SetupUnavailable('Return view did not confirm the {0} endpoint'.format(goal))
            target = chosen[goal] + (-10 if action == 'look_up' else 10)
            if not low <= target <= high:
                raise SetupUnavailable('Return correction reached the discovery window')
            node.setup_move(target)
            corrected, correction = observe(goal)
            if correction != 'candidate_view':
                raise SetupUnavailable('Adjusted endpoint is not a useful {0} view'.format(goal))
            chosen[goal] = corrected['position']
            if chosen['lower'] - chosen['upper'] < minimum_span:
                raise SetupUnavailable('Adjusted views are too close together')
    node.setup_save_discovery(chosen)
    report.update(outcome='calibrated', visual_setup_outcome='discovered',
                  operating_limits_changed=True, return_views_verified=True,
                  observed_endpoints=chosen, final_position=node.head['position'],
                  calibration={key: node.head[key] for key in
                               ('down_position', 'up_position', 'forward_position')})
    return report


def request_setup_view(jpeg, url, timeout=50):
    """Send one caller-captured frame, without a filename or expected view label."""
    request_id = uuid4().hex
    digest = hashlib.sha256(jpeg).hexdigest()
    body = json.dumps(dict(request_id=request_id,
                          jpeg_base64=base64.b64encode(jpeg).decode('ascii'))).encode()
    request = Request(url, data=body, headers={'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=timeout) as response:
            result = json.load(response)
    except HTTPError as exc:
        if exc.code in (429, 502, 503, 504):
            status = exc.code
            delay = exc.headers.get('Retry-After') if exc.headers else None
            quota = {'retry-after': delay} if delay else {}
            try:
                payload = json.loads(exc.read(8192))
                supplied = payload.get('quota', {})
                if isinstance(supplied, dict):
                    quota.update(supplied)
            except (ValueError, AttributeError):
                pass
            exc.close()
            retry = SetupRateLimited if status == 429 else SetupRetryLater
            raise retry(quota) from None
        exc.close()
        raise SetupUnavailable('Cloud camera-view check unavailable (HTTP {0})'.format(exc.code)) from None
    except Exception as exc:
        # Never print uploaded image bytes, credentials, or provider payloads.
        raise SetupUnavailable('Cloud camera-view check unavailable ({0})'.format(type(exc).__name__)) from None
    if (not isinstance(result, dict) or result.get('request_id') != request_id
            or result.get('frame_sha256') != [digest] or result.get('provider') not in ('groq', 'gemini')
            or result.get('advisory_only') is not True):
        raise SetupUnavailable('Cloud camera-view response did not match the captured image')
    return result


def select_setup_views(node, report, maximum_observations=16):
    """Observe, take small steps and retain two distinct useful operating views.

    A failed/ambiguous cloud result never changes the approved travel range.
    The caller may retain valid operator limits when selection is unavailable.
    """
    expected = approved_setup_range(node.head)
    reference, low, high = expected
    samples = report.setdefault('visual_setup_samples', [])
    chosen = {}
    visited = set()
    for goal in ('lower', 'upper'):
        # Check the known operating ends first: normally two cloud calls, not
        # a cloud request at every intermediate motor pulse. The bridge still
        # executes travel in its bounded increments inside approved limits.
        node.setup_ready()
        if approved_setup_range(node.head) != expected:
            raise SetupStateChanged('Camera range changed before checking an endpoint')
        node.setup_move(high if goal == 'lower' else low)
        retries = 0
        while True:
            node.setup_ready()
            if approved_setup_range(node.head) != expected:
                raise SetupStateChanged('Camera reference or bounds changed during setup')
            if len(samples) >= maximum_observations:
                raise SetupUnavailable('Useful-view search reached its observation budget')
            result = node.setup_observe()
            node.setup_ready()
            if approved_setup_range(node.head) != expected:
                raise SetupStateChanged('Camera reference or bounds changed during observation')
            position = result.get('position')
            tolerance = setup_tolerance(node.head)
            if (result.get('reference_id') != reference or type(position) is not int
                    or not low - tolerance <= position <= high + tolerance):
                raise SetupStateChanged('Image is not bound to a stopped pose inside the range')
            # Keep the measured pose in evidence; named targets remain strictly
            # commandable when settling lands a few counts beside an endpoint.
            position = min(high, max(low, position))
            decision = result.get('decisions', {}).get(goal, {})
            action = decision.get('action')
            samples.append(dict(goal=goal, **result))
            if action in ('observe_again', 'recover_visibility'):
                retries += 1
                if retries > 1:
                    raise SetupUnavailable('View remains unreadable or ambiguous; saved range retained')
                continue
            retries = 0
            if action == 'candidate_view':
                if goal == 'lower' or chosen['lower'] - position >= 10:
                    chosen[goal] = position
                    break
                # Two distinct poses are needed; a broad upper classification
                # may also apply near the lower view. Explore within bounds.
                action = 'look_up'
            if action not in ('look_up', 'look_down'):
                raise SetupUnavailable('Cloud returned an unsupported setup decision')
            target = min(high, max(low, position + (-5 if action == 'look_up' else 5)))
            key = (goal, position, target)
            if abs(target - position) <= tolerance or key in visited:
                raise SetupUnavailable('No further useful view inside the saved range')
            visited.add(key)
            node.setup_move(target)
    return dict(down_position=chosen['lower'], up_position=chosen['upper'],
                forward_position=chosen['lower'],
                reference_id=reference)


def run_visual_setup(node, report):
    node.setup_ready()
    expected = approved_setup_range(node.head)
    report.update(calibration_method='cloud_useful_views', mechanical_homing_attempted=False,
                  physical_limits_changed=False, route_verified=False)
    try:
        calibration = select_setup_views(node, report)
        node.setup_move(calibration['forward_position'])
        node.setup_ready()
        if approved_setup_range(node.head) != expected:
            raise SetupStateChanged('Camera range changed before saving useful views')
        node.setup_commit(calibration)
        report.update(outcome='calibrated', calibration=calibration,
                      final_position=node.head['position'], visual_setup_outcome='selected')
    except SetupUnavailable as exc:
        # Cloud trouble is not a reason to destroy a working saved range.
        node.setup_ready()
        if approved_setup_range(node.head) != expected:
            raise SetupStateChanged('Saved range changed during visual setup')
        node.setup_retain()
        report.update(outcome='calibrated', calibration_method='operator_limits',
                      visual_setup_outcome='deferred', visual_setup_reason=str(exc),
                      final_position=node.head['position'], calibration={key: node.head[key]
                          for key in ('down_position', 'forward_position', 'up_position')})
    return report
