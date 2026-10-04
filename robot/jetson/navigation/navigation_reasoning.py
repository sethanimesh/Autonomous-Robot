"""Validated scene hints plus deterministic rules; no network or motor access."""
from dataclasses import asdict
import math

try:
    from robot.jetson.navigation.local_planner import LocalRoutePlanner, RouteCandidate
except ImportError:
    from local_planner import LocalRoutePlanner, RouteCandidate

HEADINGS = {'left': -30, 'center': 0, 'right': 30}


def enum(*values):
    return dict(type='string', enum=list(values))


def record(fields):
    return dict(type='object', additionalProperties=False, properties=fields, required=list(fields))


QUALITY = enum('clear', 'dark', 'blurred', 'lens_obstructed', 'unknown')
EVIDENCE = dict(type='string', minLength=1, maxLength=350)
CORRIDOR_SCHEMA = record(dict(
    id=enum(*HEADINGS), visibility=enum('visible', 'occluded', 'unknown'),
    hazard=enum('none', 'solid_object', 'thin_object', 'person', 'drop_or_step', 'unknown'),
    evidence=EVIDENCE))
ROUTE_SCHEMA = record(dict(
    quality=QUALITY, corridors=dict(type='array', minItems=3, maxItems=3, items=CORRIDOR_SCHEMA),
    preference=dict(type='array', minItems=3, maxItems=3, items=enum(*HEADINGS)),
    near_turn_space=enum('visible_clear', 'blocked', 'unknown'), evidence=EVIDENCE))
OCCLUSION_SCHEMA = record(dict(
    quality=QUALITY,
    cause=enum('partial_person', 'behind_object', 'left_frame', 'camera_changed', 'not_visible', 'unknown'),
    inspect_side=enum('left', 'right', 'none', 'unknown'), evidence=EVIDENCE))


def validate_record(value, schema):
    """Small strict validator for our closed schema, including types and lengths."""
    kind = schema['type']
    if kind == 'object':
        if not isinstance(value, dict) or set(value) != set(schema['properties']):
            raise ValueError('Unexpected navigation interpretation fields')
        for key, child in schema['properties'].items():
            validate_record(value[key], child)
    elif kind == 'array':
        if not isinstance(value, list) or not schema['minItems'] <= len(value) <= schema['maxItems']:
            raise ValueError('Invalid navigation interpretation list')
        for item in value:
            validate_record(item, schema['items'])
    elif kind == 'string':
        if not isinstance(value, str) or not value.strip():
            raise ValueError('Missing navigation interpretation text')
        if 'enum' in schema and value not in schema['enum']:
            raise ValueError('Invalid navigation interpretation classification')
        if len(value) > schema.get('maxLength', 350):
            raise ValueError('Navigation interpretation text is oversized')
    return value


def validate_route_advice(value):
    validate_record(value, ROUTE_SCHEMA)
    if (set(value['preference']) != set(HEADINGS)
            or {c['id'] for c in value['corridors']} != set(HEADINGS)):
        raise ValueError('Each candidate corridor must appear exactly once')
    return value


def validate_occlusion_advice(value):
    return validate_record(value, OCCLUSION_SCHEMA)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def local_candidates(result, planner):
    """Keep measured candidates only; Gemini cannot manufacture clearance."""
    viable = {}
    if not isinstance(result, dict) or result.get('ok') is not True:
        return viable
    if not finite(result.get('floor_horizon_y')) or not 0 <= result['floor_horizon_y'] <= 1:
        return viable
    candidates, evidence = result.get('candidates'), result.get('evidence')
    if not isinstance(candidates, list) or not isinstance(evidence, list):
        return viable
    for name, heading in HEADINGS.items():
        rows = [c for c in candidates if isinstance(c, dict) and c.get('heading_degrees') == heading]
        pixels = [e for e in evidence if isinstance(e, dict) and e.get('heading_degrees') == heading]
        if len(rows) != 1 or len(pixels) != 1:
            continue
        e, c = pixels[0], rows[0]
        if (not all(finite(e.get(k)) for k in ('floor_fraction', 'known_fraction', 'sample_count'))
                or not .95 <= e['floor_fraction'] <= 1 or not .75 <= e['known_fraction'] <= 1
                or type(e['sample_count']) is not int or e['sample_count'] <= 0):
            continue
        fields = ('heading_degrees', 'clear_distance_m', 'confidence', 'known_fraction')
        if (not all(finite(c.get(k)) for k in fields)
                or not 0 <= c['confidence'] <= 1 or not 0 <= c['known_fraction'] <= 1):
            continue
        decision = planner.choose([RouteCandidate(**{k: c[k] for k in fields})])
        if not decision.blocked:
            viable[name] = decision
    return viable


def fuse_route_evidence(initial, fresh, advice, planner=None):
    """Intersect two local checks with semantic vetoes; rank only survivors.

    Distances are existing conservative image heuristics, not Gemini estimates.
    A small preference bonus breaks comparable-route ties, never clears a veto.
    """
    planner = planner or LocalRoutePlanner(maximum_step_m=.05)
    blocked = dict(blocked=True, heading_degrees=0., distance_m=0., score=None,
                   reason='no route passes local and semantic checks')
    try:
        validate_route_advice(advice)
    except (ValueError, TypeError):
        return dict(decision=dict(blocked, reason='invalid Gemini route interpretation'),
                    approved_headings=[], exclusions={})
    before, after = local_candidates(initial, planner), local_candidates(fresh, planner)
    exclusions, options = {}, []
    for corridor in advice['corridors']:
        name = corridor['id']
        if advice['quality'] != 'clear':
            reason = 'unusable camera view: ' + advice['quality']
        elif name not in before or name not in after:
            reason = 'local floor evidence is blocked or uncertain'
        elif corridor['visibility'] != 'visible':
            reason = 'floor is ' + corridor['visibility']
        elif corridor['hazard'] != 'none':
            reason = 'semantic hazard: ' + corridor['hazard']
        elif name != 'center' and advice['near_turn_space'] != 'visible_clear':
            reason = 'nearby turn space is not visible and clear'
        else:
            candidate = asdict(after[name])
            candidate['distance_m'] = min(before[name].distance_m, after[name].distance_m)
            candidate['score'] += .06 * (2 - advice['preference'].index(name))
            candidate['reason'] = 'fresh local floor checks with Gemini hazard vetoes and route preference'
            options.append(candidate)
            continue
        exclusions[name] = reason
    return dict(decision=max(options, key=lambda c: c['score']) if options else blocked,
                approved_headings=sorted(c['heading_degrees'] for c in options), exclusions=exclusions)


def occlusion_recovery(advice, previous_position, current_position, *, memory_age,
                       same_reference, face_visible=False, attempts=0):
    """Missing-person hypotheses select observations, never translation or identity."""
    result = dict(action='rescan', reason='no reliable recent person context', drive_allowed=False)
    if face_visible:
        return dict(result, action='hold_for_identity', reason='fresh local face takes priority')
    try:
        validate_occlusion_advice(advice)
    except (ValueError, TypeError):
        return result
    if advice['quality'] != 'clear':
        return dict(result, action='stop', reason='unusable camera view: ' + advice['quality'])
    if (not same_reference or not finite(memory_age) or not 0 <= memory_age <= 30
            or not finite(previous_position) or not finite(current_position) or attempts >= 2):
        return result
    if abs(previous_position - current_position) > 3:
        return dict(result, action='restore_previous_view', position=previous_position,
                    reason='person disappeared after a measured camera adjustment')
    if advice['cause'] in ('partial_person', 'behind_object'):
        return dict(result, action='wait_then_rescan', wait_seconds=1.2,
                    inspect_side=advice['inspect_side'], reason=advice['cause'])
    return dict(result, reason=advice['cause'], inspect_side=advice['inspect_side'])
