"""Independent identity and metric arrival policy for family missions."""
import math

# These are camera-model thresholds, not verified physical stopping gaps.
# The owner prefers closer approximate pursuit over waiting for metric setup.
APPROXIMATE_STOP_ESTIMATE_M = .20
APPROXIMATE_CLOSE_ESTIMATE_M = .12
APPROXIMATE_SHORT_STEP_ESTIMATE_M = .30


def wardrobe_view_target(guidance,head):
    """Normalized visual advice chooses a bounded head view, never motor pressure."""
    if not isinstance(guidance,dict) or not 0<=guidance.get('age_seconds',999)<=1:
        return None
    direction=guidance.get('next_view')
    if direction not in ('raise','lower'):return None
    values=[head.get(k) for k in ('position','down_position','up_position')]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):return None
    current,lower,upper=values
    destination=upper if direction=='raise' else lower
    return round(current+max(-10,min(10,destination-current)))


def identity_ready(observation,maximum_age=.75):
    if not isinstance(observation,dict):return False
    age=observation.get('age_seconds',999.)
    fresh=isinstance(age,(int,float)) and not isinstance(age,bool) and math.isfinite(age) and 0<=age<=maximum_age
    return bool(fresh and observation.get('identity_confirmed') is True and
                observation.get('track_id') and observation.get('profile_id') and
                observation.get('clothing_approach_enabled') is True)


def approach_decision(observation):
    """No range means look again, not lost identity or fabricated arrival."""
    if not identity_ready(observation,3.):return dict(action='reacquire',distance_m=0.)
    r=observation.get('range') or {}
    if any(r.get(k) is not None and r[k]!=observation.get(k) for k in ('track_id','profile_id')):
        return dict(action='inspect',distance_m=0.,reason='Distance belongs to another person track')
    age=r.get('age_seconds',0.)
    if type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<=3:
        return dict(action='inspect',distance_m=0.,reason='Person located; refreshing distance')
    numbers=[r.get(k) for k in ('distance_m','lower_m','upper_m')]
    approximate=(observation.get('approximate_approach_enabled') is True
                 and r.get('validated') is False and r.get('mode')=='approximate'
                 and r.get('source')=='metric_depth_floor_estimate')
    if approximate and any(r.get(k)!=observation.get(k) for k in ('track_id','profile_id')):
        return dict(action='inspect',distance_m=0.,reason='Distance needs a current person association')
    if approximate and r.get('available') is False:
        return dict(action='look_lower',distance_m=0.,reason='Person located; look lower to estimate the floor')
    if (r.get('validated') is not True and not approximate) or any(type(v) not in (float,int) or not math.isfinite(v) for v in numbers):
        return dict(action='inspect',distance_m=0.,reason='Person located; distance unavailable')
    distance,lower,upper=numbers
    if not 0<=lower<=distance<=upper:return dict(action='inspect',distance_m=0.,reason='Invalid distance interval')
    if approximate:
        if r.get('distance_reference')!='camera_ground_projection':
            return dict(action='inspect',distance_m=0.,reason='Unknown estimated distance reference')
        if type(r.get('consistent_samples')) is not int or r['consistent_samples']<2:
            return dict(action='inspect',distance_m=0.,reason='Person located; checking another distance estimate')
        if distance<APPROXIMATE_CLOSE_ESTIMATE_M:
            return dict(action='close',distance_m=0.,reason='Stopped: person appears close; distance is approximate')
        geometry=r.get('geometry') or {}
        fraction=geometry.get('head_fraction') if isinstance(geometry,dict) else None
        lower_inspected=(r.get('pose_source')=='current_floor_estimate'
                         and type(fraction) in (int,float) and math.isfinite(fraction)
                         and 0<=fraction<=.25)
        uncertain_feet=not r.get('feet_checked')
        if distance<=1. and uncertain_feet and not lower_inspected:
            return dict(action='look_lower',distance_m=0.,reason='Inspect for feet closer than the visible torso')
        if distance<=APPROXIMATE_STOP_ESTIMATE_M:
            return dict(action='arrived_estimate',distance_m=0.,
                        reason='Stopped near the person using estimated camera distance; front gap is unverified')
        step=.02 if distance<=APPROXIMATE_SHORT_STEP_ESTIMATE_M or (lower_inspected and uncertain_feet) else .05
        return dict(action='approach',distance_m=step,estimated=True)
    if lower<=.7 and not r.get('feet_checked'):
        return dict(action='look_lower',distance_m=0.,reason='Check for closer feet')
    if .5<=lower and upper<=.7 and r.get('consistent_samples',0)>=2:
        return dict(action='arrived',distance_m=0.)
    if lower<.5:return dict(action='close',distance_m=0.,reason='Person is already close')
    step=min(.1,max(0.,lower-.6))
    if step<.02:return dict(action='inspect',distance_m=0.,reason='Refine distance before moving closer')
    return dict(action='approach',distance_m=step)
