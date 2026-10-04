"""Scene hints bound a local camera investigation, never identify or drive."""


def search_scene_action(result, face_visible=False):
    # A real face takes priority over a model's guess about household geometry.
    if face_visible:
        return 'keep_person'
    observation = (result or {}).get('observations') or []
    if len(observation) != 1 or observation[0].get('quality') != 'clear':
        return 'use_detector_rules'
    if (observation[0].get('human_visible') == 'yes'
            and any(part != 'unclear' for part in observation[0].get('visible_parts', []))):
        return 'keep_person'
    if observation[0].get('scene') == 'ceiling_only':
        return 'return_from_ceiling'
    return 'use_detector_rules'


def upward_budget_reached(origin, current, face_visible=False, maximum_raise=24):
    """Three eight-count probes without a face are enough at one heading."""
    return not face_visible and origin - current >= maximum_raise


def human_framing_plan(result, head, face_visible=False):
    """Translate a fresh body-part observation to one bounded head step."""
    if face_visible:
        return None
    observations = (result or {}).get('observations') or []
    if len(observations) != 1:
        return None
    o = observations[0]
    if (o.get('quality') != 'clear' or o.get('human_visible') != 'yes'
            or not o.get('visible_parts')):
        return None
    hint = o.get('framing_hint')
    if hint not in ('raise', 'lower', 'hold'):
        return None
    target = int(head['position']) + (-8 if hint == 'raise' else 8 if hint == 'lower' else 0)
    target = max(int(head['minimum_target_position']), min(int(head['maximum_target_position']), target))
    return dict(scenario='gemini_' + hint, positions=[] if hint == 'hold' else [target],
                center_person=False, visible_parts=o['visible_parts'])


def combine_framing_plans(cloud, local, position, face_visible=False):
    """Evidence priorities, not calibrated probabilities: Gemini3/body1/face4."""
    def action(plan):
        if plan.get('center_person'):
            return 'center'
        positions = plan.get('positions') or []
        if not positions or positions[0] == position:
            return 'hold'
        return 'raise' if positions[0] < position else 'lower'

    candidates = []
    if cloud:
        candidates.append(('gemini', cloud, 3))
    if local:
        candidates.append(('face_detector' if face_visible else 'person_detector', local,
                           4 if face_visible else 1))
    if not candidates:
        return dict(scenario='no_framing_evidence', positions=[], center_person=False)
    scores = {}
    votes = []
    for source, plan, weight in candidates:
        direction = action(plan)
        scores[direction] = scores.get(direction, 0) + weight
        votes.append(dict(source=source, direction=direction, weight=weight))
    selected = max(candidates, key=lambda item: (scores[action(item[1])], item[2]))
    return dict(selected[1], framing_votes=votes, framing_scores=scores,
                selected_source=selected[0], sources_agree=len(scores) == 1 and len(votes) > 1)
