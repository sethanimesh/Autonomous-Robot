"""Small shared policy for resuming ordinary mission interruptions."""


def recovery_kind(report):
    if report.get('cable_heading_known') is False or report.get('stop_error'):
        return None
    error = str(report.get('error', '')).lower()
    # Repeating a command cannot repair a lost physical reference or a bad motor.
    if any(term in error for term in ('reference changed', 'physical boundary',
                                     'wrong direction', 'opposite', 'backwards', 'veered',
                                     'not homed', 'restore cable', 'outside saved')):
        return None
    if any(term in error for term in ('blocked', 'not proven clear', 'no proven', 'no clear')):
        return 'route'
    if any(term in error for term in ('target lost', 'person lost', 'target disappeared',
                                     'could not be centered')):
        return 'identity'
    if any(term in error for term in ('stale', 'timeout', 'timed out', 'unavailable',
                                     'no progress', 'no fresh', 'no new camera',
                                     'http', 'connection', 'not streaming', 'too dark',
                                     'differs too far from its request',
                                     'view is dark', 'did not recover',
                                     'did not receive a new image')):
        return 'feedback'
    return None


def motor_references(status):
    motors = (status or {}).get('motors', {})
    return tuple(motors.get(name, {}).get('generation') for name in ('left', 'right'))


def same_motor_references(before, after):
    left, right = motor_references(before), motor_references(after)
    return all(left) and left == right
