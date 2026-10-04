"""Bind a diagnostic image to stopped, fresh hardware feedback; no commands."""
import copy
import math


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def capture_pose(head, robot, head_age, robot_age):
    if any(not finite_number(age) or not 0 <= age <= 1.5 for age in (head_age, robot_age)):
        return None
    if not isinstance(head, dict) or not isinstance(robot, dict):
        return None
    if (not head.get('reference_id') or head['reference_id'] != robot.get('tool_reference_id')
            or not head.get('homed') or head.get('moving') or head.get('homing')
            or robot.get('status') != 'ok' or robot.get('motion_active')
            or not finite_number(head.get('position'))):
        return None
    motors = {}
    reported_motors = robot.get('motors')
    if not isinstance(reported_motors, dict):
        return None
    for side in ('left', 'right', 'tool'):
        motor = reported_motors.get(side, {})
        if not isinstance(motor, dict):
            return None
        if (not motor.get('generation') or not finite_number(motor.get('position'))
                or any(not finite_number(motor.get(k)) or abs(motor[k]) > 1
                       for k in ('speed', 'commanded_speed'))
                or not isinstance(motor.get('state', []), (list, tuple))
                or motor.get('moving') or 'running' in motor.get('state', [])):
            return None
        motors[side] = {k: motor[k] for k in ('generation', 'position')}
    if abs(motors['tool']['position'] - head['position']) > 1:
        return None
    return dict(head=copy.deepcopy(head), motors=motors)


def same_capture_pose(before, after):
    if not before or not after:
        return False
    if (before['head']['reference_id'] != after['head']['reference_id']
            or abs(before['head']['position'] - after['head']['position']) > 1):
        return False
    return all(before['motors'][side]['generation'] == after['motors'][side]['generation']
               and abs(before['motors'][side]['position'] - after['motors'][side]['position']) <= 1
               for side in ('left', 'right', 'tool'))
