#!/usr/bin/env python3
"""Observe a combined family mission; never send commands or save images."""
import argparse
import json
import math
from pathlib import Path
import time


def summarize(samples, profile_id, mission=None):
    fresh = [s for s in samples if s.get('profile_id') == profile_id
             and s.get('identity_confirmed') is True and s.get('track_id')
             and type(s.get('age_seconds')) in (int, float)
             and math.isfinite(s['age_seconds']) and 0 <= s['age_seconds'] <= 1]
    face = [s for s in fresh if s.get('identity_source') == 'face']
    # A fresh local appearance association and a saved-outfit comparison are
    # different evidence. Neither is renamed to a face confirmation.
    continuity = [s for s in fresh if s.get('identity_source') == 'tracking']
    wardrobe = [s for s in fresh if s.get('identity_source') == 'clothing']
    face_epoch = min((s['motion_epoch'] for s in face), default=math.inf)
    after_motion = [s for s in continuity + wardrobe if s['motion_epoch'] > face_epoch]
    bound_ranges = [s for s in fresh if isinstance(s.get('range'), dict)
              and type(s['range'].get('age_seconds')) in (int, float)
              and 0 <= s['range']['age_seconds'] <= 3
              and s['range'].get('track_id') == s['track_id']
              and s['range'].get('profile_id') == profile_id]
    ranges = [s for s in bound_ranges if s['range'].get('validated') is True]
    approximate = [s for s in bound_ranges if s['range'].get('validated') is False
                   and s['range'].get('available') is True
                   and s['range'].get('mode') == 'approximate'
                   and s['range'].get('source') == 'metric_depth_floor_estimate'
                   and s['range'].get('distance_reference') == 'camera_ground_projection'
                   and type(s['range'].get('distance_m')) in (int, float)
                   and math.isfinite(s['range']['distance_m']) and s['range']['distance_m'] >= 0]
    mission = mission or {}
    moves = [step['report'] for step in mission.get('steps', [])
             if step.get('kind') == 'target_approach' and step.get('report', {}).get('drive_started')]
    return dict(profile_id=profile_id, face_observations=len(face),
                local_clothing_continuity_observations=len(continuity),
                saved_outfit_match_observations=len(wardrobe),
                face_free_observations_after_motion=len(after_motion),
                validated_range_observations=len(ranges),
                approximate_range_observations=len(approximate),
                approach_steps=len(moves),
                encoder_travelled_m=sum(m.get('travelled_m', 0.) for m in moves),
                mission_outcome=mission.get('outcome'),
                distance_check=('available' if ranges else
                    'calibrated camera distance confirmed at standoff' if approximate else
                    'unavailable; no validated 60 cm result'),
                note='Observed evidence only; one family session does not establish confusion or distance accuracy.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile-id', required=True)
    parser.add_argument('--seconds', type=float, default=600.)
    parser.add_argument('--mission-report', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 1800:
        parser.error('duration must be 1–1800 seconds')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    samples = []
    motion = {'robot': False, 'head': False, 'epoch': 0}
    last = [None, 0.]
    rclpy.init()
    node = Node('echora_combined_family_observer')

    def on_motion(kind, message):
        try:
            value = json.loads(message.data)
            active = (bool(value.get('moving') or value.get('homing')) if kind == 'head'
                      else any(abs(float(value.get('motors', {}).get(side, {}).get(k, 0))) > 1
                               for side in ('left', 'right') for k in ('speed', 'commanded_speed')))
            if active and not any(motion[k] for k in ('robot', 'head')):
                motion['epoch'] += 1
            motion[kind] = active
        except (ValueError, TypeError, AttributeError):
            pass

    def on_track(message):
        try:
            value = json.loads(message.data)
            signature = (value.get('profile_id'), value.get('track_id'), value.get('identity_source'), motion['epoch'])
            if signature == last[0] and time.monotonic() - last[1] < .5:
                return
            sample = {k: value.get(k) for k in ('profile_id', 'target_revision', 'track_id', 'frame_key',
                'identity_confirmed', 'identity_source', 'age_seconds', 'range', 'visible_regions',
                'clothing_approach_enabled', 'approximate_approach_enabled', 'wardrobe_error')}
            sample.update(observed_at_unix=time.time(), motion_epoch=motion['epoch'])
            samples.append(sample)
            last[:] = [signature, time.monotonic()]
        except (ValueError, TypeError, AttributeError):
            pass

    node.create_subscription(String, '/perception/people_tracks', on_track, 10)
    node.create_subscription(String, '/robot_status', lambda m: on_motion('robot', m), 10)
    node.create_subscription(String, '/camera_head/status', lambda m: on_motion('head', m), 10)
    mission = None
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            try:
                mission = json.loads(Path(args.mission_report).read_text())
                if mission.get('finished_at_unix'):
                    break
            except (OSError, ValueError):
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report = dict(summary=summarize(samples, args.profile_id, mission), samples=samples,
                      images_saved=False, commands_sent=False)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':main()
