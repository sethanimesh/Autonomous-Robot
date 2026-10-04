"""Head-only hardware check: bounded targets, fresh telemetry, visual hold proof."""
import argparse
import json
import time
import urllib.request
from pathlib import Path


def main():
    import rclpy
    import cv2
    import numpy as np
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from camera_control_lease import camera_control_lease
    from camera_motion_feedback import frame_fingerprint, verify_camera_step

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets', type=int, nargs='+', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--scene-url')
    parser.add_argument('--settle-seconds', type=float, default=5.0)
    args = parser.parse_args()
    report = {'outcome': 'failure', 'steps': [], 'started_at': time.time()}
    output = Path(args.report)
    output.parent.mkdir(parents=True, exist_ok=True)
    with camera_control_lease(exclusive=True):
        rclpy.init()
        node = Node('head_movement_check')
        state = {}
        def record(key, value):
            state[key] = value
            state[key + '_at'] = time.monotonic()
        node.create_subscription(String, '/camera_head/status', lambda m: record('head', json.loads(m.data)), 10)
        node.create_subscription(String, '/robot_status', lambda m: record('robot', json.loads(m.data)), 10)
        def frame(m):
            record('frame', frame_fingerprint(m.data, m.width, m.height, m.step, m.encoding))
            state['raw'] = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height,m.step)[:, :m.width*3].reshape(m.height,m.width,3).copy()
        node.create_subscription(Image, '/camera/image_raw', frame, qos_profile_sensor_data)
        pub = node.create_publisher(String, '/camera_head/command', 1)
        def command(value):
            msg = String()
            msg.data = json.dumps(value)
            pub.publish(msg)
        def spin(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        reference = None
        baseline = None
        def check():
            now = time.monotonic()
            for key in ('head', 'robot', 'frame'):
                if key not in state or now - state[key + '_at'] > 1.25:
                    raise RuntimeError('Missing or stale ' + key)
            h, r = state['head'], state['robot']
            if (not h.get('homed') or h.get('homing') or not h.get('require_approved_reference')
                    or h.get('reference_id') != h.get('approved_reference_id')
                    or (reference and h.get('reference_id') != reference)):
                raise RuntimeError('Physical camera reference is unavailable or changed')
            if not h['minimum_position'] <= h['position'] <= h['maximum_position']:
                raise RuntimeError('Head left its physical bounds')
            tracks = [r['motors'][name] for name in ('left', 'right')]
            positions = [v['position'] for v in tracks]
            if any(v['speed'] != 0 for v in tracks) or (baseline is not None and positions != baseline):
                raise RuntimeError('Tracks did not remain stationary')
            return h, positions
        def scene():
            from camera_head_calibration import scene_sample
            results = []
            for _ in range(2):
                spin(.2)
                h, _ = check()
                started = time.monotonic()
                result = json.load(urllib.request.urlopen(args.scene_url, timeout=8))
                scene_sample(result, h['position'], reference, time.monotonic()-started)
                results.append(result)
            return results
        def snapshot(label):
            path = output.with_name(output.stem + '-' + label + '.jpg')
            path.write_bytes(urllib.request.urlopen('http://127.0.0.1:8080/snapshot.jpg', timeout=3).read())
            return str(path)
        try:
            discovery_deadline = time.monotonic() + 12
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if all(key in state for key in ("head", "robot", "frame")) and pub.get_subscription_count():
                    break
            h, baseline = check()
            reference = h['reference_id']
            if h.get('moving'):
                raise RuntimeError('Head was already moving')
            report['initial_head'] = dict(h)
            report['initial_image'] = snapshot('before')
            if args.scene_url:
                report['initial_scene'] = scene()
            for index, target in enumerate(args.targets):
                h, _ = check()
                origin = h['position']
                if not (h.get('minimum_target_position', h['minimum_position']) <= target <= h['maximum_target_position'] and 0 < abs(target-origin) <= 15):
                    raise RuntimeError('Requested target exceeds a bounded step or physical margin')
                before = state['frame']
                step = {'origin': origin, 'target': target, 'telemetry': []}
                report['steps'].append(step)
                sent = time.monotonic()
                command({'action': 'jog_to', 'target': target})
                last = 0
                acknowledged = False
                stopped_since = None
                while time.monotonic() - sent < 12:
                    rclpy.spin_once(node, timeout_sec=.05)
                    h, _ = check()
                    if state['head_at'] > last:
                        last = state['head_at']
                        step['telemetry'].append(dict(h))
                    acknowledged = acknowledged or (state['head_at'] > sent and h.get('target_position') == target)
                    if acknowledged and not h.get('moving'):
                        if stopped_since is None:
                            stopped_since = time.monotonic()
                        if time.monotonic() - stopped_since >= args.settle_seconds:
                            break
                    else:
                        stopped_since = None
                else:
                    raise RuntimeError('No acknowledged settled movement before timeout')
                step['actual'] = h['position']
                step['image'] = snapshot(str(index))
                if (h['position'] - origin) * (1 if target > origin else -1) < 3:
                    raise RuntimeError('Head made insufficient progress; no new retry cycle issued')
                after = state['frame']
                raw_after = state['raw'].copy()
                cv2.imwrite(str(output.with_name(output.stem + '-' + str(index) + '-held-a.jpg')), raw_after)
                spin(1)
                h, _ = check()
                cv2.imwrite(str(output.with_name(output.stem + '-' + str(index) + '-held-b.jpg')), state['raw'])
                step['visual'] = verify_camera_step(before, after, state['frame'])
                if abs(h['position'] - step['actual']) > 2:
                    raise RuntimeError('Encoder drift after settling')
                if args.scene_url:
                    step['scene'] = scene()
                step['outcome'] = 'passed'
                print(json.dumps({k: step[k] for k in ('origin', 'target', 'actual', 'outcome')}), flush=True)
            report['outcome'] = 'passed'
        except Exception as exc:
            report['error'] = str(exc)
            if getattr(exc, 'report', None):
                report['visual_failure'] = exc.report
            print('STOPPED: ' + str(exc), flush=True)
        finally:
            command({'action': 'stop'})
            spin(.75)
            report['final_head'] = state.get('head')
            report['final_robot'] = state.get('robot')
            report['finished_at'] = time.time()
            output.write_text(json.dumps(report, indent=2) + '\n')
            node.destroy_node()
            rclpy.shutdown()
    return 0 if report['outcome'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
