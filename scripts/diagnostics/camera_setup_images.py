#!/usr/bin/env python3
"""Collect manual camera placements and evaluate Gemini advice; never move motors."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from robot.mac.camera_setup_advisor import setup_view_decision
from robot.mac.camera_setup_pool import GeminiCameraSetupAdvisor


def capture(label, directory, console_url):
    directory = Path(directory)
    image_path = directory / (label + '.jpg')
    report_path = directory / (label + '.json')
    if image_path.exists() or report_path.exists():
        raise ValueError('This placement already exists; use a new label')
    started = time.time()
    with urlopen(console_url.rstrip('/') + '/api/status', timeout=5) as response:
        status = json.load(response)
    if status.get('camera_ready') is not True:
        raise ValueError('The webcam feed is not ready')
    if status.get('mission', {}).get('running') or status.get('manual_drive', {}).get('active'):
        raise ValueError('Finish active robot movement before collecting manual placement images')
    head = status.get('camera_head') or {}
    if head.get('moving') or head.get('homing'):
        raise ValueError('Let the camera stop before capturing')
    with urlopen(console_url.rstrip('/') + '/snapshot.jpg', timeout=5) as response:
        payload = response.read(4_000_001)
    if not 100 <= len(payload) <= 4_000_000 or not payload.startswith(b'\xff\xd8'):
        raise ValueError('The snapshot is not a supported JPEG image')
    directory.mkdir(parents=True, exist_ok=True)
    image_path.write_bytes(payload)
    report = dict(label=label, path=str(image_path.resolve()), frame_sha256=hashlib.sha256(payload).hexdigest(),
        captured_at_unix=time.time(), request_started_at_unix=started,
        capture_source=console_url, purpose='manual_placement_image_test',
        camera_ready=status['camera_ready'], robot_ready=status.get('robot_ready'),
        encoder_bound=False, calibration_applied=False,
        reported_head={key:head.get(key) for key in ('available','position','reference_id')})
    # The UI snapshot has no source timestamp or stopped-pose binding. It is
    # useful test imagery, never sufficient evidence to commit motor limits.
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    return report


def evaluate(paths, report_path, advisor=None):
    paths = [Path(path) for path in paths]
    result = (advisor or GeminiCameraSetupAdvisor()).interpret(
        [path.read_bytes() for path in paths])
    result['image_tests'] = [dict(path=str(path.resolve()), observation=observation,
        lower_decision=setup_view_decision(observation,'lower'),
        upper_decision=setup_view_decision(observation,'upper'))
        for path, observation in zip(paths, result['observations'])]
    result.update(purpose='manual_placement_image_test', calibration_applied=False)
    report_path = Path(report_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    cap=commands.add_parser('capture')
    cap.add_argument('--label', required=True)
    cap.add_argument('--directory', type=Path, required=True)
    cap.add_argument('--console-url',default='http://192.168.1.48:8080')
    assess=commands.add_parser('evaluate')
    assess.add_argument('images',nargs='+',type=Path)
    assess.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='capture':
        if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_-' for c in args.label):
            parser.error('Use a simple lowercase placement label')
        result=capture(args.label,args.directory,args.console_url)
    else:
        if not 1<=len(args.images)<=3:parser.error('Evaluate one to three images per Gemini request')
        result=evaluate(args.images,args.report)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
