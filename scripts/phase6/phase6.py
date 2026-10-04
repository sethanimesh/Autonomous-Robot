#!/usr/bin/env python3
"""Phase 6 package, readiness, restart and supervised live-run commands.

No network or robot access occurs at import, in `bundle`, or in `install`
without --restart. Existing calibration and enrollment are preserved; optional
profiles merge only their tuning fields into the current runtime configuration.
"""
import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time

SERVICES = ('echora-camera', 'echora-person-detector', 'echora-face-detector',
            'echora-target-recognizer', 'echora-target-observer', 'echora-bridge', 'echora-enrollment-console')
PROFILE = {'echora_face_detector': {'confidence_threshold': .60},
           'echora_target_recognizer': {'match_threshold': .40, 'confirmation_required': 2}}


def bundle(destination):
    root = Path(__file__).resolve().parents[2]
    files = {}
    for part in ('camera', 'ev3_bridge', 'mission', 'navigation', 'perception'):
        for path in (root/'robot/jetson'/part).glob('*.py'):
            if path.name == '__init__.py':
                continue
            if path.name in files:
                raise ValueError('Ambiguous runtime filename: ' + path.name)
            compile(path.read_text(), str(path), 'exec')
            files[path.name] = path.read_bytes()
    for part, name in (('camera','camera_transport.xml'),('ev3_bridge','control_transport.xml'),
                       ('perception','perception_transport.xml')):
        files[name] = (root/'robot/jetson'/part/name).read_bytes()
    files['phase6.py'] = Path(__file__).read_bytes()
    manifest = {name: hashlib.sha256(data).hexdigest() for name,data in files.items()}
    files['manifest.json'] = json.dumps(manifest,indent=2).encode()
    with tarfile.open(destination,'w:gz') as archive:
        for name,data in files.items():
            info=tarfile.TarInfo(name);info.size=len(data);info.mode=0o644
            archive.addfile(info,io.BytesIO(data))
    print('Bundle ready:',destination)


def read_bundle(path):
    with tarfile.open(path,'r:gz') as archive:
        files={}
        for item in archive.getmembers():
            if not item.isfile() or Path(item.name).name != item.name or item.name in files:
                raise ValueError('Invalid or duplicate bundle entry')
            files[item.name]=archive.extractfile(item).read()
    manifest=json.loads(files.pop('manifest.json'))
    if set(manifest)!=set(files): raise ValueError('Bundle manifest differs from contents')
    for name,data in files.items():
        if hashlib.sha256(data).hexdigest()!=manifest[name]: raise ValueError('Checksum mismatch: '+name)
        if name.endswith('.py'): compile(data,name,'exec')
    return files


def restart(services):
    subprocess.run(['sudo','-n','true'],check=True)
    subprocess.run(['systemctl','--user','stop','echora-face-recovery.service'],check=False,
                   stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run(['sudo','-n','systemctl','reset-failed',*services],check=True)
    subprocess.run(['sudo','-n','systemctl','restart',*services],check=True)


def install(args):
    files=read_bundle(args.archive)
    runtime=Path(args.runtime).resolve();runtime.mkdir(parents=True,exist_ok=True)
    if args.household_profile:
        import yaml
        path = runtime / 'perception.yaml'
        settings = yaml.safe_load(path.read_text())
        for node, values in PROFILE.items():
            settings.setdefault(node, {}).setdefault('ros__parameters', {}).update(values)
        files['perception.yaml'] = yaml.safe_dump(settings, sort_keys=False).encode()
    if getattr(args, 'faster_tracks', False):
        import yaml
        path = runtime / 'robot.yaml'
        settings = yaml.safe_load(path.read_text())
        settings['ev3_bridge']['ros__parameters']['max_motor_speed'] = 240
        files['robot.yaml'] = yaml.safe_dump(settings, sort_keys=False).encode()
    backup=runtime/'backups'/('phase6-'+time.strftime('%Y%m%d-%H%M%S'));backup.mkdir(parents=True)
    for name,data in files.items():
        target=runtime/name
        if target.exists(): shutil.copy2(target,backup/name)
        temporary=runtime/(name+'.new');temporary.write_bytes(data);temporary.replace(target)
    print('Installed. Backup:',backup)
    if args.restart: restart(SERVICES)


def readiness(samples, frames, with_robot=False, ages=None):
    problems = []
    warnings = []
    ages = ages or {}
    for role,field in (('person','inferences'),('face','detections_published'),('identity','matches_published')):
        values=samples.get(role,[])
        if (len(values)<2 or values[-1].get(field,0)<=values[0].get(field,0)
                or ages.get(role, 0.) > 6.):
            messages = warnings if role == 'person' else problems
            messages.append(role+' processing is not advancing')
    camera=(samples.get('camera') or [{}])[-1]
    if (frames<2 or camera.get('state')!='streaming'
            or ages.get('image', 0.) > 1.):
        problems.append('fresh camera frames unavailable')
    identity=(samples.get('identity') or [{}])[-1]
    if not identity.get('target_label'): problems.append('no target enrolled')
    observer = (samples.get('observer') or [{}])[-1]
    if (not observer.get('ok') or observer.get('target_label') != identity.get('target_label')
            or ages.get('observer', 999.) > 2.5):
        problems.append('waiting for mission target observations')
    if with_robot:
        robot=(samples.get('robot') or [{}])[-1];head=(samples.get('head') or [{}])[-1]
        if ages.get('robot', 0.) > 2.5 or ages.get('head', 0.) > 2.5:
            problems.append('waiting for fresh robot feedback')
        if not head.get('calibrated') or head.get('manual_override') or not head.get('homed'):
            problems.append('restore saved camera range for this boot')
        reference=head.get('reference_id')
        if not reference or reference!=head.get('approved_reference_id') or reference!=robot.get('tool_reference_id'):
            problems.append('camera reference unavailable or changed')
        for side in ('left','right','tool'):
            motor=robot.get('motors',{}).get(side,{})
            if motor.get('speed')!=0 or motor.get('commanded_speed')!=0:
                problems.append(side+' motor is not confirmed stopped')
            if not motor.get('generation') or not isinstance(motor.get('position'), (int, float)) or not math.isfinite(motor['position']):
                problems.append(side+' motor reference unavailable')
    return problems, warnings


def probe(seconds=8., with_robot=False):
    import rclpy
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy
    rclpy.init();node=rclpy.create_node('phase6_readiness')
    samples={};frames=set();received={}
    status_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    def receive(role,message):
        try: value=json.loads(message.data)
        except ValueError:return
        if isinstance(value,dict):
            samples.setdefault(role,[]).append(value)
            received[role] = time.monotonic()
    def receive_image(message):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        if key not in frames:
            frames.add(key)
            received['image'] = time.monotonic()
    for role,topic in (('camera','/camera/status'),('person','/perception/status'),
                       ('face','/perception/face_status'),('identity','/perception/recognition_status'),
                       ('observer', '/mission/target_observation'),
                       ('robot','/robot_status'),('head','/camera_head/status')):
        qos = 1 if role in ('robot', 'head', 'observer') else status_qos
        node.create_subscription(String,topic,lambda m,r=role:receive(r,m),qos)
    node.create_subscription(Image,'/camera/image_raw',receive_image,qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    finally:node.destroy_node();rclpy.shutdown()
    ages = {role: time.monotonic() - at for role, at in received.items()}
    problems, warnings=readiness(samples,len(frames),with_robot,ages)
    return dict(ok=not problems,problems=problems,warnings=warnings,frames=len(frames),ages=ages,
                status={k:v[-1] for k,v in samples.items()},captured_at_unix=time.time())


def start_heading(status, previous, geometry):
    robot=status['robot']
    if previous is None:return 0.
    baseline=previous['status']['robot']
    for side in ('left','right'):
        if not robot['motors'][side].get('generation') or robot['motors'][side]['generation']!=baseline['motors'][side].get('generation'):
            raise ValueError('Motor reference changed; confirm cable-neutral again')
    left=(robot['motors']['left']['position']-baseline['motors']['left']['position'])*geometry['left_sign']
    right=(robot['motors']['right']['position']-baseline['motors']['right']['position'])*geometry['right_sign']
    delta=(left-right)*geometry['wheel_radius_m']/geometry['track_width_m']*360/geometry['encoder_counts_per_rev']
    heading=previous['initial_heading_degrees']+delta
    if not math.isfinite(heading) or abs(heading)>115:raise ValueError('Restore cable-neutral before continuing')
    return heading


def run_live(args):
    import yaml
    runtime=Path(args.runtime).resolve()
    health=probe(args.seconds,True)
    if not health['ok'] and args.recover:
        restart(SERVICES[:5])  # No bridge/head restart or encoder change during recovery.
        health=probe(max(12.,args.seconds),True)
    if not health['ok']:return health
    geometry=yaml.safe_load((runtime/'robot.yaml').read_text())['ev3_bridge']['ros__parameters']
    previous=json.loads(Path(args.resume).read_text()) if args.resume else None
    if previous is None and not args.neutral:raise ValueError('Use --neutral only at the marked cable-neutral pose, or --resume with the previous preflight.json')
    heading=start_heading(health['status'],previous,geometry)
    out=runtime/'logs'/('phase6-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir(parents=True)
    health['initial_heading_degrees']=heading
    (out/'preflight.json').write_text(json.dumps(health,indent=2))
    command=[sys.executable,str(runtime/'autonomous_find.py'),'--execute','--skip-camera-calibration',
             '--cable-zero-confirmed','--preserve-initial-heading','--initial-cable-heading-degrees',str(heading),
             '--route-url',args.route_url,'--maximum-approach-steps',str(args.steps),'--report',str(out/'mission.json')]
    for option, filename in (('--scan-script', 'bounded_target_scan.py'),
                             ('--approach-script', 'closed_loop_detour.py')):
        command.extend([option, str(runtime / filename)])
    print('Running; report:',out/'mission.json',flush=True)
    subprocess.run(command,cwd=runtime,check=False)
    result=json.loads((out/'mission.json').read_text())
    result['resume_preflight']=str(out/'preflight.json')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    pack=sub.add_parser('bundle');pack.add_argument('--output',required=True)
    deploy=sub.add_parser('install');deploy.add_argument('archive');deploy.add_argument('--household-profile',action='store_true');deploy.add_argument('--restart',action='store_true')
    deploy.add_argument('--faster-tracks', action='store_true',
                        help='merge the 240 degrees/second track cap into existing robot.yaml')
    check=sub.add_parser('check');check.add_argument('--with-robot',action='store_true')
    repair=sub.add_parser('recover')
    live=sub.add_parser('run');live.add_argument('--neutral',action='store_true');live.add_argument('--resume');live.add_argument('--recover',action='store_true');live.add_argument('--steps',type=int,default=8);live.add_argument('--route-url',default='http://127.0.0.1:18091/route')
    for p in (deploy,check,repair,live):p.add_argument('--runtime',default='/home/animesh/echora')
    for p in (check,repair,live):p.add_argument('--seconds',type=float,default=8.);p.add_argument('--report')
    args=parser.parse_args(argv)
    if hasattr(args,'seconds') and not 4<=args.seconds<=60:parser.error('seconds must be between 4 and 60')
    if args.command=='run' and (args.neutral and args.resume or not 1<=args.steps<=30):parser.error('choose neutral or resume, and 1–30 steps')
    if args.command=='bundle':bundle(args.output);return 0
    if args.command=='install':install(args);return 0
    sys.path.insert(0,args.runtime)
    from image_subscription import configure_perception_transport
    configure_perception_transport()
    if args.command=='recover':restart(SERVICES[:5]);result=probe(max(12.,args.seconds))
    elif args.command=='check':result=probe(args.seconds,args.with_robot)
    else:result=run_live(args)
    if args.report:Path(args.report).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0 if result.get('ok') or result.get('outcome') in ('target_found_at_standoff','target_found_at_estimated_standoff','target_found_not_at_standoff','step_complete_target_reacquired') else 2


if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,OSError,subprocess.CalledProcessError) as exc:
        print('Not completed:',exc,file=sys.stderr);sys.exit(2)
