#!/usr/bin/env python3
"""Family-memory inspection and measured range acceptance. Never commands motors."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))


def calibrate(document):
    from robot.mac.person_range import MODEL
    samples=document['samples']
    training=[s for s in samples if s['split']=='train']
    validation=[s for s in samples if s['split']=='validation']
    if len(training)<3 or len(validation)<6:
        raise ValueError('Supply at least three training and six held-out measured samples')
    ids=[s['frame_sha256'] for s in samples]
    if len(set(ids))!=len(ids):raise ValueError('Training and validation must use distinct frames')
    for s in samples:
        if not s.get('frame_sha256') or s.get('posture') not in ('seated','standing'):
            raise ValueError('Each measured frame needs its image hash and posture')
        if any(not isinstance(s[k],(int,float)) or not math.isfinite(s[k]) or s[k]<=0 for k in ('measured_m','estimated_m')):
            raise ValueError('Distances must be finite positive measurements')
        if type(s.get('head_fraction')) not in (int,float) or not math.isfinite(s['head_fraction']) or not 0<=s['head_fraction']<=1:
            raise ValueError('Each sample needs its measured head fraction')
    # Samples are robot-front gaps. Scale acts on camera-forward depth before
    # subtracting the measured camera-to-front offset, not on the final gap.
    scale=statistics.median((s['measured_m']+s.get('front_offset_m',0))/(s['estimated_m']+s.get('front_offset_m',0)) for s in training)
    def corrected(s):return (s['estimated_m']+s.get('front_offset_m',0))*scale-s.get('front_offset_m',0)
    near=[s for s in validation if .5<=s['measured_m']<=1]
    far=[s for s in validation if 1<s['measured_m']<=3]
    if len(near)<3 or len(far)<3 or {s['posture'] for s in validation}!={'seated','standing'}:
        raise ValueError('Held-out checks need near/far distances and seated/standing examples')
    if any({s['posture'] for s in group}!={'seated','standing'} for group in (near,far)):
        raise ValueError('Test both postures in both distance bands')
    near_error=max(abs(corrected(s)-s['measured_m']) for s in near)
    far_error=max(abs(corrected(s)-s['measured_m'])/s['measured_m'] for s in far)
    result={k:document[k] for k in ('intrinsics','image_size','head_poses')}
    if len(result['head_poses'])<2 or any(set(p)!= {'fraction','height_m','pitch_degrees','front_offset_m'} for p in result['head_poses']):
        raise ValueError('Measure the camera height, pitch and front offset at lower and upper views')
    for pose in result['head_poses']:
        if (any(type(v) not in (int,float) or not math.isfinite(v) for v in pose.values())
                or not 0<=pose['fraction']<=1 or not .03<=pose['height_m']<=1
                or not -85<=pose['pitch_degrees']<=85 or not -1<=pose['front_offset_m']<=1):
            raise ValueError('Head geometry must contain finite measured values')
    fractions=[p['fraction'] for p in result['head_poses']]
    if len(set(fractions))!=len(fractions):raise ValueError('Head geometry positions must differ')
    for s in samples:
        if not min(fractions)<=s['head_fraction']<=max(fractions):raise ValueError('Sample outside measured head geometry')
    tested=[s['head_fraction'] for s in validation]
    if max(tested)-min(tested)<.2:raise ValueError('Validate more than one useful head position')
    result['distortion']=document.get('distortion',[])
    result['validated_head_fraction']=[min(tested),max(tested)]
    return dict(result,schema=1,model=MODEL,depth_scale=scale,near_error_m=max(.03,near_error),
                far_relative_error=max(.05,far_error),validated=near_error<=.10 and far_error<=.20,
                acceptance=dict(near_max_error_m=near_error,far_max_relative_error=far_error,
                    validation_count=len(validation),frames=[s['frame_sha256'] for s in validation]))


def evaluate_wardrobe(document,root,advisor):
    """Recorded crops and explicit labels; no face enrollment or motor access."""
    import time
    references=document['references'];cases=document['cases'];results=[]
    owners={r['profile_id'] for r in references}
    for case in cases:
        current=(root/case['image']).read_bytes()
        chosen=[r for r in references if r['id'] in case['references']]
        if len(chosen)!=len(set(case['references'])):raise ValueError('Unknown evaluation reference')
        images=[(r,(root/r['image']).read_bytes()) for r in chosen]
        if any(hashlib.sha256(image).digest()==hashlib.sha256(current).digest() for _,image in images):
            raise ValueError('Evaluation candidates must differ from their reference images')
        binding=dict(request_id=case['id'],frame_key=['recorded',len(results),0],track_id=case['id'],
                     revision='evaluation',image_sha256=hashlib.sha256(current).hexdigest())
        started=time.monotonic()
        try:
            answer=advisor.interpret(dict(operation='compare',binding=binding,image=base64.b64encode(current).decode(),
                references=[dict(id=r['id'],image=base64.b64encode(image).decode()) for r,image in images]))
            value=answer['interpretation']
            match=next((r for r in chosen if r['id']==value['reference_id']),None)
            identified=match['profile_id'] if match and value['decision']=='match' else None
            results.append(dict(id=case['id'],scenario=case['scenario'],expected=case['expected_profile'],
                identified=identified,false_identity=identified is not None and identified!=case['expected_profile'],
                seconds=time.monotonic()-started,answer=answer))
        except Exception as exc:
            results.append(dict(id=case['id'],scenario=case['scenario'],error=type(exc).__name__))
            if getattr(exc,'status',None)==429:break  # Respect provider cooldown; never rotate to evade quotas.
    required={'same_outfit','different_outfit','partial_trousers','side_view','low_light','shared_clothes','crossing'}
    covered={r['scenario'] for r in results if not r.get('error')}
    matched={r['identified'] for r in results if r.get('identified') and r.get('identified')==r.get('expected')}
    false_count=sum(r.get('false_identity',False) for r in results)
    ready=(len(owners)>=2 and required<=covered and owners<=matched and len(results)==len(cases)
           and not false_count and not any(r.get('error') for r in results))
    return dict(observation_only=True,ready_for_supervised_check=ready,
                false_identity_count=false_count,required_scenarios_missing=sorted(required-covered),cases=results)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    status=sub.add_parser('status');status.add_argument('--database',required=True)
    calibration=sub.add_parser('calibrate-range');calibration.add_argument('measurements');calibration.add_argument('--output',required=True)
    calibration.add_argument('--head-poses',help='Measured geometry JSON; required when input is a directory of captured samples')
    evaluation=sub.add_parser('evaluate-wardrobe');evaluation.add_argument('manifest');evaluation.add_argument('--output',required=True)
    evaluation.add_argument('--execute-cloud',action='store_true',help='Send the manifest clothing crops through the configured Gemini ADC account')
    warm=sub.add_parser('warm-depth');warm.add_argument('--device',default='mps')
    diagnostic=sub.add_parser('diagnose-range',help='Compare one stationary view without changing range settings or saving its image')
    diagnostic.add_argument('--ui-url',default='http://192.168.1.48:8080')
    diagnostic.add_argument('--height-m',type=float)
    diagnostic.add_argument('--front-offset-m',type=float)
    units=diagnostic.add_mutually_exclusive_group()
    units.add_argument('--measured-m',type=float)
    units.add_argument('--measured-inches',type=float)
    diagnostic.add_argument('--output',required=True)
    capture=sub.add_parser('capture-range')
    capture.add_argument('--ui-url',default='http://192.168.1.48:8080')
    capture.add_argument('--measured-m',type=float,required=True)
    capture.add_argument('--height-m',type=float,required=True)
    capture.add_argument('--pitch-degrees',type=float,
                         help='Optional measured tilt; otherwise estimate it from the segmented floor plane')
    capture.add_argument('--front-offset-m',type=float,required=True)
    capture.add_argument('--posture',choices=['seated','standing'],required=True)
    capture.add_argument('--split',choices=['train','validation'],required=True)
    capture.add_argument('--output',required=True)
    args=parser.parse_args()
    if args.command=='status':
        from robot.jetson.perception.family_store import FamilyStore
        if not Path(args.database).exists():raise SystemExit('Family database is not installed yet')
        store=FamilyStore(args.database)
        print(json.dumps(dict(selected=store.selected_id(),profiles=store.profiles(),outfits=store.wardrobe()),indent=2))
    elif args.command=='calibrate-range':
        source=Path(args.measurements)
        if source.is_dir():
            if not args.head_poses:raise SystemExit('Provide --head-poses with the measured camera geometry')
            samples=[json.loads(p.read_text()) for p in sorted(source.glob('*.json'))]
            if not samples:raise SystemExit('No measured samples found')
            camera={k:samples[0][k] for k in ('intrinsics','image_size','distortion')}
            if any(any(s[k]!=v for k,v in camera.items()) for s in samples):raise SystemExit('Samples use different camera calibrations')
            document=dict(camera,samples=samples,head_poses=json.loads(Path(args.head_poses).read_text()))
        else:document=json.loads(source.read_text())
        result=calibrate(document)
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        temporary=output.with_suffix('.tmp');temporary.write_text(json.dumps(result,indent=2));temporary.replace(output)
        print(json.dumps(result['acceptance'],indent=2))
        if not result['validated']:raise SystemExit('Range acceptance failed; observation-only mode remains appropriate')
    elif args.command=='evaluate-wardrobe':
        if not args.execute_cloud:raise SystemExit('Use --execute-cloud to run the specified images through Gemini; no motors are used')
        from robot.mac.wardrobe_advisor import WardrobeAdvisor
        path=Path(args.manifest).resolve()
        result=evaluate_wardrobe(json.loads(path.read_text()),path.parent,WardrobeAdvisor())
        Path(args.output).write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))
    elif args.command=='diagnose-range':
        from robot.mac.range_diagnostics import capture_diagnostic
        gap=args.measured_inches*.0254 if args.measured_inches is not None else args.measured_m
        report=capture_diagnostic(args.ui_url,height_m=args.height_m,
            front_offset_m=args.front_offset_m,measured_gap_m=gap)
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(cases=report['cases'],comparison=report['comparison']),indent=2))
    elif args.command=='warm-depth':
        from robot.mac.person_range import MetricDepthBackend,MODEL
        from PIL import Image
        import time
        backend=MetricDepthBackend(args.device);image=Image.new('RGB',(640,480),(128,128,128))
        backend.infer(image)
        started=time.monotonic();result=backend.infer(image)
        print(json.dumps(dict(model=MODEL,shape=list(result.shape),warm_seconds=time.monotonic()-started)))
    else:
        from urllib.request import urlopen
        from PIL import Image
        import io
        import numpy as np
        import yaml
        from robot.mac.person_range import MetricDepthBackend,estimate_range,infer_floor_pose,head_fraction
        from robot.mac.route_perception import RoutePerceptionEngine
        with urlopen(args.ui_url+'/api/status',timeout=4) as response:status=json.load(response)
        if status.get('mission',{}).get('running'):raise SystemExit('Finish the mission before taking a measured example')
        with urlopen(args.ui_url+'/api/family/range-frame',timeout=4) as response:capture=json.load(response)
        head=capture['head'];track=capture['track'];jpeg=base64.b64decode(capture['image'],validate=True)
        if hashlib.sha256(jpeg).hexdigest()!=capture['image_sha256']:raise SystemExit('Measurement image binding changed')
        image=Image.open(io.BytesIO(jpeg)).convert('RGB')
        if capture.get('capture_schema')!=1 or not capture.get('camera'):
            raise SystemExit('Activate the updated console before taking a measured example')
        camera=capture['camera']
        if list(image.size)!=camera['image_size']:raise SystemExit('Camera resolution changed')
        config=dict(camera,depth_scale=1.,near_error_m=.1,far_relative_error=.2,validated=False)
        depth=MetricDepthBackend().infer(image);masks=RoutePerceptionEngine().infer(jpeg,return_masks=True)
        floor_pose = (infer_floor_pose(depth,masks['floor'],config['intrinsics'],args.height_m,config['distortion'])
                      if args.pitch_degrees is None else None)
        pose=dict(height_m=args.height_m,pitch_degrees=floor_pose['pitch_degrees'] if floor_pose else args.pitch_degrees,
                  front_offset_m=args.front_offset_m)
        result=estimate_range(depth,masks['person'],np.zeros_like(masks['floor']),track['body_box'],config,pose)
        sample=dict(frame_sha256=hashlib.sha256(jpeg).hexdigest(),measured_m=args.measured_m,
            estimated_m=result['distance_m'],front_offset_m=args.front_offset_m,posture=args.posture,split=args.split,
            head_fraction=head_fraction(head),pose_binding=capture['pose_binding'],
            frame_key=track['frame_key'],profile_id=track['profile_id'],track_id=track['track_id'],
            image_size=list(image.size),intrinsics=config['intrinsics'],distortion=config['distortion'],head_pose=pose,
            floor_pose=floor_pose,source='metric_indoor_small_raw')
        Path(args.output).write_text(json.dumps(sample,indent=2));print(json.dumps(sample,indent=2))


if __name__=='__main__':main()
