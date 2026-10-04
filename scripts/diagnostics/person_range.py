#!/usr/bin/env python3
"""Capture numeric person-range evidence or replay it without any hardware."""
import argparse
import io
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))

from robot.mac.range_replay import camera_from_yaml, surface_record, replay_documents


def capture(args):
    from PIL import Image
    from scripts.diagnostics.compare_camera_tilt import capture_live
    from robot.mac.person_range import MetricDepthBackend
    from robot.mac.range_diagnostics import compare_range_models
    from robot.mac.route_perception import RoutePerceptionEngine
    jpeg,binding=capture_live(args.host,person=True)
    boxes=binding['person_boxes']
    index=args.candidate
    if index is None:
        if len(boxes)!=1:
            raise ValueError('Multiple people are visible; choose --candidate INDEX for this diagnostic')
        index=0
    if not 0<=index<len(boxes): raise ValueError('Candidate index is outside this capture')
    camera=camera_from_yaml(binding['camera_yaml'])
    image=Image.open(io.BytesIO(jpeg)).convert('RGB')
    if list(image.size)!=camera['image_size']:
        raise ValueError('Camera dimensions differ from calibrated intrinsics')
    # Initialise only what this check needs. GeoCalib and cloud calls are not used.
    segmentation=RoutePerceptionEngine()
    depth=MetricDepthBackend(segmentation.device).infer(image)
    masks=segmentation.infer(jpeg,return_masks=True)
    box=[int(v) for v in boxes[index]]
    report=compare_range_models(depth,masks,box,camera,binding['pose_binding']['head'],
        height_m=args.height_m,front_offset_m=args.front_offset_m,
        measured_gap_m=args.gap_m if args.measured_region=='nearest_visible_surface' else None)
    report.update(surface_record(depth,masks,box))
    return dict(schema=2,capture=binding,camera=camera,people=[report],
        selected_candidate=index,operator_gap_m=args.gap_m,measured_region=args.measured_region,
        operator_lower_height_m=args.height_m,front_offset_m=args.front_offset_m,
        operator_foot_on_floor={'unknown':None,'on-floor':True,'raised':False}[args.feet],
        identity_source='anonymous_local_person_detection',recognition_confirmed=False,
        observation_only=True,images_saved=False,commands_sent=False,validated=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    live=commands.add_parser('capture',help='Stationary capture; numeric evidence only')
    live.add_argument('--host',default='192.168.1.48')
    live.add_argument('--height-m',type=float,required=True)
    live.add_argument('--front-offset-m',type=float)
    live.add_argument('--gap-m',type=float)
    live.add_argument('--measured-region',choices=['nearest_foot','nearest_visible_surface'],default='nearest_foot')
    live.add_argument('--feet',choices=['unknown','on-floor','raised'],default='unknown')
    live.add_argument('--candidate',type=int)
    offline=commands.add_parser('replay',help='Offline comparison; no inference or hardware access')
    offline.add_argument('records',nargs='+',type=Path)
    offline.add_argument('--geometry',type=Path,help='Optional experimental pose with camera/encoder provenance')
    for command in (live,offline):command.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        if args.command=='capture': report=capture(args)
        else:
            documents=[]
            for path in args.records:
                try: documents.append((str(path),json.loads(path.read_text())))
                except (OSError,ValueError) as exc: documents.append((str(path),dict(read_error=str(exc))))
            geometry=json.loads(args.geometry.read_text()) if args.geometry else None
            report=replay_documents(documents,geometry)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(report=str(args.output),records=len(report.get('results',report.get('people',[]))),
                              observation_only=True,commands_sent=False)))
    except (ValueError,OSError,KeyError) as exc:
        parser.exit(1,str(exc)+'\n')


if __name__=='__main__': main()
