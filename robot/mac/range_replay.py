"""Replay numeric range evidence without cameras, cloud calls or motor access.

Sparse depth samples can compare geometry; they cannot recreate an image, rerun
segmentation, identify feet or validate mask extensions in historical frames.
"""
import math

from robot.jetson.perception.range_capture import same_capture_pose
from robot.mac.range_anchor import sample_rays, fit_floor_correction, corrected_surfaces
from robot.mac.person_range import MODEL, person_surface


def comparable_pose(before, after):
    try:
        return same_capture_pose(before,after)
    except (KeyError,TypeError,ValueError):
        return False


def camera_from_yaml(value):
    data = value['camera_matrix']['data']
    return dict(image_size=[value['image_width'],value['image_height']],
                intrinsics=dict(fx=data[0],fy=data[4],cx=data[2],cy=data[5]),
                distortion=value['distortion_coefficients']['data'])


def numeric_samples(depth, mask, limit=1000):
    import numpy as np
    depth = np.asarray(depth); mask = np.asarray(mask,dtype=bool)
    if depth.shape != mask.shape or depth.ndim != 2:
        raise ValueError('Sample mask and depth layouts differ')
    ys,xs = np.where(mask & np.isfinite(depth) & (depth>.05))
    if not len(xs): return []
    # Evenly spaced row-ordered samples retain bottom coverage deterministically.
    indices = np.linspace(0,len(xs)-1,min(len(xs),limit),dtype=int)
    return np.column_stack((xs[indices],ys[indices],depth[ys[indices],xs[indices]])).tolist()


def surface_record(depth, masks, box):
    """Keep counts and edge context so crop faults do not need another pose."""
    import numpy as np
    surface,evidence = person_surface(masks['person'],box)
    person = np.asarray(masks['person'],dtype=bool)
    floor = np.asarray(masks['floor'],dtype=bool)
    x1,y1,x2,y2 = [int(v) for v in box]
    roi = np.zeros_like(surface); roi[y1:y2,x1:x2]=True
    return dict(box=list(box),surface_evidence=evidence,
                person_depth_samples=numeric_samples(depth,surface),
                detector_person_depth_samples=numeric_samples(depth,person & roi),
                floor_depth_samples=numeric_samples(depth,floor),
                person_pixels=int(surface.sum()), floor_pixels=int(floor.sum()))


def normalize_record(document):
    """Read both the original September captures and the reusable schema."""
    if document.get('read_error'):
        raise ValueError(document['read_error'])
    capture = document['capture']
    camera = document.get('camera') or camera_from_yaml(capture['camera_yaml'])
    people = document['people']
    if len(people) != 1:
        raise ValueError('Replay one explicitly selected candidate per record')
    person = people[0]
    if person.get('model',MODEL) != MODEL:
        raise ValueError('Record belongs to another depth model')
    samples,rays = sample_rays(person['person_depth_samples'],camera)
    sample_rays(person['floor_depth_samples'],camera)
    if len(samples)<30:
        raise ValueError('Not enough saved person samples')
    return dict(camera=camera,capture=capture,person=person,samples=samples,rays=rays,
                height=document.get('operator_lower_height_m'),
                gap=document.get('operator_gap_m'),front_offset=document.get('front_offset_m'),
                foot_on_floor=document.get('operator_foot_on_floor'),
                # Old records measured a foot, never the nearest knee or torso.
                measured_region=document.get('measured_region','nearest_foot'))


def summarize_surfaces(forward, samples, offset=None):
    import numpy as np
    if not np.isfinite(forward).all() or np.any(forward<=0):
        raise ValueError('Geometry yields invalid person distances')
    low = samples[:,1]>=np.quantile(samples[:,1],.98)
    result = dict(nearest_visible_surface_m=float(np.quantile(forward,.05)),
                  lowest_image_band_median_m=float(np.median(forward[low])),
                  lowest_band_is_identified_foot=False,
                  distance_reference='camera_ground_projection')
    if offset is not None:
        if type(offset) not in (int,float) or not math.isfinite(offset) or not -1<=offset<=1:
            raise ValueError('Invalid camera/front offset')
        result['nearest_visible_front_gap_m']=result['nearest_visible_surface_m']-offset
    return result


def replay_one(record, geometry=None):
    import numpy as np
    p = record['person']; fit = p['floor_fit']; samples = record['samples']; rays=record['rays']
    height = record['height']
    if type(height) not in (int,float) or not math.isfinite(height) or not .03<=height<=1:
        raise ValueError('Missing measured lens height')
    scale = height/fit['plane_distance_model_units']
    pitch = math.radians(fit['pitch_degrees'])
    forward = samples[:,2]*scale*(math.cos(pitch)-rays[:,1]*math.sin(pitch))
    result = dict(frame_key=record['capture']['frame_key'],
                  frame_sha256=record['capture']['image_sha256'],
                  model=MODEL,sample_count=len(samples),floor_fit=fit,
                  sampled_floor_estimate=summarize_surfaces(forward,samples,record['front_offset']),
                  recorded_full_mask_estimate=p.get('cases',{}).get('measured_height'),
                  surface_evidence=p.get('surface_evidence'),
                  full_mask_available=False, mask_extension_replay_available=False,
                  measured_gap_m=record['gap'], measured_region=record['measured_region'],
                  foot_on_floor=record['foot_on_floor'], validated=False,
                  comparison=dict(available=False,reason='Saved foot distance is not a measurement of the nearest visible body surface; camera/front offset may also be unknown'))
    box = p['box']
    result['sample_at_detector_bottom']=bool(np.max(samples[:,1])>=box[3]-2)
    result['detector_bottom_contact_proves_clipping']=False
    if record['measured_region']=='nearest_visible_surface' and record['front_offset'] is not None and record['gap'] is not None:
        gap=record['gap']
        if type(gap) not in (int,float) or not math.isfinite(gap) or not .05<=gap<=10:
            raise ValueError('Invalid measured gap')
        error=result['sampled_floor_estimate']['nearest_visible_front_gap_m']-gap
        result['comparison']=dict(available=True,signed_error_m=error,
            point_within_target=abs(error)<=(.1 if gap<=1 else .2*gap),
            formal_accuracy_validation=False,sample_based=True)
    if geometry is not None:
        compatible = (geometry.get('camera')==record['camera'] and
                      comparable_pose(geometry.get('pose_binding'),record['capture'].get('pose_binding')))
        if not compatible:
            result['anchored_candidate']=dict(available=False,reason='Independent geometry belongs to another camera or hardware pose')
        else:
            try:
                correction=fit_floor_correction(p['floor_depth_samples'],record['camera'],geometry['pose'])
                surfaces=corrected_surfaces(samples,record['camera'],correction)
                anchor=result['frame_sha256']==geometry.get('anchor_frame_sha256')
                result['anchored_candidate']=dict(available=True,correction=correction,
                    **summarize_surfaces(surfaces['forward_m'],samples),
                    is_anchor_frame=anchor,validated=False,
                    geometry_provenance=geometry.get('provenance','unspecified'))
            except ValueError as exc:
                result['anchored_candidate']=dict(available=False,reason=str(exc))
    return result


def replay_documents(documents, geometry=None):
    records=[]; results=[]
    for name,document in documents:
        try:
            record=normalize_record(document)
            result=replay_one(record,geometry)
            records.append((name,record)); results.append(dict(input=name,ok=True,**result))
        except (ValueError,KeyError,TypeError,IndexError,ZeroDivisionError) as exc:
            results.append(dict(input=name,ok=False,reason=str(exc),validated=False))
    comparisons=[]
    for i,(name,a) in enumerate(records):
        for other,b in records[i+1:]:
            if (a['camera']!=b['camera'] or not comparable_pose(a['capture'].get('pose_binding'),b['capture'].get('pose_binding'))
                    or a['capture']['frame_key']==b['capture']['frame_key']):continue
            pa,pb=a['person']['floor_fit'],b['person']['floor_fit']
            comparisons.append(dict(inputs=[name,other],same_stopped_pose=True,
                depth_pitch_change_degrees=pb['pitch_degrees']-pa['pitch_degrees'],
                model_floor_distance_ratio=pb['plane_distance_model_units']/pa['plane_distance_model_units'],
                proves_physical_tilt_change=False))
    return dict(schema=1,observation_only=True,validated=False,images_saved=False,
                commands_sent=False,runtime_changes=False,replay_kind='sparse_numeric_geometry',
                results=results,same_pose_comparisons=comparisons,
                limitations=['Sparse samples cannot rerun segmentation or recover clipped pixels.',
                             'Lowest image pixels are not automatically feet.',
                             'A fit on its own anchor frame is not held-out accuracy validation.'])
