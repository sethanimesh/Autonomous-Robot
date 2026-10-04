"""Metric person range. A calibrated camera model, never clothing size, sets scale."""
import base64
import hashlib
import io
import math
import threading
import time
from functools import lru_cache

MODEL = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'
APPROXIMATE_SOURCE = 'metric_depth_floor_estimate'


def floor_plane_orientation(depth, floor_mask, intrinsics, distortion=()):
    """Estimate tilt from a segmented floor without assuming camera height.

    A uniform depth-scale error changes the plane distance, not its normal.
    The reported plane distance is in model units, not calibrated metres.
    """
    import cv2
    import numpy as np
    depth = np.asarray(depth)
    floor = np.asarray(floor_mask, dtype=bool)
    if depth.ndim != 2 or floor.shape != depth.shape:
        raise ValueError('Floor and depth layouts differ')
    fx, fy, cx, cy = (float(intrinsics[k]) for k in ('fx', 'fy', 'cx', 'cy'))
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError('Invalid calibrated camera intrinsics')
    ys, xs = np.where(floor & np.isfinite(depth) & (depth > .05) & (depth < 10))
    h, w = depth.shape
    if len(xs) < 200 or np.ptp(xs) < .2*w or np.ptp(ys) < .08*h:
        raise ValueError('Show a broader patch of floor to estimate camera tilt')
    rng = np.random.default_rng(7)
    chosen = rng.choice(len(xs), min(2500, len(xs)), replace=False)
    xs, ys = xs[chosen], ys[chosen]
    pixels = np.stack((xs, ys), axis=-1).astype(np.float64).reshape(-1, 1, 2)
    matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
    rays = cv2.undistortPoints(pixels, matrix, np.asarray(distortion, dtype=float) if len(distortion) else None)[:, 0]
    points = np.column_stack((rays, np.ones(len(rays)))) * depth[ys, xs, None]
    best = None
    for _ in range(100):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[1] < 0:
            normal = -normal
        distance = float(a @ normal)
        if distance <= 0 or normal[1] < .25 or abs(normal[0]) > .26:
            continue
        residual = np.abs(points @ normal-distance)
        inliers = residual <= .10*distance
        score = (int(inliers.sum()), -float(np.median(residual)))
        if best is None or score > best[0]:
            best = (score, inliers)
    if best is None or best[0][0] < .65*len(points):
        raise ValueError('Floor depth does not form a consistent plane; another view is needed')
    inliers = best[1]
    for _ in range(2):
        subset = points[inliers]
        centre = subset.mean(axis=0)
        _, _, vectors = np.linalg.svd(subset-centre, full_matrices=False)
        normal = vectors[-1]
        if normal[1] < 0:
            normal = -normal
        distance = float(centre @ normal)
        if distance <= 0:
            raise ValueError('Floor is not below the camera')
        residual = np.abs(points @ normal-distance)
        inliers = residual <= .10*distance
        if inliers.mean() < .65:
            raise ValueError('Floor plane is unstable; take another view')
    if normal[1] < .25 or abs(normal[0]) > .26:
        raise ValueError('Camera roll or floor orientation needs a measured tilt')
    return dict(pitch_degrees=math.degrees(math.atan2(normal[2], normal[1])),
                plane_distance_model_units=distance, source='segmented_depth_floor_plane',
                floor_inlier_fraction=float(inliers.mean()),
                floor_residual_fraction=float(np.median(residual[inliers])/distance),
                validated=False)


def infer_floor_pose(depth, floor_mask, intrinsics, height_m, distortion=()):
    """Attach the physical height for calibration captures; never validate range."""
    if type(height_m) not in (int, float) or not math.isfinite(height_m) or not .03 <= height_m <= 1:
        raise ValueError('Supply the measured lens height')
    orientation = floor_plane_orientation(depth, floor_mask, intrinsics, distortion)
    return dict(orientation, height_m=float(height_m),
                floor_scale_hint=height_m/orientation['plane_distance_model_units'])


@lru_cache(maxsize=4)
def vertical_rays(width,height,fx,fy,cx,cy,distortion):
    import numpy as np
    if not all(math.isfinite(v) for v in (fx,fy,cx,cy)) or min(fx,fy)<=0:
        raise ValueError('Invalid calibrated camera intrinsics')
    ys,xs=np.indices((height,width),dtype=np.float32)
    if not distortion:return (ys-cy)/fy
    import cv2
    matrix=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=float)
    points=np.stack((xs,ys),axis=-1).reshape(-1,1,2)
    return cv2.undistortPoints(points,matrix,np.array(distortion,dtype=float))[:,0,1].reshape(height,width)


def head_fraction(head):
    values = [head.get(k) for k in ('position', 'down_position', 'up_position')]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Head position unavailable')
    position, lower, upper = values
    if lower == upper:
        raise ValueError('Camera range is unavailable')
    tolerance = head.get('settle_tolerance', 3)
    if type(tolerance) not in (int, float) or not math.isfinite(tolerance):
        raise ValueError('Invalid head settling tolerance')
    tolerance = max(0, min(6, tolerance))
    if not min(lower, upper)-tolerance <= position <= max(lower, upper)+tolerance:
        raise ValueError('Camera is outside its known range')
    return max(0., min(1., (position-lower)/(upper-lower)))


def camera_pose(calibration, head):
    if calibration.get('schema')!=1 or calibration.get('model')!=MODEL:
        raise ValueError('Metric range needs a matching measured calibration')
    fraction=head_fraction(head)
    validated_span=calibration.get('validated_head_fraction')
    if calibration.get('validated') and validated_span and not validated_span[0]-.05<=fraction<=validated_span[1]+.05:
        raise ValueError('No held-out range measurements cover this head position')
    poses=sorted(calibration['head_poses'],key=lambda p:p['fraction'])
    if not poses or fraction<poses[0]['fraction']-.05 or fraction>poses[-1]['fraction']+.05:
        raise ValueError('This camera angle has not been measured')
    a,b=poses[0],poses[-1]
    for left,right in zip(poses,poses[1:]):
        if left['fraction']<=fraction<=right['fraction']:a,b=left,right;break
    t=max(0.,min(1.,(fraction-a['fraction'])/max(.001,b['fraction']-a['fraction'])))
    result={k:float(a[k])*(1-t)+float(b[k])*t for k in ('height_m','pitch_degrees','front_offset_m')}
    if not all(math.isfinite(v) for v in result.values()) or not .03<=result['height_m']<=1:
        raise ValueError('Invalid measured head geometry')
    result['fraction']=fraction
    return result


def person_surface(person_mask, box):
    """Recover small detector cutoffs using the connected person segmentation.

    Semantic masks do not distinguish touching people. Extend only a component
    mostly owned by this detection, within a small neighbourhood of the box.
    Uncertain extensions keep the original pixels and request another foot view.
    """
    import cv2
    import numpy as np
    mask = np.asarray(person_mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError('Expected a person segmentation image')
    h, w = mask.shape
    x1, y1, x2, y2 = [int(v) for v in box]
    if not 0 <= x1 < x2 <= w or not 0 <= y1 < y2 <= h:
        raise ValueError('Person box outside image')
    roi = np.zeros_like(mask); roi[y1:y2, x1:x2] = True
    selected = mask & roi
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    inside = np.bincount(labels[selected], minlength=count)
    total = np.bincount(labels.ravel(), minlength=count)
    inside[0] = 0
    primary = int(inside.argmax())
    owned = (primary != 0 and inside[primary] >= 30
             and inside[primary] >= .6 * selected.sum()
             and inside[primary] >= .6 * total[primary])
    continuation = False
    if owned:
        component = labels == primary
        pad_x = min(16, max(2, round(.1 * (x2-x1))))
        pad_y = min(64, max(4, round(.2 * (y2-y1))))
        neighbourhood = roi.copy()
        neighbourhood[y1:min(h, y2+pad_y), max(0, x1-pad_x):min(w, x2+pad_x)] = True
        selected |= component & neighbourhood
        # A component reaching beyond the new crop has not supplied its end.
        continuation = bool((component[y2:] & ~neighbourhood[y2:]).any())
    elif selected.any():
        # No unique connected owner: do not take another person's lower pixels.
        ids = np.flatnonzero(inside)
        continuation = bool(np.isin(labels[y2:], ids).any()) if len(ids) else False
    ys, xs = np.where(selected)
    extent = [int(xs.min()), int(ys.min()), int(xs.max()+1), int(ys.max()+1)] if len(xs) else None
    return selected, dict(detector_box=[x1, y1, x2, y2], surface_box=extent,
                          extension_pixels=int((selected & ~roi).sum()),
                          lower_continuation=continuation,
                          image_bottom_clipped=bool(len(ys) and ys.max() >= h-2),
                          unique_component=bool(owned))


def estimate_range(depth,person_mask,floor_mask,box,calibration,pose):
    import numpy as np
    from robot.jetson.navigation.ground_plane import expected_floor_z_m
    depth=np.asarray(depth);mask=np.asarray(person_mask,dtype=bool)
    if depth.ndim!=2 or mask.shape!=depth.shape or np.asarray(floor_mask).shape!=depth.shape:
        raise ValueError('Depth and segmentation must be from the same image layout')
    h,w=depth.shape;fy,cy=calibration['intrinsics']['fy'],calibration['intrinsics']['cy']
    intrinsics=calibration['intrinsics']
    rays=vertical_rays(w,h,intrinsics.get('fx',fy),fy,intrinsics.get('cx',w/2),cy,tuple(calibration.get('distortion',[])))
    surface, surface_evidence=person_surface(mask,box)
    valid=surface & np.isfinite(depth) & (depth>.05)
    if valid.sum()<30:raise ValueError('No segmented person surface for distance')
    scale=float(calibration['depth_scale'])
    floor=np.asarray(floor_mask,dtype=bool)&np.isfinite(depth)&(depth>.05)
    ys,xs=np.where(floor)
    ratios=[]
    for y,x in zip(ys[::20],xs[::20]):
        expected=expected_floor_z_m(float(rays[y,x]),1.,0.,pose['pitch_degrees'],pose['height_m'])
        if expected:ratios.append(expected/depth[y,x])
    if len(ratios)>=50:
        measured=float(np.median(ratios));spread=float(np.median(np.abs(np.log(np.array(ratios)/measured))))
        if spread<.12:
            if not .7<=measured/scale<=1.3:raise ValueError('Floor geometry disagrees with saved scale')
            # The measured, held-out scale remains authoritative. An adaptive
            # correction here would invalidate the recorded error bounds.
    ys,xs=np.where(valid);z=depth[ys,xs]*scale
    pitch=math.radians(pose['pitch_degrees'])
    forward=z*(math.cos(pitch)-rays[ys,xs]*math.sin(pitch))-pose['front_offset_m']
    body_height=pose['height_m']-z*(math.sin(pitch)+rays[ys,xs]*math.cos(pitch))
    bottom=ys>=np.quantile(ys,.9)
    ground_contact=bottom.sum()>=5 and float(np.quantile(np.abs(body_height[bottom]),.2))<=.12
    forward=forward[np.isfinite(forward)&(forward>0)]
    if len(forward)<30:raise ValueError('No valid forward person range')
    distance=float(np.quantile(forward,.05))
    error=max(float(calibration['near_error_m']) if distance<=1 else distance*float(calibration['far_relative_error']),
              float(np.quantile(forward,.15)-np.quantile(forward,.05))/2)
    # Check floor directly under the actual lower person surface. Floor at an
    # unrelated corner of the detector box is not evidence of visible feet.
    contact_columns = np.unique(xs[bottom & (np.abs(body_height)<=.12)])
    supported = 0
    for x in contact_columns:
        edge = int(np.flatnonzero(surface[:,x])[-1])
        supported += bool(floor[edge+1:min(h,edge+21),x].any())
    local_floor = len(contact_columns)>=3 and supported/len(contact_columns)>=.2
    feet_checked=bool(pose.get('fraction',1.)<=.25 and ground_contact and local_floor
                      and not surface_evidence['lower_continuation']
                      and not surface_evidence['image_bottom_clipped'])
    return dict(distance_m=distance,uncertainty_m=error,lower_m=max(0,distance-error),upper_m=distance+error,
                feet_checked=feet_checked,source='metric_depth_floor_geometry',
                surface_evidence=surface_evidence,
                validated=calibration.get('validated') is True,
                next_view='hold' if feet_checked else 'lower')


class MetricDepthBackend:
    def __init__(self,device='mps'):
        self.device=device;self.model=None

    def infer(self,image):
        import torch
        if self.model is None:
            from transformers import AutoImageProcessor,AutoModelForDepthEstimation
            self.processor=AutoImageProcessor.from_pretrained(MODEL)
            self.model=AutoModelForDepthEstimation.from_pretrained(MODEL).to(self.device).eval()
        inputs=self.processor(images=image,return_tensors='pt').to(self.device)
        with torch.inference_mode():
            depth=self.model(**inputs).predicted_depth
            depth=torch.nn.functional.interpolate(depth[:,None],size=(image.height,image.width),mode='bilinear',align_corners=False)
        return depth[0,0].cpu().numpy()


class PersonRangeService:
    def __init__(self,segmentation,backend=None,clock=time.monotonic,nominal_lower_height_m=.15):
        self.segmentation=segmentation;self.backend=backend or MetricDepthBackend(segmentation.device)
        self.lock=threading.Lock()
        self.clock=clock
        self.floor_pose_cache=[]
        self.scale_reference=None
        # Historical owner measurement, not a measurement of each newly
        # selected semantic lower view. Report it so range errors are traceable.
        self.nominal_lower_height_m=nominal_lower_height_m

    def automatic_pose(self, depth, floor, camera, head, motion_epoch=None):
        """Fit this view; a brief floor occlusion may reuse the same head pose.

        The configured lower-view height anchors scale at that view only.
        Other views inherit that correction and estimate their own height.
        This prior and plane consistency never establish validated accuracy.
        """
        reference=head.get('reference_id')
        if not reference or head.get('moving') or head.get('homing'):
            raise ValueError('Wait for a known stopped camera view')
        fraction=head_fraction(head)
        position=head['position']
        now=self.clock()
        nominal_height=camera.get('nominal_lower_height_m',self.nominal_lower_height_m)
        if type(nominal_height) not in (int,float) or not math.isfinite(nominal_height) or not .03<=nominal_height<=.5:
            raise ValueError('Configure a nominal camera height for the lower view')
        camera_key=repr((camera.get('intrinsics'),camera.get('distortion'),camera.get('image_size'),nominal_height))
        if self.scale_reference and (self.scale_reference['reference']!=reference or self.scale_reference['camera_key']!=camera_key):
            self.scale_reference=None
        if fraction>.25 and self.scale_reference is None:
            raise ValueError('Look lower once to initialize the camera distance estimate')
        self.floor_pose_cache=[p for p in self.floor_pose_cache
                               if p['reference']==reference and p['camera_key']==camera_key
                               and p['motion_epoch']==motion_epoch
                               and 0<=now-p['at']<=30]
        try:
            fit=floor_plane_orientation(depth,floor,camera['intrinsics'],camera.get('distortion',()))
            if fraction<=.25:
                self.scale_reference=dict(reference=reference,camera_key=camera_key,
                                          scale=nominal_height/fit['plane_distance_model_units'])
            scale=self.scale_reference['scale']
            height=fit['plane_distance_model_units']*scale
            if not .03<=height<=1.:
                raise ValueError('Estimated floor geometry is inconsistent with the low camera')
            pose=dict(height_m=height,pitch_degrees=fit['pitch_degrees'],front_offset_m=0.,
                      fraction=fraction,depth_scale=scale,nominal_lower_height_m=nominal_height,
                      floor_plane_model_units=fit['plane_distance_model_units'],
                      floor_inlier_fraction=fit['floor_inlier_fraction'],
                      floor_residual_fraction=fit['floor_residual_fraction'],
                      pose_source='current_floor_estimate')
            self.floor_pose_cache=[p for p in self.floor_pose_cache if abs(p['position']-position)>3]
            self.floor_pose_cache.append(dict(reference=reference,camera_key=camera_key,
                                              motion_epoch=motion_epoch,position=position,at=now,pose=pose))
            self.floor_pose_cache=self.floor_pose_cache[-16:]
            return pose
        except ValueError:
            cached=[p for p in self.floor_pose_cache if abs(p['position']-position)<=3]
            if not cached:raise
            return dict(max(cached,key=lambda p:p['at'])['pose'],fraction=fraction,
                        pose_source='recent_same_view_floor_estimate')

    def estimate(self,request):
        from PIL import Image
        jpeg=base64.b64decode(request['image'],validate=True);binding=request['binding']
        if hashlib.sha256(jpeg).hexdigest()!=binding.get('image_sha256'):raise ValueError('Range image binding changed')
        calibration=request.get('calibration') or {}
        approximate=request.get('mode')=='approximate' and calibration.get('validated') is not True
        image=Image.open(io.BytesIO(jpeg)).convert('RGB')
        camera=request.get('camera') if approximate else calibration
        if not isinstance(camera,dict) or list(image.size)!=camera.get('image_size'):
            raise ValueError('Range camera intrinsics resolution changed')
        if not self.lock.acquire(False):raise ValueError('Range processing busy')
        try:
            depth=self.backend.infer(image)
            masks=self.segmentation.infer(jpeg,return_masks=True)
            if approximate:
                try:
                    pose=self.automatic_pose(depth,masks['floor'],camera,request['head'],binding.get('epoch'))
                except ValueError as exc:
                    return dict(ok=True,binding=binding,range=dict(available=False,
                        validated=False,mode='approximate',source=APPROXIMATE_SOURCE,
                        next_view='lower',reason=str(exc)))
                calibration=dict(camera,depth_scale=pose['depth_scale'],near_error_m=.15,
                                 far_relative_error=.20,validated=False)
            else:
                pose=camera_pose(calibration,request['head'])
            try:
                result=estimate_range(depth,masks['person'],masks['floor'],request['box'],calibration,pose)
            except ValueError as exc:
                if not approximate:raise
                return dict(ok=True,binding=binding,range=dict(available=False,validated=False,
                    mode='approximate',source=APPROXIMATE_SOURCE,next_view='lower',reason=str(exc)))
            if approximate:
                result.update(available=True,validated=False,mode='approximate',source=APPROXIMATE_SOURCE,
                    distance_reference='camera_ground_projection',front_offset_measured=False,
                    uncertainty_kind='heuristic_not_validated_accuracy',pose_source=pose['pose_source'],
                    scale_source='nominal_lower_height_reference',
                    nominal_lower_height_m=pose['nominal_lower_height_m'],
                    geometry=dict(
                        head_reference_id=request['head']['reference_id'],
                        head_position=request['head']['position'],
                        head_fraction=pose['fraction'],
                        height_m=pose['height_m'],
                        height_source='nominal_prior_not_current_measurement',
                        pitch_degrees=pose['pitch_degrees'],
                        depth_scale=pose['depth_scale'],
                        floor_plane_model_units=pose['floor_plane_model_units'],
                        floor_inlier_fraction=pose['floor_inlier_fraction'],
                        floor_residual_fraction=pose['floor_residual_fraction'],
                        person_forward_model_units=result['distance_m']/pose['depth_scale']))
            return dict(ok=True,binding=binding,range=result)
        finally:self.lock.release()
