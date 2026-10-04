"""Diagnostic depth correction from independent geometry; never motor control.

The depth model's floor normal cannot establish its own metric accuracy.
With independently supplied camera geometry, fit scale AND offset in inverse
depth to floor samples and evaluate person samples separately.
"""
import math


def sample_rays(samples, camera):
    import cv2
    import numpy as np
    values=np.asarray(samples,dtype=float)
    if values.ndim!=2 or values.shape[1]!=3 or not np.isfinite(values).all():
        raise ValueError('Expected finite pixel-x, pixel-y, depth samples')
    width,height=camera['image_size'];intrinsics=camera['intrinsics']
    fx,fy,cx,cy=[float(intrinsics[k]) for k in ('fx','fy','cx','cy')]
    if not all(math.isfinite(v) for v in (fx,fy,cx,cy)) or min(fx,fy)<=0:
        raise ValueError('Invalid calibrated intrinsics')
    if (np.any(values[:,0]<0) or np.any(values[:,0]>=width)
            or np.any(values[:,1]<0) or np.any(values[:,1]>=height)
            or np.any(values[:,2]<=0)):
        raise ValueError('Samples lie outside the image or have invalid depth')
    matrix=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    distortion=np.asarray(camera.get('distortion',[]),dtype=float)
    rays=cv2.undistortPoints(values[:,:2].reshape(-1,1,2),matrix,
        distortion if len(distortion) else None)[:,0]
    return values,rays


def pose_values(pose):
    values=[pose.get(k) for k in ('height_m','pitch_degrees','roll_degrees')]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        raise ValueError('Independent height, pitch and roll are required')
    h,p,r=values
    if not .03<=h<=1 or not -80<=p<=80 or not -25<=r<=25:
        raise ValueError('Unsupported camera geometry')
    return h,math.radians(p),math.radians(r)


def floor_point_pose(pixel, camera_distance_m, height_m, roll_degrees, camera):
    """A known floor contact determines pitch; distance is camera-forward.

Callers must resolve or explicitly label any robot-front offset assumption.
An occluded/clipped foot or a lifted foot is not a known floor contact.
"""
    import numpy as np
    h,_,roll=pose_values(dict(height_m=height_m,pitch_degrees=0,roll_degrees=roll_degrees))
    if type(camera_distance_m) not in (int,float) or not math.isfinite(camera_distance_m) or camera_distance_m<=.05:
        raise ValueError('Supply the distance to this floor contact')
    _,rays=sample_rays([[pixel[0],pixel[1],1.]],camera)
    effective=rays[0,0]*math.sin(roll)+rays[0,1]*math.cos(roll)
    pitch=math.atan2(h,camera_distance_m)-math.atan(effective)
    return dict(height_m=h,pitch_degrees=float(np.degrees(pitch)),roll_degrees=roll_degrees,
                source='operator_floor_point',validated=False)


def fit_floor_correction(samples,camera,pose):
    import numpy as np
    values,rays=sample_rays(samples,camera);height,pitch,roll=pose_values(pose)
    if len(values)<100 or np.ptp(values[:,0])<.2*camera['image_size'][0] or np.ptp(values[:,1])<.08*camera['image_size'][1]:
        raise ValueError('A broader floor sample is required')
    vertical=rays[:,0]*math.sin(roll)+rays[:,1]*math.cos(roll)
    expected=(vertical*math.cos(pitch)+math.sin(pitch))/height
    keep=expected>.05
    raw=1/values[:,2];matrix=np.column_stack((raw,np.ones(len(raw))))
    if keep.sum()<100 or np.ptp(raw[keep])<.02:
        raise ValueError('Floor depth has insufficient variation')
    initial_count=int(keep.sum())
    candidates=np.flatnonzero(keep);rng=np.random.default_rng(11);best=None
    for _ in range(80):
        i,j=rng.choice(candidates,2,replace=False)
        if abs(raw[i]-raw[j])<.02:continue
        alpha=(expected[i]-expected[j])/(raw[i]-raw[j])
        if alpha<=0:continue
        beta=expected[i]-alpha*raw[i]
        inliers=keep&(np.abs(alpha*raw+beta-expected)<np.maximum(.03,.05*np.abs(expected)))
        score=int(inliers.sum())
        if best is None or score>best[0]:best=(score,inliers)
    if best is None or best[0]<max(100,.65*initial_count):
        raise ValueError('Floor does not support this independent geometry')
    keep=best[1]
    for _ in range(5):
        coefficients=np.linalg.lstsq(matrix[keep],expected[keep],rcond=None)[0]
        residual=np.abs(matrix@coefficients-expected)
        threshold=max(.03,3*float(np.median(residual[keep])))
        keep=(expected>.05)&(residual<=threshold)
        if keep.sum()<max(100,.65*initial_count):
            raise ValueError('Floor does not support this independent geometry')
    alpha,beta=(float(x) for x in coefficients)
    if not math.isfinite(alpha+beta) or alpha<=0:
        raise ValueError('Invalid inverse-depth correction')
    relative=np.abs(matrix[keep]@coefficients-expected[keep])/expected[keep]
    if np.median(relative)>.10:
        raise ValueError('Floor correction remains inconsistent')
    return dict(inverse_scale=alpha,inverse_offset=beta,sample_count=int(keep.sum()),
        median_relative_floor_residual=float(np.median(relative)),pose=dict(pose),validated=False)


def corrected_surfaces(samples,camera,correction):
    import numpy as np
    values,rays=sample_rays(samples,camera);height,pitch,roll=pose_values(correction['pose'])
    inverse=correction['inverse_scale']/values[:,2]+correction['inverse_offset']
    # A failed correction cannot discard the nearest pixels and report only
    # whichever distant surfaces happened to remain positive.
    if not np.isfinite(inverse).all() or np.any(inverse<=.02):
        raise ValueError('Correction extrapolates beyond supported person depth')
    z=1/inverse;vertical=rays[:,0]*math.sin(roll)+rays[:,1]*math.cos(roll)
    forward=z*(math.cos(pitch)-vertical*math.sin(pitch))
    if np.any(forward<=0):raise ValueError('Corrected person range is invalid')
    return dict(forward_m=forward,above_floor_m=height-z*(math.sin(pitch)+vertical*math.cos(pitch)))
