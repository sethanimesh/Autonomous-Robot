"""Stationary range comparisons. Outputs numbers only, never a motor command."""
import base64
import hashlib
import io
import json
import math
from urllib.request import urlopen

from robot.mac.person_range import MODEL, MetricDepthBackend, estimate_range, floor_plane_orientation, head_fraction


def compare_range_models(depth, masks, box, camera, head, *, height_m=None,
                         front_offset_m=None, measured_gap_m=None):
    for name, value, low, high in (
        ('height', height_m, .03, 1.), ('front offset', front_offset_m, -1., 1.),
        ('gap', measured_gap_m, .05, 10.),
    ):
        if value is not None and (type(value) not in (int, float)
                                 or not math.isfinite(value) or not low <= value <= high):
            raise ValueError('Invalid measured ' + name)
    if front_offset_m is not None and height_m is None:
        raise ValueError('Provide lens height with the camera-to-front offset')
    fraction = head_fraction(head)
    fit = floor_plane_orientation(depth, masks['floor'], camera['intrinsics'], camera.get('distortion', []))
    model_height = fit['plane_distance_model_units']

    def estimate(height, offset, origin):
        scale = height/model_height
        calibration = dict(camera, depth_scale=scale, near_error_m=.15,
                           far_relative_error=.20, validated=False)
        pose = dict(height_m=height, pitch_degrees=fit['pitch_degrees'],
                    front_offset_m=offset, fraction=fraction)
        try:
            result = estimate_range(depth, masks['person'], masks['floor'], box, calibration, pose)
            return dict(result, available=True, distance_reference=origin,
                        height_used_m=height, depth_scale=scale)
        except ValueError as exc:
            return dict(available=False, reason=str(exc), validated=False,
                        distance_reference=origin, height_used_m=height, depth_scale=scale)

    cases = dict(raw_model=estimate(model_height, 0., 'camera_ground_projection'),
                 historical_15cm_prior=estimate(.15, 0., 'camera_ground_projection'))
    if height_m is not None:
        cases['measured_height'] = estimate(height_m, 0., 'camera_ground_projection')
        if front_offset_m is not None:
            cases['measured_height_front_gap'] = estimate(height_m, front_offset_m, 'robot_front')
    comparison = dict(available=False, reason='A measured gap, lens height and camera/front offset are needed for a common-origin comparison')
    front = cases.get('measured_height_front_gap') or {}
    if measured_gap_m is not None and front.get('available'):
        error = front['distance_m']-measured_gap_m
        comparison = dict(available=True, measured_gap_m=measured_gap_m,
                          estimated_gap_m=front['distance_m'], signed_error_m=error,
                          point_within_target=abs(error) <= (.10 if measured_gap_m <= 1. else .2*measured_gap_m),
                          formal_accuracy_validation=False)
    return dict(schema=1, model=MODEL, observation_only=True, validated=False,
                floor_fit=fit, head_fraction=fraction, cases=cases, comparison=comparison,
                measurements=dict(height_m=height_m, front_offset_m=front_offset_m,
                                  robot_front_gap_m=measured_gap_m),
                note='One comparison does not validate accuracy across views or distances; no runtime settings are changed.')


def capture_diagnostic(ui_url, *, height_m=None, front_offset_m=None, measured_gap_m=None,
                       backend=None, segmentation=None):
    """The image is transient; only frame binding and numerical results leave here."""
    from PIL import Image
    from robot.mac.route_perception import RoutePerceptionEngine
    with urlopen(ui_url.rstrip('/')+'/api/status', timeout=4) as response:
        status = json.load(response)
    if status.get('mission', {}).get('running') or status.get('manual_drive', {}).get('active'):
        raise ValueError('Finish movement before taking a measured example')
    if not status.get('robot_ready'):
        raise ValueError('A fresh camera-head position is required; leave the EV3 off until the prepared capture')
    with urlopen(ui_url.rstrip('/')+'/api/family/range-frame', timeout=4) as response:
        capture = json.load(response)
    if capture.get('capture_schema') != 1 or not capture.get('pose_binding') or not capture.get('camera'):
        raise ValueError('Activate the updated console before taking a geometry comparison')
    jpeg = base64.b64decode(capture['image'], validate=True)
    if hashlib.sha256(jpeg).hexdigest() != capture['image_sha256']:
        raise ValueError('Measurement image binding changed')
    image = Image.open(io.BytesIO(jpeg)).convert('RGB')
    camera, head, track = capture['camera'], capture['head'], capture['track']
    if list(image.size) != camera['image_size'] or head != capture['pose_binding']['head']:
        raise ValueError('Camera geometry does not match the capture')
    segmentation = segmentation or RoutePerceptionEngine()
    backend = backend or MetricDepthBackend(segmentation.device)
    depth = backend.infer(image)
    masks = segmentation.infer(jpeg, return_masks=True)
    report = compare_range_models(depth, masks, track['body_box'], camera, head,
                                  height_m=height_m, front_offset_m=front_offset_m,
                                  measured_gap_m=measured_gap_m)
    report.update(frame_sha256=capture['image_sha256'], frame_key=track['frame_key'],
                  profile_id=track['profile_id'], track_id=track['track_id'],
                  pose_binding=capture['pose_binding'], camera=camera,
                  images_saved=False, commands_sent=False)
    return report
