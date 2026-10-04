"""Geometric match to an operator-labelled view, not a general scene classifier."""
import hashlib


def match_view_reference(image_bytes, reference_bytes):
    import cv2
    import numpy as np
    current = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    reference = cv2.imdecode(np.frombuffer(reference_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    if current is None or reference is None:
        raise ValueError('Reference and current frame must be valid images')
    result = {'verified': False, 'reference_sha256': hashlib.sha256(reference_bytes).hexdigest()}
    detector = cv2.ORB_create(nfeatures=1800, fastThreshold=10)
    a, da = detector.detectAndCompute(current, None)
    b, db = detector.detectAndCompute(reference, None)
    if da is None or db is None:
        return dict(result, reason='not enough features')
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(da, db, k=2)
    matches = [pair[0] for pair in pairs if len(pair)==2 and pair[0].distance < .70*pair[1].distance]
    if len(matches)<24:
        return dict(result, reason='not enough unambiguous matches', matches=len(matches))
    pa = np.float32([a[m.queryIdx].pt for m in matches])
    pb = np.float32([b[m.trainIdx].pt for m in matches])
    transform, mask = cv2.findHomography(pa, pb, cv2.RANSAC, 3.0)
    if transform is None or mask is None or not np.all(np.isfinite(transform)):
        return dict(result, reason='no consistent geometry')
    projected = cv2.perspectiveTransform(pa.reshape(-1, 1, 2), transform).reshape(-1, 2)
    # Use the final fitted geometry rather than version-dependent RANSAC masks.
    inliers = np.isfinite(projected).all(axis=1) & (np.linalg.norm(projected - pb, axis=1) <= 3.0)
    h,w = current.shape;rh,rw=reference.shape
    coverage=np.ptp(pa[inliers],axis=0)/np.array([w,h]) if inliers.any() else np.zeros(2)
    center=cv2.perspectiveTransform(np.float32([[[w/2,h/2]]]),transform)[0,0]
    offset=(center-np.array([rw/2,rh/2]))/np.array([rw,rh])
    corners=cv2.perspectiveTransform(np.float32([[[0,0],[w,0],[w,h],[0,h]]]),transform)[0]
    area=abs(cv2.contourArea(corners))/(rw*rh)
    ratio=float(inliers.mean())
    verified=bool(inliers.sum()>=24 and ratio>=.60 and np.all(coverage>=.30)
                  and np.all(np.abs(offset)<=.20) and .6<=area<=1.6)
    return dict(result, verified=verified, inliers=int(inliers.sum()), inlier_ratio=ratio,
                feature_coverage=coverage.tolist(), center_offset=offset.tolist(), projected_area_ratio=area,
                reason='matching view' if verified else 'different view or weak geometry')
