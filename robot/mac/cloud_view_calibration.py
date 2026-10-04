"""Bind cloud advice to cached stationary views; never approve route clearance."""
import hashlib
import json
import threading
import time
from collections import OrderedDict
from robot.mac.vision_scene_advisor import GeminiVisionSceneAdvisor, GroqAccessError
from robot.mac.camera_setup_pool import CameraCloudError
from robot.mac.view_reference import match_view_reference


class CloudViewCalibration:
    def __init__(self, advisor=None, clock=time.monotonic, matcher=match_view_reference):
        self.advisor = advisor
        self.clock, self.matcher = clock, matcher
        self.lock = threading.Lock()
        self.frames = OrderedDict()
        self.references = {}
        self.last_quota = {}
        self.retry_at = 0

    def remember(self, payload, head):
        if (not head.get('homed') or head.get('moving') or head.get('homing')
                or not head.get('reference_id') or head.get('manual_override')
                or head.get('reference_id') != head.get('approved_reference_id')):
            return
        digest = hashlib.sha256(payload).hexdigest()
        with self.lock:
            self.frames[digest] = dict(payload=payload, reference_id=head['reference_id'],
                                       position=int(head['position']), captured_at=self.clock())
            self.frames.move_to_end(digest)
            while len(self.frames) > 64:
                self.frames.popitem(last=False)

    def analyze(self, request, current_position=None):
        if self.clock() < self.retry_at:
            raise CameraCloudError(429, self.retry_at-self.clock())
        if not isinstance(request, dict) or set(request) != {'reference_id', 'frame_sha256'}:
            raise ValueError('Cloud calibration requires a reference and three cached frame IDs')
        reference, digests = request['reference_id'], request['frame_sha256']
        if (not isinstance(reference,str) or not reference or not isinstance(digests,list)
                or len(digests)!=3 or any(not isinstance(d,str) for d in digests) or len(set(digests))!=3):
            raise ValueError('Select three distinct frames from the upward sweep')
        with self.lock:
            frames = [self.frames.get(d) for d in digests]
        if any(f is None or f['reference_id']!=reference or self.clock()-f['captured_at']>600 for f in frames):
            raise ValueError('Scene frames expired or belong to another camera reference; recapture them')
        positions = [f['position'] for f in frames]
        if not positions[0] > positions[1] > positions[2]:
            raise ValueError('Scene frames must follow a strictly upward camera sweep')
        if current_position is not None and abs(max(frames, key=lambda f: f['captured_at'])['position'] - current_position) > 3:
            raise ValueError('Keep the camera at the most recently captured position')
        if self.advisor is None:
            self.advisor = GeminiVisionSceneAdvisor()
        advisor = self.advisor
        try:
            chronological = all(a['captured_at'] < b['captured_at'] for a,b in zip(frames,frames[1:]))
            result = advisor.interpret([f['payload'] for f in frames], upward_sequence=chronological)
        except (GroqAccessError, CameraCloudError) as exc:
            self.last_quota = exc.quota
            if exc.status == 429:
                try:
                    delay = float(exc.quota.get('retry-after', 60))
                except (ValueError, TypeError):
                    delay = 60
                self.retry_at = self.clock() + max(60, delay)
            raise
        observations = result['observations']
        self.last_quota = result.get('quota',{})
        # Require a coherent floor -> room -> overhead sequence. Unknown never
        # becomes an affirmative ceiling label, regardless of model prose.
        verified = (result.get('provider') in ('gemini','groq') and result.get('frame_sha256')==digests
                    and observations[0]['view']=='floor_room'
                    and observations[0]['floor_visible']=='yes'
                    and observations[1]['view'] in ('room','floor_room')
                    and observations[2]['view']=='ceiling'
                    and observations[2]['ceiling_visible']=='yes'
                    and observations[2]['floor_visible']=='no')
        token = hashlib.sha256(json.dumps(dict(model=result['model'],frames=digests,observations=observations),sort_keys=True).encode()).hexdigest()
        return dict(result, ok=True, reference_id=reference, positions=positions,
                    upper_verified=verified, advice_id=token,
                    reference_payloads=[f['payload'] for f in frames])

    def accept(self, result):
        # Service has verified unchanged stopped pose/reference during inference.
        if result['upper_verified']:
            with self.lock:
                self.references = {role:dict(payload=payload,reference_id=result['reference_id'],
                    advice_id=result['advice_id'],model=result['model'],provider=result['provider'])
                    for role,payload in zip(('floor','room','overhead'),result['reference_payloads'])}

    def match(self, payload, head):
        with self.lock:
            references = dict(self.references)
        matches=[]
        for role,reference in references.items():
            if reference['reference_id'] != head.get('reference_id'):
                continue
            result=self.matcher(payload,reference['payload'])
            if result.get('verified'):
                matches.append(dict(result,provider=reference['provider'],label_source=reference['provider'],role=role,
                    reference_id=reference['reference_id'],advice_id=reference['advice_id'],model=reference['model']))
        return matches[0] if len(matches)==1 else {'verified':False}
