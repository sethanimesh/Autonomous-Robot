"""Per-face-track household matching; histories never combine different people."""
from collections import deque
import uuid


def iou(a, b):
    overlap = max(0, min(a[2],b[2])-max(a[0],b[0])) * max(0,min(a[3],b[3])-max(a[1],b[1]))
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-overlap
    return overlap / union if union > 0 else 0.


class FamilyFaceMatcher:
    def __init__(self, threshold=.4, margin=.05, required=2, window=5):
        self.threshold,self.margin,self.required,self.window = threshold,margin,required,window
        self.tracks = {}

    def observe(self, faces, now):
        self.tracks = {k:v for k,v in self.tracks.items() if now-v['at'] <= 1.5}
        used, result = set(), []
        for face in faces:
            choices = [(iou(face['box'],v['box']), k) for k,v in self.tracks.items() if k not in used]
            choices.sort(reverse=True)
            key = choices[0][1] if choices and choices[0][0]>=.2 and (len(choices)==1 or choices[0][0]-choices[1][0]>.1) else uuid.uuid4().hex
            used.add(key)
            track = self.tracks.setdefault(key,dict(history=deque(maxlen=self.window)))
            scores = sorted(face['scores'], key=lambda p:p['score'], reverse=True)
            winner = scores[0] if scores and scores[0]['score']>=self.threshold and (len(scores)==1 or scores[0]['score']-scores[1]['score']>=self.margin) else None
            identity = (winner['profile_id'],winner['revision']) if winner else None
            track['history'].append(identity)
            track.update(box=face['box'],at=now)
            confirmed = identity is not None and sum(x==identity for x in track['history'])>=self.required
            result.append(dict(box=face['box'],face_track_id=key,confirmed=confirmed,
                               profile_id=winner['profile_id'] if confirmed else None,
                               revision=winner['revision'] if confirmed else None))
        return result
