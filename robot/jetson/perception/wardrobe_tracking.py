"""Long-lived clothing tracker. Durable references only come from face anchors."""
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import time
import uuid
from urllib.request import Request, urlopen
from urllib.error import HTTPError

try:
    from .family_faces import iou
except ImportError:
    from family_faces import iou


def descriptor(image):
    import cv2
    import numpy as np
    if image.size == 0 or min(image.shape[:2]) < 8: return None
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    if hsv[:,:,2].mean()<12: return None
    result={}
    for name,lo,hi in [('upper',.15,.5),('lower',.5,.85),('footwear',.85,1.),('whole',.1,.9)]:
        h,w=image.shape[:2];crop=hsv[int(lo*h):max(int(hi*h),int(lo*h)+1),int(w*.15):max(int(w*.85),1)]
        hist=cv2.calcHist([crop],[0,1],None,[12,4],[0,180,0,256]).ravel()
        hist=(hist/max(float(hist.sum()),1)).tolist()
        gray=crop[:,:,2];edges=cv2.Laplacian(gray,cv2.CV_32F)
        result[name]=dict(colour=hist,texture=min(1.,float(np.mean(np.abs(edges)))/50))
    return result


def similarity(a,b):
    if not a or not b:return 0.
    scores=[]
    for region in set(a)&set(b):
        x,y=a[region],b[region]
        colour=sum(math.sqrt(max(0.,u*v)) for u,v in zip(x['colour'],y['colour']))
        scores.append(.9*colour+.1*(1-abs(x['texture']-y['texture'])))
    return sum(scores)/len(scores) if scores else 0.


def appearance_bands(image):
    """Unlabelled vertical strips for matching a cropped view to a known view."""
    import cv2
    import numpy as np
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    h,w=hsv.shape[:2];result=[]
    for index in range(8):
        crop=hsv[index*h//8:max(index*h//8+1,(index+1)*h//8),w//5:max(w//5+1,4*w//5)]
        colour=cv2.calcHist([crop],[0,1],None,[12,4],[0,180,0,256]).ravel()
        brightness=cv2.calcHist([crop],[2],None,[8],[0,256]).ravel()
        result.append(dict(colour=(colour/max(float(colour.sum()),1)).tolist(),
                           brightness=(brightness/max(float(brightness.sum()),1)).tolist(),
                           texture=min(1.,float(np.abs(cv2.Laplacian(crop[:,:,2],cv2.CV_32F)).mean())/50)))
    return result


def partial_view_similarity(a,b):
    """Align ordered strips, retaining at least half of the reference view.

    Brightness distinguishes pale trousers from dark clothing with similar hue.
    These signatures complement face/cloud-confirmed references for continuous tracking.
    """
    if not a or not b:return 0.
    def score(x,y):
        overlap=lambda key:sum(math.sqrt(max(0.,u*v)) for u,v in zip(x[key],y[key]))
        return .65*overlap('colour')+.25*overlap('brightness')+.1*(1-abs(x['texture']-y['texture']))
    best=0.
    for current,reference in ((a,b),(b,a)):
        for length in range(4,9):
            for start in range(9-length):
                pairs=[score(current[i],reference[start+min(length-1,int((i+.5)*length/8))]) for i in range(8)]
                best=max(best,sum(pairs)/8)
    return best


def post_json(url, payload):
    request=Request(url,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    try:
        with urlopen(request,timeout=10) as response:
            return json.load(response)
    except HTTPError as exc:
        delay=5.
        try:
            value=json.loads(exc.read(8192))
            delay=float((value.get('quota') or {}).get('retry-after',5))
        except (ValueError,TypeError):pass
        finally:exc.close()
        error=RuntimeError('Vision service temporarily unavailable')
        error.retry_after=max(5.,min(86400.,delay)) if math.isfinite(delay) else 5.
        raise error from None


class WardrobeTracker:
    def __init__(self, store, url, transport=post_json):
        self.store,self.url,self.transport=store,url,transport
        self.tracks={};self.epoch=0;self.future=None;self.pending=None
        self.executor=ThreadPoolExecutor(max_workers=1)
        self.last_error=None;self.last_cloud_at=-100.;self.retry_at=-100.
        self.last_result=None
        self.references=[];self.catalog_at=-100.

    def close(self):
        self.executor.shutdown(wait=False,cancel_futures=True)

    def invalidate_position(self):
        self.epoch+=1
        for t in self.tracks.values():
            t['position_valid']=False
            t['range']=None
            t['visible_regions']=[]
            t['partial_support']=0

    def observe(self,image,people,faces,frame_key,now):
        """Inputs are from one source frame, boxes in pixel coordinates."""
        import cv2
        if now-self.catalog_at>2:
            self.references=self.store.wardrobe(images=True);self.catalog_at=now
        self.poll(now)
        h,w=image.shape[:2]
        assigned=set()
        # A crossing/merge cannot donate either person's identity to a new box.
        for box in people:
            x1,y1,x2,y2=map(int,box)
            x1,y1=max(0,x1),max(0,y1);x2,y2=min(w,x2),min(h,y2)
            if x2-x1<8 or y2-y1<8:continue
            box=[x1,y1,x2,y2]
            crop=image[y1:y2,x1:x2];desc=descriptor(crop)
            if desc is None:continue
            bands=appearance_bands(crop)
            candidates=[]
            for tid,t in self.tracks.items():
                if tid in assigned or now-t['seen_at']>20:continue
                appearance=similarity(desc,t['reference_descriptor'])
                # A missing detection invalidates the current location, but its
                # recent box still helps reacquire in an unchanged camera view.
                # Motor/camera movement changes epoch and removes that cue.
                same_view=t.get('epoch')==self.epoch and now-t['seen_at']<=2.
                spatial_available=t['position_valid'] or same_view
                spatial=iou(box,t['box']) if spatial_available else 0.
                if appearance>=.78 and (spatial>=.15 or not t['position_valid']):
                    candidates.append((.7*appearance+.3*spatial,tid,False,spatial_available))
                elif (len(people)==1 and t.get('profile_id') and now-t['seen_at']<=10
                      and (not same_view or t.get('partial_support') or y1<=2 or y2>=h-2)):
                    partial=partial_view_similarity(bands,t.get('reference_bands'))
                    if partial>=.90:candidates.append((.7*partial,tid,True,False))
            # After movement, all old positions are unavailable. Normalize that
            # appearance-only comparison together; do not boost an old candidate
            # against one that still has fresh spatial evidence.
            if candidates and not any(c[3] for c in candidates):
                candidates=[(score/.7,tid,partial,spatial) for score,tid,partial,spatial in candidates]
            candidates.sort(reverse=True)
            unique=candidates and (len(candidates)==1 or candidates[0][0]-candidates[1][0]>.08)
            tid=candidates[0][1] if unique else uuid.uuid4().hex
            if not unique:
                for _,other,_,_ in candidates:self.tracks[other]['position_valid']=False
            t=self.tracks.setdefault(tid,dict(id=tid,profile_id=None,revision=None,identity_source=None,
                reference_descriptor=desc,hits=0,last_request_signature=None,range=None))
            assigned.add(tid)
            t.update(box=list(box),seen_at=now,frame_key=frame_key,position_valid=True,epoch=self.epoch,descriptor=desc)
            t['bands']=bands
            t['partial_support']=t.get('partial_support',0)+1 if unique and candidates[0][2] else 0
            t['hits']+=1
            associated=[]
            for face in faces:
                b=face['box'];area=max(1.,(b[2]-b[0])*(b[3]-b[1]))
                overlap=max(0,min(x2,b[2])-max(x1,b[0]))*max(0,min(y2,b[3])-max(y1,b[1]))/area
                # Face must belong to exactly one current body.
                containing=sum(max(0,min(p[2],b[2])-max(p[0],b[0]))*max(0,min(p[3],b[3])-max(p[1],b[1]))/area>=.8 for p in people)
                if face.get('confirmed') and overlap>=.8 and containing==1:associated.append(face)
            face=associated[0] if len(associated)==1 else None
            if face:
                if t['profile_id']!=face['profile_id'] or t['revision']!=face['revision']:
                    t['last_request_signature']=None
                    t.pop('outfit_id',None)
                    t['range']=None
                t.update(profile_id=face['profile_id'],revision=face['revision'],identity_source='face',
                         reference_descriptor=desc,reference_bands=bands,partial_support=0,face_at=now,face_box=face['box'])
            elif t.get('profile_id'):
                t['identity_source']='tracking'
            # Crop below a current face so wardrobe retention does not retain faces.
            cut=max(0,int(face['box'][3])-y1) if face else int(.25*crop.shape[0])
            clothing=crop[min(cut,crop.shape[0]-8):]
            clothing=cv2.resize(clothing,(max(8,int(clothing.shape[1]*min(1.,320/clothing.shape[0]))),min(320,clothing.shape[0])))
            ok,jpeg=cv2.imencode('.jpg',clothing,[cv2.IMWRITE_JPEG_QUALITY,82])
            if ok and t['hits']>=2:self.schedule(t,jpeg.tobytes(),face,now)
        for tid,t in self.tracks.items():
            if tid not in assigned:t['position_valid']=False
        self.tracks={k:v for k,v in self.tracks.items() if now-v['seen_at']<=30}

    def schedule(self,t,jpeg,face,now):
        if self.future is not None or now<self.retry_at or now-self.last_cloud_at<3:return
        if t['profile_id'] and not face:return
        scores=[]
        for r in self.references:
            views=r.get('views') or [dict(image=image,descriptors=r['descriptors']) for image in r.get('images',[])]
            if not views:continue
            view=max(views,key=lambda v:similarity(t['descriptor'],v['descriptors']))
            scores.append((similarity(t['descriptor'],view['descriptors']),dict(r,comparison_image=view['image'])))
        scores.sort(key=lambda x:x[0],reverse=True)
        eligible=[r for score,r in scores if score>=.45 and r.get('images')]
        references=eligible[:1]
        if references:
            other=next((r for r in eligible[1:] if r['profile_id']!=references[0]['profile_id']),None)
            references += [other] if other else eligible[1:2]
        # Remember once per distinct face-confirmed outfit; local comparisons
        # never silently merge outfits into permanent ownership records.
        signature=(self.epoch,t['profile_id'],tuple(r['id'] for r in references),
                   tuple((region,tuple(round(x,1) for x in values['colour']),round(values['texture'],1))
                         for region,values in sorted(t['descriptor'].items())))
        if signature==t['last_request_signature']:return
        operation='compare' if references else 'describe'
        if not face and operation=='describe':return
        binding=dict(request_id=uuid.uuid4().hex,track_id=t['id'],profile_id=t['profile_id'],frame_key=t['frame_key'],
            revision=t['revision'] or 'unidentified',epoch=self.epoch,image_sha256=hashlib.sha256(jpeg).hexdigest())
        request=dict(operation=operation,binding=binding,image=base64.b64encode(jpeg).decode(),
                     references=[dict(id=r['id'],image=r['comparison_image']) for r in references])
        self.pending=dict(binding=binding,references=references,face=face,jpeg=jpeg,descriptor=t['descriptor'],
                          bands=t.get('bands'),signature=signature)
        self.future=self.executor.submit(self.transport,self.url,request)
        self.last_result=dict(state='pending',track_id=t['id'],request_id=binding['request_id'])
        t['last_request_signature']=signature
        self.last_cloud_at=now

    def poll(self,now):
        if self.future is None or not self.future.done():return
        future,p=self.future,self.pending
        waiting_for_body=False
        self.last_result=dict(state='received',track_id=p['binding']['track_id'],
                              request_id=p['binding'].get('request_id'))
        try:
            result=future.result()
            if now-self.last_cloud_at>10:
                self.last_result['state']='expired';return
            if not result.get('ok') or result.get('binding')!=p['binding']:raise ValueError('Wardrobe binding mismatch')
            t=self.tracks.get(p['binding']['track_id'])
            if not t or self.epoch!=p['binding']['epoch'] or t['epoch']!=p['binding']['epoch']:
                self.last_result['state']='track_or_view_changed';return
            if (t.get('profile_id')!=p['binding'].get('profile_id')
                    or (t.get('revision') or 'unidentified')!=p['binding'].get('revision')):
                self.last_result['state']='identity_changed'
                return  # A newer face result always wins over a delayed clothing reply.
            if not t['position_valid'] or now-t['seen_at']>1:
                # Keep a completed answer inside its original ten-second window
                # until a fresh body observation reconnects it to this track.
                waiting_for_body=True;self.last_result['state']='waiting_for_body';return
            answer=result['interpretation'];match=next((r for r in p['references'] if r['id']==answer.get('reference_id')),None)
            self.last_result.update(state='evaluated',decision=answer.get('decision'),model=result.get('model'))
            t['visible_regions']=[r['region'] for r in answer['regions'] if r.get('visible')]
            t['next_view']=answer.get('next_view','uncertain')
            t['advice_at']=now
            if p['face']:
                face=p['face'];profile=self.store.load(face['profile_id'])
                if not profile or profile['revision']!=face['revision']:return
                oid=match['id'] if match and answer['decision']=='match' and match['profile_id']==face['profile_id'] else None
                # Ambiguity cannot merge records, but a real face can teach a new outfit.
                self.store.remember(face['profile_id'],face['revision'],answer['regions'],p['descriptor'],p['jpeg'],
                    dict(source='face',confirmed=True,frame_key=p['binding']['frame_key']),oid)
                self.catalog_at=-100.
                self.last_result['state']='outfit_saved'
            elif answer.get('decision')=='match' and match and answer.get('supporting_regions') and not answer.get('conflicting_regions'):
                profile=self.store.load(match['profile_id'])
                if not profile or profile['revision']!=match['revision']:return
                if match['id'] not in {r['id'] for r in self.store.wardrobe(match['profile_id'])}:return
                # Near-identical references owned by different people stay ambiguous.
                compared={r['id'] for r in p['references']}
                competing=[r for r in self.references if r['id'] not in compared and r['profile_id']!=match['profile_id']
                           and similarity(r['descriptors'],match['descriptors'])>.95]
                if competing:
                    t['last_request_signature']=p['signature']
                    t['next_view']='uncertain'
                    return
                if similarity(t['descriptor'],p['descriptor'])<.78:return
                t.update(profile_id=match['profile_id'],revision=match['revision'],identity_source='clothing',
                         reference_descriptor=p['descriptor'],reference_bands=p.get('bands'),partial_support=0,outfit_id=match['id'])
                self.last_result['state']='clothing_matched'
            t['last_request_signature']=p['signature'];self.last_error=None
        except Exception as exc:
            self.last_result['state']='unavailable'
            self.last_error=type(exc).__name__+': '+str(exc)[:160];self.retry_at=now+getattr(exc,'retry_after',5)
        finally:
            if not waiting_for_body:self.future=self.pending=None

    def selected(self,pid,now):
        profile=self.store.load(pid)
        if not profile:return None
        tracks=[t for t in self.tracks.values() if t['profile_id']==pid and t['revision']==profile['revision']
                and t['position_valid'] and now-t['seen_at']<=1. and t.get('partial_support')!=1]
        outfit_ids={r['id'] for r in self.references}
        tracks=[t for t in tracks if not t.get('outfit_id') or t['outfit_id'] in outfit_ids]
        return dict(tracks[0]) if len(tracks)==1 else None

    def guidance(self,now,profile_id=None):
        candidates=[t for t in self.tracks.values() if t.get('position_valid') and 0<=now-t['seen_at']<=1.
                    and (not t.get('profile_id') or t['profile_id']==profile_id)]
        if len(candidates)!=1:return None
        t=candidates[0]
        pending=self.pending and self.pending['binding']['track_id']==t['id'] and self.pending['binding']['epoch']==t['epoch'] and now-self.last_cloud_at<=10
        return dict(track_id=t['id'],frame_key=t['frame_key'],box=t['box'],age_seconds=now-t['seen_at'],
            pending=bool(pending),next_view=t.get('next_view','uncertain') if now-t.get('advice_at',-100)<=5 else 'uncertain')
