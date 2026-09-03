#!/usr/bin/env python3
"""Temporary browser console for live and uploaded target enrollment."""

import argparse
import json
import os
import shutil
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from camera_controls import validate_camera_jog
from face_detections import FaceDetection, select_faces
from face_inference import TensorRTMultiOutputBackend, YuNetFaceDetector
from face_observations import FaceObservationError, parse_face_observations
from face_sync import ExactPairMatcher, stamp_key
from recognition_core import FaceEmbeddingRecognizer, TargetStore, assess_quality, classify_pose, cosine_similarity


BEST_EFFORT = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)
REQUIRED = {"center": 3, "left": 2, "right": 2}
MINIMUM_TOTAL = 10
MAXIMUM_SAMPLES = 18
MAX_UPLOAD_BYTES = 10 * 1024 * 1024
PHOTO_DIRECTORY_PREFIX = "target_photos_"

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Echora Target Enrollment</title><style>
:root{color-scheme:dark;font-family:ui-rounded,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#0b1118;color:#edf4fb}*{box-sizing:border-box}body{margin:0}.wrap{max-width:980px;margin:auto;padding:20px}.hero{display:flex;justify-content:space-between;gap:16px;align-items:end;margin-bottom:16px}h1{font-size:clamp(25px,4vw,40px);margin:0}p{color:#9fb0c1}.pill{padding:7px 12px;border-radius:999px;background:#172330;color:#9fc8ff;font-weight:700}.grid{display:grid;grid-template-columns:1.45fr 1fr;gap:16px}.card{background:#121c27;border:1px solid #263442;border-radius:18px;padding:16px;box-shadow:0 12px 35px #0005}.preview{padding:0;overflow:hidden;background:#000;min-height:320px;display:grid;place-items:center}.preview img{display:block;width:100%;height:auto}.state{font-size:24px;font-weight:800;margin:5px 0}.counts{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:14px 0}.count{background:#0c151e;border-radius:12px;padding:12px;text-align:center}.count b{display:block;font-size:23px}label{display:block;margin:11px 0 5px;color:#afbecd;font-weight:650}input,select,button{font:inherit;width:100%;border-radius:11px;border:1px solid #344658;padding:11px 12px;background:#0b141d;color:white}button{border:0;background:#2388ff;font-weight:800;cursor:pointer;margin-top:10px}button.secondary{background:#263747}button.danger{background:#6c2832}button:disabled{opacity:.45;cursor:not-allowed}.row{display:grid;grid-template-columns:1fr 1fr;gap:9px}.consent{display:flex;align-items:flex-start;gap:8px;font-size:13px;color:#aebdca}.consent input{width:auto;margin-top:3px}.progress{height:8px;border-radius:99px;background:#263441;overflow:hidden}.progress i{display:block;height:100%;background:#35d49a;width:0}.message{min-height:42px;padding:10px 0;color:#84e8bc;font-weight:700}.small{font-size:12px;color:#8393a2}@media(max-width:760px){.grid{grid-template-columns:1fr}.hero{display:block}.preview{min-height:220px}}
</style></head><body><div class="wrap"><div class="hero"><div><h1>Target person enrollment</h1><p>Use uploaded photos and a short live camera session for the most reliable result.</p></div><span class="pill" id="service">Connecting…</span></div><div class="grid"><div class="card preview"><img src="/stream" alt="Live camera preview"></div><div class="card"><div class="small">CAMERA TILT</div><div class="state" id="headState">Checking motor…</div><div class="row"><button class="secondary" id="tiltUp" onclick="jogCamera(5)">Tilt up ▲</button><button class="secondary" id="tiltDown" onclick="jogCamera(-5)">Tilt down ▼</button></div><p class="small">One safe 5° step per click. The tracks stay stopped.</p><div class="small">CURRENT GUIDANCE</div><div class="state" id="guide">Wait for camera</div><div class="progress"><i id="bar"></i></div><div class="counts"><div class="count"><b id="center">0</b>Front</div><div class="count"><b id="left">0</b>Left</div><div class="count"><b id="right">0</b>Right</div></div><div id="message" class="message"></div><label>Target label</label><input id="label" maxlength="40" placeholder="Example: Mom"><label>Photo storage</label><select id="retention"><option value="embeddings">Embeddings only — delete photos</option><option value="face_crops">Keep aligned face crops</option></select><label class="consent"><input type="checkbox" id="consent">I have this person’s permission to create a biometric face template.</label><button id="start" onclick="startEnrollment()">Start new enrollment</button><label>Add clear photographs</label><input id="files" type="file" accept="image/*" multiple><button class="secondary" id="upload" onclick="uploadPhotos()">Add selected photos</button><div class="row"><button class="secondary" id="cancel" onclick="action('/api/enrollment/cancel')">Cancel</button><button id="finish" onclick="action('/api/enrollment/finish')">Finish enrollment</button></div><button class="danger" onclick="deleteTarget()">Delete enrolled target</button><p class="small">The console is temporary and available only while its service is running. No full camera frames are stored.</p></div></div></div>
<script>
const H={'X-Echora-Action':'1'};async function post(url,body,type='application/json'){let h={...H};if(type)h['Content-Type']=type;let r=await fetch(url,{method:'POST',headers:h,body});let j=await r.json();if(!r.ok)throw Error(j.error||'Request failed');return j}
async function action(url){try{await post(url,'{}');await refresh()}catch(e){alert(e.message)}}
async function jogCamera(degrees){try{document.getElementById('tiltUp').disabled=true;document.getElementById('tiltDown').disabled=true;await post('/api/camera/jog',JSON.stringify({degrees}));setTimeout(refresh,350)}catch(e){alert(e.message);await refresh()}}
async function startEnrollment(){let body={label:document.getElementById('label').value,retention:document.getElementById('retention').value,consent:document.getElementById('consent').checked};try{await post('/api/enrollment/start',JSON.stringify(body));await refresh()}catch(e){alert(e.message)}}
async function normalizedBlob(file){let img=await createImageBitmap(file,{imageOrientation:'from-image'});let scale=Math.min(1,1600/Math.max(img.width,img.height));let c=document.createElement('canvas');c.width=Math.round(img.width*scale);c.height=Math.round(img.height*scale);c.getContext('2d').drawImage(img,0,0,c.width,c.height);return await new Promise(ok=>c.toBlob(ok,'image/jpeg',.92))}
async function uploadPhotos(){let fs=[...document.getElementById('files').files];if(!fs.length)return alert('Choose one or more photos first.');for(let f of fs){try{let b=await normalizedBlob(f);let r=await post('/api/enrollment/upload',b,'image/jpeg');document.getElementById('message').textContent=r.message}catch(e){alert(f.name+': '+e.message)}}await refresh()}
async function deleteTarget(){if(!confirm('Delete the enrolled target and any retained face crops?'))return;await action('/api/enrollment/delete')}
async function refresh(){try{let s=await (await fetch('/api/status',{cache:'no-store'})).json();document.getElementById('service').textContent=s.enrolling?'Capturing':'Ready';document.getElementById('guide').textContent=s.guidance;for(let p of ['center','left','right'])document.getElementById(p).textContent=s.pose_counts[p]||0;document.getElementById('bar').style.width=Math.min(100,100*s.sample_count/s.minimum_total)+'%';document.getElementById('message').textContent=s.message||'';document.getElementById('finish').disabled=!s.ready;document.getElementById('upload').disabled=!s.enrolling;document.getElementById('cancel').disabled=!s.enrolling;let h=s.camera_head||{};let usable=h.available&&h.homed&&!h.moving&&!h.homing;document.getElementById('headState').textContent=h.available?(h.homing?'Homing…':h.moving?'Moving…':'Position '+h.position+'°'):'Motor unavailable';document.getElementById('tiltUp').disabled=!usable||h.position>=h.maximum_position;document.getElementById('tiltDown').disabled=!usable||h.position<=h.minimum_position;if(!s.enrolling&&s.target_label&&!document.getElementById('label').value)document.getElementById('label').value=s.target_label}catch(e){document.getElementById('service').textContent='Offline';document.getElementById('tiltUp').disabled=true;document.getElementById('tiltDown').disabled=true}}setInterval(refresh,900);refresh();
</script></body></html>""".encode("utf-8")


class Frame(object):
    __slots__ = ("image", "stamp", "frame_id")
    def __init__(self, image, stamp, frame_id):
        self.image, self.stamp, self.frame_id = image, stamp, frame_id


class EnrollmentSession(object):
    def __init__(self, label, retention):
        self.label = label
        self.retention = retention
        self.samples = []
        self.crops = []
        self.message = "Stand 1–2 metres away and look straight at the camera."
        self.last_capture = 0.0

    @property
    def pose_counts(self):
        return {pose: sum(1 for sample in self.samples if sample["pose"] == pose) for pose in REQUIRED}

    @property
    def ready(self):
        counts = self.pose_counts
        return len(self.samples) >= MINIMUM_TOTAL and all(counts[pose] >= wanted for pose, wanted in REQUIRED.items())

    @property
    def guidance(self):
        counts = self.pose_counts
        if counts["center"] < REQUIRED["center"]:
            return "Look straight at the camera"
        if counts["left"] < REQUIRED["left"] and counts["right"] < REQUIRED["right"]:
            return "Turn your face slightly to either side"
        if counts["left"] < REQUIRED["left"] or counts["right"] < REQUIRED["right"]:
            return "Now turn your face to the other side"
        if len(self.samples) < MINIMUM_TOTAL:
            return "Move naturally for a few more views"
        return "Ready to finish"

    def add(self, embedding, aligned, pose, source, quality):
        if len(self.samples) >= MAXIMUM_SAMPLES:
            self.message = "Enough samples collected. Finish enrollment."
            return False
        if self.samples:
            identity_score = max(
                cosine_similarity(embedding, sample["embedding"])
                for sample in self.samples
            )
            if identity_score < 0.25:
                self.message = "That face does not match the current enrollment."
                return False
        same_pose = [sample["embedding"] for sample in self.samples if sample["pose"] == pose]
        if same_pose and max(cosine_similarity(embedding, item) for item in same_pose) > 0.997:
            self.message = "That view is already covered—change angle slightly."
            return False
        self.samples.append({"embedding": embedding, "pose": pose, "source": source, "quality": quality})
        self.crops.append(aligned.copy())
        self.last_capture = time.monotonic()
        self.message = "Captured {0} view ({1}/{2}).".format(pose, len(self.samples), MINIMUM_TOTAL)
        return True


class EnrollmentNode(Node):
    def __init__(self, args):
        super().__init__("echora_enrollment_console")
        self.args = args
        backend = TensorRTMultiOutputBackend(args.model_path, (112, 112))
        self.recognizer = FaceEmbeddingRecognizer(backend, args.input_mean, args.input_std)
        detector_backend = TensorRTMultiOutputBackend(args.yunet_engine_path, (640, 640))
        self.upload_detector = YuNetFaceDetector(detector_backend, 0.75)
        self.store = TargetStore(args.store_path, args.model_name, args.model_sha256)
        self.matcher = ExactPairMatcher(24)
        self.lock = threading.RLock()
        self.runtime_lock = threading.Lock()
        self.session = None
        self.latest_jpeg = None
        self.latest_frame_time = None
        self.camera_head_status = None
        self.camera_head_status_time = None
        self.last_message = "Start an enrollment or add photographs."
        self.create_subscription(Image, args.image_topic, self.on_image, BEST_EFFORT)
        self.create_subscription(String, args.observations_topic, self.on_observations, 10)
        self.create_subscription(String, args.camera_head_status_topic, self.on_camera_head_status, 10)
        self.camera_head_command_publisher = self.create_publisher(String, args.camera_head_command_topic, 10)

    def on_camera_head_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.camera_head_status = status
            self.camera_head_status_time = time.monotonic()

    def on_image(self, message):
        if message.encoding not in ("bgr8", "rgb8") or len(message.data) != message.height * message.width * 3:
            return
        image = numpy.frombuffer(message.data, numpy.uint8).reshape(message.height, message.width, 3)
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        pair = self.matcher.offer_left(stamp_key(message.header.stamp), Frame(image, message.header.stamp, message.header.frame_id))
        if pair is not None:
            self.process_pair(*pair)

    def on_observations(self, message):
        try:
            observations = parse_face_observations(message.data)
        except FaceObservationError:
            return
        pair = self.matcher.offer_right(observations["key"], observations)
        if pair is not None:
            self.process_pair(*pair)

    def process_pair(self, frame, observations):
        if frame.frame_id != observations["frame_id"]:
            return
        faces = observations["faces"]
        canvas = frame.image.copy()
        with self.lock:
            session = self.session
        for face in faces:
            x1, y1, x2, y2 = [int(round(v)) for v in face["box"]]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 205, 255), 2)
            for point in face["landmarks"]:
                cv2.circle(canvas, tuple(int(round(v)) for v in point), 2, (60, 230, 130), -1)
        if session is not None and time.monotonic() - session.last_capture >= 0.65:
            if len(faces) != 1:
                session.message = "Keep only the target visible." if faces else "No clear face—move into view."
            else:
                accepted, reason, quality = assess_quality(frame.image, faces[0])
                if not accepted:
                    session.message = reason.capitalize() + "."
                else:
                    with self.runtime_lock:
                        embedding, aligned = self.recognizer.embedding(frame.image, faces[0]["landmarks"])
                    with self.lock:
                        if self.session is session:
                            session.add(embedding, aligned, classify_pose(faces[0]["landmarks"]), "live", quality)
        banner = "Enrollment: {0}".format(session.guidance if session else "ready")
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 30), (8, 15, 22), -1)
        cv2.putText(canvas, banner, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            with self.lock:
                self.latest_jpeg = encoded.tobytes()
                self.latest_frame_time = time.monotonic()

    def status(self):
        try:
            target = self.store.load()
        except Exception as exc:
            target = None
            self.last_message = "Stored target error: {0}".format(exc)
        with self.lock:
            session = self.session
            camera_ready = self.latest_frame_time is not None and time.monotonic() - self.latest_frame_time < 2.0
            head_age = None if self.camera_head_status_time is None else time.monotonic() - self.camera_head_status_time
            head_status = None if self.camera_head_status is None else dict(self.camera_head_status)
            if head_status is not None:
                head_status["available"] = head_age <= 2.0
            return {
                "camera_ready": camera_ready,
                "enrolling": session is not None,
                "target_label": None if target is None else target["label"],
                "target_sample_count": 0 if target is None else len(target["samples"]),
                "sample_count": 0 if session is None else len(session.samples),
                "minimum_total": MINIMUM_TOTAL,
                "pose_counts": {pose: 0 for pose in REQUIRED} if session is None else session.pose_counts,
                "ready": False if session is None else session.ready,
                "guidance": "Wait for camera" if not camera_ready else ("Start a new enrollment" if session is None else session.guidance),
                "message": self.last_message if session is None else session.message,
                "model": self.args.model_name,
                "provider": self.recognizer.provider,
                "privacy": "No full frames stored; aligned crops retained only if selected.",
                "camera_head": {"available": False} if head_status is None else head_status,
            }

    def jog_camera(self, request):
        degrees = request.get("degrees")
        if isinstance(degrees, bool) or not isinstance(degrees, (int, float)):
            raise ValueError("Camera tilt step is invalid.")
        degrees = int(degrees)
        with self.lock:
            status = None if self.camera_head_status is None else dict(self.camera_head_status)
            age = None if self.camera_head_status_time is None else time.monotonic() - self.camera_head_status_time
        target = validate_camera_jog(status, age, degrees)
        message = String()
        message.data = json.dumps({"action": "jog", "degrees": degrees}, separators=(",", ":"))
        self.camera_head_command_publisher.publish(message)
        with self.lock:
            self.last_message = "Camera tilt requested: {0}° → {1}°.".format(status["position"], target)
        return self.status()

    def start(self, request):
        label = str(request.get("label", "")).strip()
        retention = request.get("retention", "embeddings")
        if not request.get("consent"):
            raise ValueError("Confirm that the target person has given permission.")
        if not label or len(label) > 40:
            raise ValueError("Enter a target label between 1 and 40 characters.")
        if retention not in ("embeddings", "face_crops"):
            raise ValueError("Invalid photo-storage choice.")
        with self.lock:
            self.session = EnrollmentSession(label, retention)
        return self.status()

    def cancel(self):
        with self.lock:
            self.session = None
            self.last_message = "Enrollment cancelled; nothing was saved."
        return self.status()

    def upload(self, data):
        if not data or len(data) > MAX_UPLOAD_BYTES:
            raise ValueError("Each photo must be smaller than 10 MB.")
        with self.lock:
            session = self.session
        if session is None:
            raise ValueError("Start an enrollment before adding photos.")
        image = cv2.imdecode(numpy.frombuffer(data, numpy.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("The uploaded file is not a readable image.")
        with self.runtime_lock:
            candidates = self.upload_detector.infer(image)
        faces = select_faces(
            [FaceDetection(x1, y1, x2, y2, score, landmarks) for x1, y1, x2, y2, score, landmarks in candidates],
            0.30,
            10,
        )
        if not faces:
            raise ValueError("No clear face was found in that photo.")
        if len(faces) != 1:
            raise ValueError("Use a photo containing exactly one clear face.")
        detected = faces[0]
        face = {
            "box": (detected.x1, detected.y1, detected.x2, detected.y2),
            "landmarks": detected.landmarks,
            "score": detected.score,
        }
        accepted, reason, quality = assess_quality(image, face, minimum_face_pixels=80, minimum_blur=35.0)
        if not accepted:
            raise ValueError("Photo rejected: {0}.".format(reason))
        with self.runtime_lock:
            embedding, aligned = self.recognizer.embedding(image, face["landmarks"])
        with self.lock:
            if self.session is not session:
                raise ValueError("Enrollment was cancelled while the photo was processing.")
            added = session.add(embedding, aligned, classify_pose(face["landmarks"]), "upload", quality)
        return {"ok": True, "added": added, "message": session.message}

    def finish(self):
        with self.lock:
            session = self.session
            if session is None:
                raise ValueError("No enrollment is in progress.")
            if not session.ready:
                raise ValueError("Collect at least 10 good views, including front, left, and right angles.")
            data_root = os.path.dirname(self.args.store_path)
            photo_directory = None
            if session.retention == "face_crops":
                photo_directory = PHOTO_DIRECTORY_PREFIX + "{0:x}".format(time.time_ns())
                photo_root = os.path.join(data_root, photo_directory)
                os.makedirs(photo_root, mode=0o700)
                os.chmod(photo_root, 0o700)
                for index, (sample, crop) in enumerate(zip(session.samples, session.crops), 1):
                    name = "face_{0:02d}.jpg".format(index)
                    path = os.path.join(photo_root, name)
                    if not cv2.imwrite(path, crop, [cv2.IMWRITE_JPEG_QUALITY, 94]):
                        raise RuntimeError("Could not retain an aligned face crop.")
                    os.chmod(path, 0o600)
                    sample["photo"] = photo_directory + "/" + name
            self.store.save(session.label, session.samples, session.retention)
            self.cleanup_photo_directories(keep=photo_directory)
            self.session = None
            self.last_message = "Target enrolled successfully with {0} diverse views.".format(len(session.samples))
        return self.status()

    def delete_target(self):
        self.store.delete()
        self.cleanup_photo_directories()
        with self.lock:
            self.session = None
            self.last_message = "Enrolled target and retained face crops deleted."
        return self.status()

    def cleanup_photo_directories(self, keep=None):
        data_root = os.path.abspath(os.path.dirname(self.args.store_path))
        try:
            names = os.listdir(data_root)
        except OSError:
            return
        for name in names:
            if not name.startswith(PHOTO_DIRECTORY_PREFIX) or name == keep:
                continue
            path = os.path.abspath(os.path.join(data_root, name))
            if os.path.dirname(path) == data_root and os.path.isdir(path):
                shutil.rmtree(path)

    def close(self):
        self.recognizer.close()
        self.upload_detector.close()


class Handler(BaseHTTPRequestHandler):
    node = None
    def log_message(self, *_args):
        pass
    def json_response(self, payload, status=200):
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
    def do_GET(self):
        if self.path == "/api/status":
            self.json_response(self.node.status()); return
        if self.path.startswith("/stream"):
            self.stream(); return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(PAGE)))
        self.end_headers(); self.wfile.write(PAGE)
    def do_POST(self):
        if self.headers.get("X-Echora-Action") != "1":
            self.json_response({"error": "Missing action header."}, 403); return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if self.path == "/api/enrollment/upload":
                if length > MAX_UPLOAD_BYTES:
                    raise ValueError("Each photo must be smaller than 10 MB.")
                result = self.node.upload(self.rfile.read(length))
            else:
                if length > 65536:
                    raise ValueError("Request is too large.")
                raw = self.rfile.read(length)
                request = json.loads(raw.decode("utf-8") or "{}")
                actions = {
                    "/api/enrollment/start": lambda: self.node.start(request),
                    "/api/enrollment/cancel": self.node.cancel,
                    "/api/enrollment/finish": self.node.finish,
                    "/api/enrollment/delete": self.node.delete_target,
                    "/api/camera/jog": lambda: self.node.jog_camera(request),
                }
                if self.path not in actions:
                    self.json_response({"error": "Not found."}, 404); return
                result = actions[self.path]()
            self.json_response(result)
        except (ValueError, RuntimeError) as exc:
            self.json_response({"error": str(exc)}, 400)
    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while True:
                with self.node.lock:
                    jpeg = self.node.latest_jpeg
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write("Content-Length: {0}\r\n\r\n".format(len(jpeg)).encode("ascii"))
                    self.wfile.write(jpeg + b"\r\n")
                time.sleep(0.12)
        except (BrokenPipeError, ConnectionResetError):
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--observations-topic", default="/perception/face_observations")
    parser.add_argument("--camera-head-status-topic", default="/camera_head/status")
    parser.add_argument("--camera-head-command-topic", default="/camera_head/command")
    parser.add_argument("--model-name", default="antelopev2_glintr100")
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--input-mean", type=float, default=127.5)
    parser.add_argument("--input-std", type=float, default=127.5)
    parser.add_argument("--yunet-engine-path", required=True)
    parser.add_argument("--store-path", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = EnrollmentNode(args)
    Handler.node = node
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("enrollment console ready on http://{0}:{1}/".format(args.bind, args.port), flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        server.shutdown(); server.server_close(); node.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
