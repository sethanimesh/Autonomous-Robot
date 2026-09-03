# Jetson Perception

`person_detector.py`, `face_detector.py`, and `target_recognizer.py` form the
stationary Phase 3 perception pipeline. YOLOX finds people, YuNet finds faces
inside exact-frame person regions, and AntelopeV2 compares aligned faces with
the enrolled target. All three execute on the Jetson GPU through TensorRT FP16.

These nodes only look. There is no tracking, following, liveness/authentication
claim, or motor command. They do not connect to the EV3. Full camera frames are
never written to disk by this pipeline.

## Topics

| Topic | Type | QoS |
| --- | --- | --- |
| `/camera/image_raw` (in) | `sensor_msgs/msg/Image` (`bgr8`/`rgb8`) | best effort, depth 1 |
| `/perception/person_detections` | `vision_msgs/msg/Detection2DArray` | reliable, depth 10 |
| `/perception/person_image` | `sensor_msgs/msg/Image` (`bgr8`) | best effort, depth 1 |
| `/perception/status` | `std_msgs/msg/String` (JSON) | reliable, transient local, depth 1 |
| `/perception/face_observations` | `std_msgs/msg/String` (JSON) | transient boxes plus five landmarks |
| `/perception/target_matches` | `vision_msgs/msg/Detection2DArray` | target boxes and similarity |
| `/perception/recognition_status` | `std_msgs/msg/String` (JSON) | target and confirmation state |

Every detection carries the **source image's** timestamp and frame ID, on both
the array header and each `Detection2D` header, so a detection can always be
related back to the exact frame it came from. The annotated image carries the
same stamp and frame ID and has the same dimensions as the source.

An empty `Detection2DArray` is still published when nobody is in view. That is
how a subscriber tells "nobody here" apart from "the detector has stopped".

### The pose field is deliberately empty

`ObjectHypothesisWithPose.pose` is left at its zero default. The camera now has
accepted intrinsics, but a monocular 2D box still has no measured depth, so no
metric position exists to report. Nothing here may be used as a 3D person or
face position. Bounding boxes are source-image pixels only.

## Face detector

| Topic | Type | Purpose |
| --- | --- | --- |
| `/camera/image_raw` (in) | `sensor_msgs/msg/Image` | Timestamped source frame |
| `/perception/person_detections` (in) | `vision_msgs/msg/Detection2DArray` | YOLOX gate |
| `/perception/face_detections` | `vision_msgs/msg/Detection2DArray` | Face boxes and confidence |
| `/perception/face_observations` | `std_msgs/msg/String` (JSON) | Exact stamp, boxes, and five transient landmarks |
| `/perception/face_image` | `sensor_msgs/msg/Image` | Optional boxes plus five landmarks |
| `/perception/face_status` | `std_msgs/msg/String` (JSON) | Health, latency, matching, provider |

The face node maintains two small bounded timestamp caches because ROS may
deliver the camera callback or the downstream person callback first. A pair is
accepted only when seconds, nanoseconds, and frame ID agree exactly. The
largest three upper-person regions are processed per output cycle, bounding
worst-case work in a crowded room. Overlap across person boxes is removed with
global NMS.

### YuNet model

| | |
| --- | --- |
| Name | YuNet `face_detection_yunet_2023mar.onnx` |
| Source | OpenCV Zoo, `models/face_detection_yunet` |
| License | MIT |
| Input | `1x3x640x640`, raw BGR 0-255 |
| Outputs | 12 tensors: class, objectness, box, and five keypoints at strides 8/16/32 |
| ONNX | 232,589 bytes; SHA-256 `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4` |
| Orin engine | 559,156 bytes; SHA-256 `b1a09ee0e20e33aaefdb0b902286b79d72eefdeade5f3d08ec9db8521fc7d196` |

Model files are ignored by Git. Download the official OpenCV Zoo ONNX file,
verify the checksum above, then build the machine-specific engine on the Orin:

```text
python3 build_engine.py models/yunet_2023mar.onnx models/yunet_2023mar_fp16.engine --fp16
```

SCRFD was evaluated first but its official pretrained weights are restricted
to non-commercial research. YuNet was selected because its model is explicitly
MIT-licensed, it is tiny enough to coexist with YOLOX on the Orin, and its five
landmarks are useful for the later face-alignment stage.

The crop is aspect-preserving and top-left letterboxed before inference.
Directly stretching the wide upper-body crop to 640x640 was tested and failed:
it narrowed the face enough to make detections intermittent. The corrected
pipeline detected one live face in 50/50 processed views at 0.75 threshold.

The annotated image is capped at 3 Hz because converting a full raw ROS image
is CPU-heavy; structured detections remain uncapped up to the configured 10 Hz.
After the final restart, with preview disabled, the live stream measured its
full configured **10.0 Hz** across 99 inferences, with mean per-region inference
plus decode near 18 ms. With the preview active it remains around 5-7 Hz while
still providing a usable operator view.

`echora-face-detector.service` is enabled at boot. The optional
`echora-face-preview.service` serves `http://192.168.1.48:8080/`, is not enabled
at boot, and should be stopped after physical testing because it is an
unauthenticated LAN camera stream.

## Target-person recognition and enrollment

The recognizer uses the official InsightFace **AntelopeV2** package's
`glintr100.onnx`: ResNet-100 trained on Glint360K, 512 output dimensions. This
is the highest-capacity recognition model in the official public InsightFace
packs. Its pretrained weights are explicitly restricted to **non-commercial
research use**; this repository's owner confirmed the robot will remain a
personal, non-commercial project. Do not reuse these weights commercially.

| Artifact | Verified value |
| --- | --- |
| Official package | `antelopev2.zip`, SHA-256 `8e182f14fc6e80b3bfa375b33eb6cff7ee05d8ef7633e738d1c89021dcf0c5c5` |
| Recognition ONNX | 260,665,334 bytes, SHA-256 `4ab1d6435d639628a6f3e5008dd4f929edf4c4124b1a7169e1048f9fef534cdf` |
| Orin FP16 engine | 131,373,148 bytes, SHA-256 `94bc49a39a76ed9cab5547bcba129757b71323f89b267021c74f04208ab5d2c1` |
| Measured precision | 829 FP16 tensors, 1 FP32 tensor |
| Input/output | normalized RGB `1×3×112×112` → 512-value embedding |

YuNet's five landmarks are transformed to the model's ArcFace 112×112
reference geometry. Every embedding is L2-normalized. A probe is compared with
several pose-specific enrollment templates; the score blends the best view
with top-three consensus. The default threshold is 0.45, and three of five
recent observations must pass before `target_confirmed`. This is a search
signal, not a security or liveness guarantee.

`echora-target-recognizer.service` is enabled at boot and safely reports
`not_enrolled` until a target exists. The private store is
`/home/animesh/echora/data/target_person.json`, mode 0600 in a mode 0700
directory, atomically replaced, and bound to the exact model checksum.

For enrollment, start the static `echora-enrollment-console.service` and open
`http://192.168.1.48:8080/`. It is deliberately not enabled at boot because it
is an unauthenticated LAN operator interface. The workflow accepts uploaded
photos plus live views, requires consent, at least ten accepted samples, and
front/left/right coverage. It rejects blur, poor lighting, small faces,
duplicates, multiple people, and a face inconsistent with the session. The
default stores embeddings only. Opt-in retention stores 112×112 aligned face
crops with mode 0600; original uploads and room frames are never retained.

During camera-head calibration, this same page shows the live encoder position
and provides **Tilt up** / **Tilt down** controls. Each click is one five-degree
step. Controls disable when head status is stale, unhomed, outside limits, or
already moving. The EV3 still owns track isolation and rejects unsafe overlap.

Measured public-image checks on the deployed engine: 0.9824 cosine similarity
for one identity after a brightness change, 0.0250 for two different identities,
and 14.37 ms mean embedding latency (23.0 ms p95 over 30 runs). A full temporary
ROS test published three known-target frames, all three matched at up to 0.9713;
three unknown-person frames produced zero matches. The real target is now
enrolled from 18 diverse live views. A 15-second stationary acceptance run
matched 102/102 frames, scoring 0.9364–0.9616 (0.9502 mean), with seven of seven
health reports confirmed and zero inference errors. A stable 15-second test
with a different real person then observed a face in 148/148 frames and
produced zero target matches. Similarity stayed at 0.1014–0.1666 (0.1334 mean),
well below the 0.45 threshold; all seven health reports stayed `searching` and
inference errors remained zero. Varied distance and lighting still require
physical acceptance.

## Model

| | |
| --- | --- |
| Name | **YOLOX-s** |
| Version | release `0.1.1rc0` |
| Source | `https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.onnx` |
| License | **Apache-2.0** (Megvii YOLOX) |
| Training data | COCO (80 classes); only class **0, `person`** is used |
| ONNX size | 35,858,002 bytes |
| ONNX SHA-256 | `c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063` |
| Engine | `/home/animesh/echora/models/yolox_s_fp16.engine`, 20,891,644 bytes |
| Engine SHA-256 | `4298ac7979ccae8411f20e1eec5a7a36ab887a2314afbe71a72cd9c5dd033517` |
| Input | `1x3x640x640` NCHW float32, raw 0-255 **BGR**, no mean/std normalisation |
| Output | `1x8400x85`, raw and undecoded |

YOLOX-tiny at 416 was also built and measured as a lighter alternative:
ONNX SHA-256 `427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7`,
engine SHA-256 `67d4213a478ca865e4a75cedd788e267a6595f9da2ff1f4da89f70d390a6fe28`.
It is faster (15.8 ms against 23.7 ms) but has materially lower accuracy
(COCO mAP 32.8 against 40.5). YOLOX-s was chosen because the headroom exists.

**Ultralytics YOLOv8/v11 was deliberately not used.** It is AGPL-3.0, which
would attach copyleft obligations to anything the project later distributes or
exposes as a network service. YOLOX is Apache-2.0 and carries no such term.

Model files are **not committed**: `.onnx`, `.engine` and `.engine.json` are
gitignored. Rebuild the engine on the Jetson with:

```text
python3 build_engine.py models/yolox_s.onnx models/yolox_s_fp16.engine --fp16
python3 build_engine.py --inspect models/yolox_s_fp16.engine   # refresh metadata
```

An engine is specific to the GPU and TensorRT version that built it and must
never be copied between machines.

## Acceleration is measured, not claimed

The provider string is derived from evidence, never from the build flag:

- `build_engine.py` builds with `ProfilingVerbosity.DETAILED` and reads back
  each tensor's `Format/Datatype` from the engine inspector, writing the
  histogram to a `<engine>.json` sidecar.
- For the deployed engine that histogram is **274 FP16 tensors against 21
  FP32**, so `measured_precision` is `fp16`.
- The node reports `provider=tensorrt_fp16`, `device=Orin`. If the precision
  cannot be confirmed it reports plain `tensorrt` rather than guessing.
- GPU work was independently confirmed with `tegrastats`: `GR3D_FREQ` moved
  between 20% and 56% while detecting, against 0% idle.

**PyTorch on this Jetson is a CPU-only wheel** (`2.6.0+cpu`,
`torch.cuda.is_available()` is `False`), so it offers no acceleration at all.
TensorRT is the real path and is also faster than PyTorch-CUDA would be.

`cuda-python==12.6.2.post1` was installed (`pip --user`) to give TensorRT
host/device memory and stream management; neither `pycuda` nor `cuda-python`
was present. `ros-humble-vision-msgs` 4.1.1 was installed for
`Detection2DArray`.

## Measured performance

Jetson Orin Nano, MAXN_SUPER, 640x480 `bgr8` input, YOLOX-s FP16:

| Stage | Mean | p95 | Max |
| --- | ---: | ---: | ---: |
| Preprocess (letterbox, CHW) | 2.98 ms | 3.01 ms | 4.66 ms |
| **GPU inference** | **17.07 ms** | 17.19 ms | 17.46 ms |
| Decode + threshold | 3.60 ms | 3.77 ms | 3.87 ms |
| **Total** | **23.65 ms** | 23.89 ms | 25.13 ms |

That is a **42 fps ceiling**. `max_inference_rate_hz` is set to **15.0**, well
under the camera rate, so the node keeps headroom and always works on a recent
frame instead of racing the camera. Model load is **0.42 s** and warm-up
**0.11 s**, both once at startup.

Detection accuracy, measured against the live camera under normal indoor
lighting (see the development log for the full table): **100% of processed
frames** contained a correct detection at near, medium, far, partial and
two-person poses, against an 80% target, and **zero false positives** across
1304 frames of a genuinely empty room.

## Frame handling

The subscription callback is deliberately cheap: it validates the message and
drops it into a **one-deep slot**, then returns. A timer running at
`max_inference_rate_hz` takes whatever is in that slot and runs inference.

This is what makes an unbounded backlog impossible — the queue depth is one by
construction, not by policy. A frame displaced before it was processed is
counted in `frames_dropped`. At 17 fps input and 15 Hz inference roughly 10%
of frames are dropped by design; earlier, against a 27 fps camera, it was 43%.
Dropping surplus frames is correct behaviour, not a fault: `last_frame_age_sec`
stayed at 0.04 s throughout.

A frame older than `max_frame_age_sec` is discarded rather than detected on, so
a published detection always describes a recent view.

## Failure handling

- Unsupported encodings, zero-sized images, and payloads that disagree with the
  declared geometry are rejected with a reason code and counted, never
  reshaped into garbage.
- Any inference exception is caught, counted in `inference_errors`, and the
  node continues. One bad frame is never fatal.
- When frames stop arriving the node reports `state: stale_input`, logs
  **once**, and keeps waiting. It does not busy-loop and does not exit, so
  systemd never restarts it for a missing camera. Detection resumes
  automatically when frames return.
- Rejections and errors are logged on first occurrence and then at intervals,
  so a persistent fault cannot flood the journal.
- SIGINT invalidates the rclpy context underneath the executor, which surfaces
  as an `RCLError` rather than `KeyboardInterrupt`. `main()` treats an
  exception raised after the context is gone as a normal shutdown and re-raises
  anything else, so a clean stop exits 0 with no traceback.

## Parameters

Configured in `config/perception.yaml`, deployed as
`/home/animesh/echora/perception.yaml`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `image_topic` | `/camera/image_raw` | Source image topic |
| `detections_topic` | `/perception/person_detections` | Detection output |
| `annotated_image_topic` | `/perception/person_image` | Annotated output |
| `status_topic` | `/perception/status` | Health output |
| `model_name` | `yolox_s` | Reported in status |
| `model_path` | `.../yolox_s_fp16.engine` | TensorRT engine |
| `onnx_path` | `.../yolox_s.onnx` | Used only by the CPU fallback |
| `model_input_width` / `_height` | 640 / 640 | Must match the engine; multiple of 32 |
| `confidence_threshold` | 0.45 | Minimum score |
| `nms_iou_threshold` | 0.45 | Overlap above which a box is suppressed |
| `max_inference_rate_hz` | 15.0 | Inference cap; also the timer period |
| `inference_device` | `tensorrt` | `tensorrt`, `cpu`, or `auto` |
| `publish_annotated_image` | `true` | Enable the annotated topic |
| `annotate_only_when_subscribed` | `true` | Skip the copy when nobody listens |
| `enable_timing_diagnostics` | `true` | Include latency fields in status |
| `person_class_id` | 0 | COCO `person` |
| `person_class_label` | `person` | Published `class_id` string |
| `max_frame_age_sec` | 0.5 | Older frames are dropped |
| `frame_timeout_sec` | 2.0 | Missing longer than this is reported stale |
| `max_detections` | 50 | Cap on published boxes |
| `status_interval_sec` | 5.0 | `/perception/status` period |
| `latency_window` | 120 | Rolling window for mean and p95 |

`inference_device` is set to `tensorrt` rather than `auto` for the managed
service on purpose: a silent downgrade to the CPU would make the reported
provider misleading. `auto` falls back and records the reason.

## Running

Managed by enabled `echora-person-detector.service`. To run by hand:

```text
source /opt/ros/humble/setup.bash
python3 person_detector.py --ros-args --params-file perception.yaml
```

## Hardware-free logic

`detector_config.py`, `detections.py`, `detector_health.py`, `image_intake.py`
and `messages.py` import no ROS, OpenCV, numpy, CUDA, or TensorRT. Between them
they cover configuration and threshold validation, the YOLOX anchor grid,
box conversion, letterbox rescaling, clipping, IoU, non-maximum suppression,
person-class filtering, image validation, staleness rules, newest-frame
arbitration, rolling rate and latency, status generation, message assembly,
and annotation placement — all in the repository test suite.

`inference.py` is importable without TensorRT so its model-loading failure
paths are tested too. The ROS node itself holds only pixels, CUDA calls, and
ROS plumbing.
