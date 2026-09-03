# Echora Robo

Echora Robo is an indoor tracked robot built from a LEGO EV3 chassis, a Jetson
Orin Nano, and a USB camera. The Jetson is the autonomy computer; the EV3 is the
low-level motor and sensor controller.

The project is being developed in small, testable stages. Phase 1 and 2 (safe
Jetson-to-EV3 motor control and its ROS 2 integration) are complete. Phase 3 is
now complete through calibrated camera acquisition, stationary person and face
detection, plus the deployed target-person enrollment and recognition stack.
The real target is enrolled and passes stationary known-person recognition.
The next physical step is validating rejection of a different person, followed
by known-target checks under varied distance and indoor lighting.

## Working rules

- Default to stopping when communication or sensor data is uncertain.
- Test one physical capability at a time.
- Do not move motors until their ports, orientation, and safe test setup are
  confirmed.
- Record every meaningful test in `docs/DEVELOPMENT_LOG.md`.
- Report measured numbers, including disappointing ones, rather than requested
  or expected ones.
- Keep wiring and machine facts current in `docs/HARDWARE_NOTES.md`.
- Never store machine passwords or other secrets in this repository.

## Current status

The Mac, Jetson, and EV3 have been inventoried over SSH. The USB camera has been
restored and OpenCV can capture frames. The owner confirmed A as the tool/camera
head motor, B as the left track, C as the right track, and a safe raised test
setup.

A minimal Python 3.5-compatible EV3 control service is implemented, passes local
automated safety tests, and has passed its first raised-chassis hardware test.
All three motors responded on their expected ports, while watchdog and client
disconnect stops worked correctly. The server is now enabled as the managed
`echora-ev3.service` on the EV3. A tested Jetson client is deployed and has
passed paired forward-sign, reverse-sign, and both turn-sign tests. The
non-working IR sensor is explicitly deferred.

The first ROS 2 vertical slice is also live: enabled `echora-bridge.service` on
the Jetson subscribes to `/cmd_vel` and publishes EV3 feedback on
`/robot_status`, encoder odometry on `/odom`, track state on `/joint_states`, and
the `odom → base_link` transform. Raised-chassis linear, angular, and odometry
tests passed. Metric geometry remains provisional until the chassis is
physically measured.

The stationary camera pipeline is live. Enabled
`echora-camera.service` publishes `/camera/image_raw`, `/camera/camera_info`,
and `/camera/status` from the USB camera at a measured **27.3 fps** (MJPG
640×480, requested 30). The node validates every frame, recovers automatically
from a camera disconnect, and was verified over a continuous five-minute run.
The camera is now physically calibrated at 640×480 using a ChArUco target.
`/camera/camera_info` publishes the accepted `plumb_bob` model with exact
image-matching timestamps. The final model measured 0.596 px RMS error,
preserves 89.2% valid image area at full field of view, and passed a live
raw-versus-rectified visual check.

Stationary person detection is live on top of it. Enabled
`echora-person-detector.service` runs **YOLOX-s (Apache-2.0)** as a **TensorRT
FP16** engine on the Jetson GPU and publishes
`/perception/person_detections` (`vision_msgs/msg/Detection2DArray`),
`/perception/person_image`, and `/perception/status`. Measured GPU inference is
**17.1 ms** (23.7 ms end to end, a 42 fps ceiling) at a configured 15 Hz cap,
and the reported provider `tensorrt_fp16` is verified from the engine's own
tensor datatypes rather than assumed. Under live test it detected a person in
**100% of processed frames** at near, medium, far, partially visible, and
two-person poses — against an 80% target — with **zero false positives** across
1304 frames of an empty room. It survives camera loss, reports stale input, and
resumes automatically.

Stationary face detection is now live as a second bounded stage. Enabled
`echora-face-detector.service` runs **YuNet 2023mar (MIT)** through **TensorRT
FP16**, but only inside exact-frame YOLOX person regions. It publishes
`/perception/face_detections`, transient five-landmark observations,
`/perception/face_image`, and
`/perception/face_status`. A live ten-second acceptance run found one face in
**50/50 processed views**, with every face timestamp matching both the observed
camera and person messages, zero malformed results, and zero inference errors.
GPU inference plus decode averages roughly **16-18 ms** per person region.
Five landmarks are drawn for validation but are not persisted.

Target-person recognition is deployed as enabled
`echora-target-recognizer.service`. It runs InsightFace AntelopeV2's
**ResNet-100 Glint360K** recognizer as a measured TensorRT FP16 engine, aligns
each face from YuNet's five landmarks, compares it against several enrolled
views, and requires **three matches in five recent observations** before
reporting `target_confirmed`. It publishes `/perception/target_matches` and
`/perception/recognition_status`. The model is restricted to
**non-commercial research/personal use**, which the owner explicitly confirmed
for this project.

The temporary enrollment console is available at
`http://192.168.1.48:8080/` while `echora-enrollment-console.service` is
running. It accepts both guided live views and uploaded photos, rejects mixed
identities and poor samples, requires front/left/right diversity, and stores
only private numerical embeddings by default. The opt-in photo setting retains
only aligned 112×112 face crops, never full camera frames.

Note that the installed **PyTorch is a CPU-only build** and provides no
acceleration; TensorRT is the working GPU path. Model weights and engines are
**not** committed. No tracking, following, mapping, or recording has been
added, and the motors stayed stopped throughout perception development.

See:

- [Development log](docs/DEVELOPMENT_LOG.md)
- [Hardware notes](docs/HARDWARE_NOTES.md)
- [Existing EV3 server notes](docs/EXISTING_EV3_SERVER.md)
- [Proposed fail-safe EV3 service](robot/ev3/server/README.md)
- [Jetson EV3 client](robot/jetson/ev3_bridge/README.md)
- [Jetson USB camera source](robot/jetson/camera/README.md)
- [Jetson person detector](robot/jetson/perception/README.md)
