# Echora Robo

Echora Robo is an indoor tracked robot built from a LEGO EV3 chassis, a Jetson
Orin Nano, and a USB camera. The Jetson is the autonomy computer; the EV3 is the
low-level motor and sensor controller.

The project is being developed in small, testable stages. Phase 1 and 2 (safe
Jetson-to-EV3 motor control and its ROS 2 integration) are complete. Phase 3 is
now complete for camera acquisition and stationary person detection.

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
The camera is **not yet physically calibrated**: ChArUco collection, strict
quality checks, a printable target, and deployment are ready, but `CameraInfo`
remains explicitly zeroed until a real board capture passes. It must not yet be
used for metric vision.

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

Note that the installed **PyTorch is a CPU-only build** and provides no
acceleration; TensorRT is the working GPU path. Model weights and engines are
**not** committed. No face detection, recognition, identity, tracking,
following, mapping, or recording has been added, and the motors stayed stopped
throughout.

See:

- [Development log](docs/DEVELOPMENT_LOG.md)
- [Hardware notes](docs/HARDWARE_NOTES.md)
- [Existing EV3 server notes](docs/EXISTING_EV3_SERVER.md)
- [Proposed fail-safe EV3 service](robot/ev3/server/README.md)
- [Jetson EV3 client](robot/jetson/ev3_bridge/README.md)
- [Jetson USB camera source](robot/jetson/camera/README.md)
- [Jetson person detector](robot/jetson/perception/README.md)
