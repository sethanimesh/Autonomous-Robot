# Echora Robo

Echora Robo is an indoor tracked robot built from a LEGO EV3 chassis, a Jetson
Orin Nano, and a USB camera. The Jetson is the autonomy computer; the EV3 is the
low-level motor and sensor controller.

The project is being developed in small, testable stages. Phase 1 and 2 (safe
Jetson-to-EV3 motor control and its ROS 2 integration) are complete. Phase 3 is
now complete through calibrated camera acquisition, stationary person and face
detection, plus the deployed target-person enrollment and recognition stack.
The real target is enrolled and passes stationary known-person recognition;
a different real person also passed the stationary rejection test with zero
false matches. Varied-distance/lighting validation was explicitly deferred.
Phase 4 encoder-odometry floor calibration is complete. The active next phase
is camera-only autonomous movement inside one prepared room. Mapping, Nav2, and
multi-room search are deliberately deferred until after the PWA.

Startup calibration runs before the browser find-person mission and remains
under hardware validation. Physical camera limits must be explicitly referenced;
segmentation never defines encoder zero. Prepared calibration restores poses
within the same reference and checks floor, room, and upper views on return.
The target margin and named-move tolerance are now three counts, based on the
head tests; the five-second settling pause remains in place.

Repeated head testing completed 18 bounded moves without motor retries, followed
by a semantic survey. SegFormer and tested local vision-language models missed
the wooden overhead panel. An optional operator-labelled image match can verify
that particular view without changing floor masks or route-clearance decisions.
A network-reset crash that erased the reference has a tested EV3 repair staged
for installation. A new power cycle still requires a confirmed physical lower
pose. Complete autonomous Phase 6 operation has not been demonstrated.

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
tests passed. Effective floor geometry is now measured and deployed.

Phase 4 has a bounded ROS calibration runner that records encoder and pose
start/end values, requires confirmed stop feedback, saves both successful and
failed attempts, and calculates effective wheel radius and track width from
real measurements. It failed closed while the EV3 was offline, then completed
real straight and turn calibration after connectivity returned. The deployed
effective geometry is a 0.0144504 m drive radius and 0.182557 m track width. A
final measured turn matched 90 degrees left, straight travel was reported as
straight, and every run ended with confirmed stopped feedback.

## Active roadmap

1. Secure or replace the movement-sensitive USB camera lead and fit permanent
   strain relief while retaining the tested cable-neutral marker.
2. Validate one target approach from beyond the accepted safe standoff; the
   bounded seated-person search and stop are now proven live.
3. Voice and higher-level intelligence for the single-room mission.
4. PWA for enrollment, live status, and mission control.
5. Deferred: mapping/localization, whole-home Nav2, and multi-room search.

The operator-selected camera range is normalized to **0 for floor, -27 for the
person/forward search, -42 for face/identity, and -54 for the hard maximum
height**. The named poses were verified with live frames and detector output.
Position moves have tight software bounds and mutual exclusion with the
chassis; every stop path brakes both tracks and actively holds the camera motor.

The same page provides **Tilt up** and **Tilt down** controls in bounded 5° and
15° steps. Each click requests one encoder position move and shows the current
encoder position. During a move, the same-direction buttons stay available to
retry the unchanged absolute target; the opposite direction is disabled so
clicks cannot stack extra travel. The EV3 keeps both tracks stopped.

Live before/after frames on 2026-09-05 established the current linkage direction:
**negative encoder counts lift the camera and positive counts lower it**. The
browser sends semantic directions through one sign mapping. Autonomous travel
uses targets no larger than 15 counts. If the encoder advances fewer than two
counts for one second, the brick retries the same target once; difficult
negative lifting uses 1500 counts/s while gravity-assisted lowering keeps its
normal speed. A second one-second stall stops and holds.

A live 15-count round trip from maximum height succeeded: -54 to -40 and back
to -55 against a -54 target, with no retry needed. A later floor → person → face
cycle reached +1, -27, and -42 with both tracks stopped. At -27 the person
detector found the standing subject in 17/31 frames; at -42 it found the subject
in 29/29 frames and confirmed the enrolled target.

The console also exposes `/snapshot.jpg`, a no-store copy of the latest frame.
Calibration and autonomous tilt checks compare a frame before movement, a frame
after the loaded linkage has settled, and a second held frame. Encoder motion
without a corresponding stable visual change is rejected. If USB drops, the
robot remains stopped while a bounded reconnect window waits for a fresh frame.
This local check is deterministic; an LLM is not in the motor safety loop.

The single-room controller will move only in short segments and inspect the
forward and downward views between them. Uncertain or stale vision means stop.
The camera-head motion and chassis motion are coordinated so the recognition
pipeline never assumes the camera is forward while it is checking the floor.

The first local-navigation slice now uses the measured 20 cm × 25 cm chassis
and requires perception to prove a 30 cm wide corridor before selecting one
5–10 cm motion primitive. Route confidence and known-image coverage fail
closed. Because a long external cable remains attached, full scans are limited
to ±90° inside a hard ±120° tether envelope with a 5° margin. Chassis motion
is locked until the operator confirms the marked cable-neutral pose, every turn
is checked before motion, measured afterward, and returned toward its scan
origin. See `docs/CABLE_AND_CAMERA_SUPPORT.md` for the physical harness.

A supervised right-first Phase 6 scan found the seated enrolled target after
two measured ~12° right turns. It stopped at an absolute cable heading of
**+24.46°** and a 0.2083 person-box height fraction, already beyond the 0.15
safe-standoff threshold, so no forward drive was issued. The suspect USB lead
disconnected during the first turn and recovered automatically; permanent
strain relief or lead replacement remains required before unattended use.

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

The live camera and enrollment console is kept active at
`http://192.168.1.48:8080/` by the boot-enabled
`echora-enrollment-console.service`. Its **Find person** button starts the
bounded single-room Phase 6 mission only after the operator confirms the robot
is at the marked cable-neutral heading and the tether is clear. The adjacent
**STOP ROBOT** button independently stops the mission process, chassis, and
camera head. A supervised manual-drive panel provides press-and-hold forward,
back, left, and right controls. Releasing stops, stale browser commands stop in
0.25 seconds, and odometry blocks further turning near the tether envelope.
The same page accepts guided live views and uploaded photos,
rejects mixed identities and poor samples, requires front/left/right diversity,
and stores only private numerical embeddings by default. The opt-in photo
setting retains only aligned 112×112 face crops, never full camera frames.

Note that the installed **PyTorch is a CPU-only build** and provides no
acceleration; TensorRT is the working GPU path. Model weights and engines are
**not** committed. Mapping and recording have not been added. Phase 6 movement
is restricted to the deterministic, tether-bounded find-and-approach mission;
the perception nodes themselves never command motors.

See:

- [Development log](docs/DEVELOPMENT_LOG.md)
- [Hardware notes](docs/HARDWARE_NOTES.md)
- [Existing EV3 server notes](docs/EXISTING_EV3_SERVER.md)
- [Proposed fail-safe EV3 service](robot/ev3/server/README.md)
- [Jetson EV3 client](robot/jetson/ev3_bridge/README.md)
- [Jetson USB camera source](robot/jetson/camera/README.md)
- [Jetson person detector](robot/jetson/perception/README.md)
