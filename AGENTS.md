# AGENTS.md

THIS IS FOR PERSONAL NON-COMMERCIAL USE, WE CAN USE ANYTHING

## Project Goal

Build an identity-aware LEGO EV3 / Jetson robot for **On-Call Hospital Assistance
with Recipient Directed Care Coordination**. The robot locates the pre-enrolled
caregiver selected in an assistance request, inspects the route, approaches in
short checked segments, and presents an approved message.

The current scope is a prepared single room. A
home simulation covering recipient search, approach, playback, and human
acknowledgement. Describe this as a simulated caregiver scenario, keeping it
separate from hospital deployment or measured clinical benefit. Retained
component and partial-mission results must keep their original conditions.

The Jetson retains mission and movement authority. The EV3 remains the low-level
motor and encoder controller. The companion Mac handles heavier depth,
segmentation, speech adapters, and cloud requests; remote services return
interpretations rather than motor commands. Mapping, Nav2, and multi-room search
are deferred extensions.

This document is intentionally lightweight. Treat it as guidance rather than a rigid specification.

---

## High-Level Architecture

```text
                   ┌──────────────────────────────┐
                   │      Jetson Orin Nano        │
                   │                              │
USB Camera ───────▶│  Perception                  │
                   │  - Person detection          │
                   │  - Face identification       │
                   │                              │
                   │  Visual evidence / state     │
                   │  - Camera-view coordination  │
                   │  - Encoder odometry          │
                   │                              │
                   │  Navigation                  │
                   │  - Route assessment          │
                   │  - Short checked movements   │
                   │                              │
                   │  Mission Logic               │
                   │  - Find selected caregiver   │
                   │  - Supervise request delivery│
                   │                              │
                   │  ROS 2                       │
                   └──────────────┬───────────────┘
                                  │
                          Wi-Fi / TCP
                                  │
                   ┌──────────────▼───────────────┐
                   │         EV3 + ev3dev         │
                   │                              │
                   │  Motor control               │
                   │  Wheel encoder feedback      │
                   │  Camera tilt: large motor A  │
                   │  IR sensor: deferred         │
                   │  Emergency local stop        │
                   └──────────────┬───────────────┘
                                  │
                       Tracks / LEGO hardware
```

The companion Mac provides asynchronous visual and speech services to the
Jetson. Result acceptance must check source-image, profile, camera-reference,
and motion-state bindings before an interpretation influences a mission action.

---

## Suggested Technology Stack

### Jetson

Use ROS 2 as the main integration framework. The current stack includes:

- ROS 2
- OpenCV for camera handling
- TensorRT engines with FP16 enabled for Jetson perception
- YOLOX-s for person detection, YuNet for faces/landmarks, and InsightFace
  AntelopeV2 embeddings for enrolled-recipient recognition
- SQLite for identity profiles and appearance memory
- Mac-backed Depth Anything V2, SegFormer-B0, and Gemini visible-hazard analysis
- ros2_control concepts where useful, without forcing ROS onto the EV3 itself

Nav2, ORB-SLAM3/RTAB-Map, depth cameras, and LiDAR remain possible later additions.

ROS 2 should mostly be used as the message bus connecting perception, localization, navigation, robot state, and mission logic.

Avoid turning every small function into its own ROS node unless it provides a useful boundary.

### EV3

Keep the EV3 simple.

Use:

- ev3dev
- Python
- ev3dev2 motor/sensor APIs
- a lightweight TCP/WebSocket/UDP service for Jetson communication

The EV3 should not perform heavy AI, SLAM, or path planning.

Its responsibilities are roughly:

```text
receive velocity/movement command
        ↓
control left/right motors
        ↓
read encoders + IR sensor
        ↓
return robot state
```

The EV3 may independently stop the motors when a close obstacle is detected or communication is lost.

---

## Communication Model

Prefer a small Jetson ↔ EV3 bridge.

Example conceptual messages:

```json
{
  "linear": 0.20,
  "angular": -0.35
}
```

EV3 response:

```json
{
  "left_encoder": 1234,
  "right_encoder": 1218,
  "ir_distance": 42,
  "connected": true
}
```

The exact protocol can evolve.

A useful abstraction on the Jetson side is:

```text
ROS 2 /cmd_vel
      ↓
EV3 Bridge
      ↓
Wi-Fi
      ↓
EV3 motor controller
```

and:

```text
EV3 encoders + sensors
      ↓
EV3 Bridge
      ↓
ROS 2 odometry / sensor topics
```

This lets the rest of the autonomy stack behave like it is controlling a normal ROS robot.

---

## Main Software Layers

### 1. Hardware Layer

Handles:

- left track motor
- right track motor
- large camera-head motor A
- IR sensor if restored later
- encoder readings

Do not mix AI or navigation logic into this layer.

---

### 2. EV3 Bridge

Responsible for:

- sending motion commands
- receiving encoder data
- receiving IR readings
- connection health
- translating ROS commands into EV3 commands

Keep the public interface simple enough that the underlying transport can be replaced later.

---

### 3. Robot State / Odometry

Estimate robot motion from left and right track encoders.

Maintain:

```text
x
y
heading
linear velocity
angular velocity
```

Accuracy does not need to be perfect initially.

The visual localization system can later correct encoder drift.

---

### 4. Camera / Perception

The USB camera is attached to the Jetson.

Perception pipeline:

```text
camera frame
    ↓
person detector
    ↓
person crop / face detector
    ↓
face embedding
    ↓
compare against target-person embeddings
```

Do not run face recognition on every frame if it is unnecessary.

A practical sequence is:

```text
detect person
    ↓
is face visible?
    ↓
generate embedding
    ↓
compare
```

Store several reference embeddings rather than relying on one photograph.

Target-person recognition should be probabilistic, with multiple observations preferred before declaring success.

---

## Mapping and Localization

Start modularly so the mapping system can be changed later.

### Deferred mapping experiments

Experiment with monocular visual SLAM using the USB camera.

Possible tools include:

- ORB-SLAM3
- RTAB-Map
- other ROS-compatible visual SLAM packages

Monocular SLAM may be sufficient for experimentation but can struggle with:

- blank walls
- poor lighting
- motion blur
- repetitive indoor textures
- scale estimation

Do not architect the system around the assumption that the webcam will always be sufficient.

A later depth camera or 2D LiDAR should be easy to add without rewriting the mission system.

---

## Navigation

Nav2 is a candidate for a later mapped navigation layer. The current mission
uses stopped visual inspection and short encoder-monitored movement segments.

Conceptually:

```text
current pose + map
       ↓
global planner
       ↓
local planner / controller
       ↓
/cmd_vel
       ↓
EV3 bridge
```

The robot should ideally be able to:

- rotate in place
- navigate to a coordinate
- stop near obstacles
- cancel a goal
- recover when blocked
- return to previously visited locations

The tracked chassis is useful because it can perform near-zero-radius turns.

---

## Obstacle Handling

Use multiple levels rather than relying on a single system.

```text
Navigation planner
      ↓
camera / map-based obstacle handling
      ↓
EV3 IR proximity check
      ↓
motors
```

The IR sensor is currently deferred and does not provide an active obstacle
stop. If restored, treat it as a local safety/proximity signal rather than the
primary mapping sensor.

If the EV3 detects something dangerously close, stopping locally is preferable to waiting for the Jetson.

---

## Mission Layer

The top-level behavior should remain easy to understand.

Example state flow:

```text
IDLE
 ↓
SELECT_RECIPIENT
 ↓
PREPARE_CAMERA
 ↓
SEARCH
 ↓
SCAN
 ↓
PERSON_DETECTED
 ↓
IDENTIFY
 ├── not target → continue search
 └── target
       ↓
    CHECK_ROUTE
       ↓
    APPROACH_SEGMENT
       ↓
    REACQUIRE_TARGET
       ↓
     FOUND
```

This can initially be implemented as a normal state machine.

Do not introduce an LLM or autonomous-agent framework for basic movement decisions unless there is a clear reason.

Later, an LLM can sit above the deterministic robotics stack for commands such as:

> Find the selected nurse and deliver the approved assistance request.

The LLM should translate intent into robot missions, not directly control motors.

---

## Deferred Multi-Room Search Strategy

Avoid random wandering once mapping works.

A basic systematic search can be:

```text
known rooms / regions
       ↓
choose unsearched region
       ↓
navigate there
       ↓
perform 360° visual scan
       ↓
mark region searched
       ↓
continue
```

Eventually the robot can maintain information such as:

```text
living_room: searched
kitchen: searched
bedroom_1: target not seen
bedroom_2: pending
```

Room labels can initially be manually configured.

Automatic semantic room understanding can come later.

---

## Camera Head

The existing motor-A mechanism moves the camera vertically. The attached motor
was physically identified as an EV3 large motor, not a medium motor.

The current mechanism supplies referenced person, face, and floor views. The
tracked chassis rotates to scan horizontally within the tether limits. Stop
before changing camera views, reject observations from an incompatible view or
motion state, and reacquire the recipient after each approach segment.

Later, the mechanism can be redesigned into a pan or pan/tilt head.

Keep camera control behind an abstraction such as:

```python
camera_head.look_forward()
camera_head.look_up()
camera_head.look_down()
```

This avoids coupling perception code to the LEGO mechanism.

---

## Suggested Repository Structure

```text
robot/
├── ev3/
│   ├── server/
│   ├── motors/
│   ├── sensors/
│   └── safety/
│
├── jetson/
│   ├── ev3_bridge/
│   ├── perception/
│   ├── localization/
│   ├── navigation/
│   ├── mission/
│   └── camera/
│
├── ros2_ws/
│   └── src/
│
├── config/
│   ├── robot.yaml
│   ├── navigation.yaml
│   └── perception.yaml
│
├── scripts/
├── tests/
└── AGENTS.md
```

This is a suggestion rather than a required layout.

Prefer boundaries based on responsibilities over creating many tiny modules.

---

## Development Order

Build capabilities incrementally.

### Phase 1 — Robot control

Goal:

```text
Jetson → EV3 → motors
```

Demonstrate:

- forward
- backward
- rotate left
- rotate right
- stop
- encoder feedback
- IR feedback

---

### Phase 2 — ROS integration

Expose the EV3 robot through ROS 2.

Typical interfaces:

```text
/cmd_vel
/odom
/ir_distance
/robot_status
```

At this stage the Jetson should be able to treat the EV3 chassis like a normal mobile robot.

---

### Phase 3 — Camera perception

Get stable USB-camera input.

Then add:

```text
camera
 ↓
person detection
 ↓
face detection
 ↓
target-person recognition
```

Test this while the robot is stationary first.

---

### Phase 4 — Odometry

Use EV3 motor encoders to estimate:

- distance travelled
- rotation
- pose

Validate simple movements such as:

```text
drive 1 meter
rotate 90°
drive back
```

Exact precision is not required initially.

---

### Phase 5 — Camera-only single-room movement

Use the movable camera and calibrated odometry without a persistent map.

Demonstrate:

- safe limits and named forward/down positions for camera-head motor A
- a controlled 360-degree chassis scan
- forward and downward visual checks before each movement
- short low-speed movement segments with a stop and recheck between them
- visually detect blocked floor, choose a clearer left/right route, and retry
- remain within one prepared room

The camera is the active safety and guidance sensor in this phase. Coordinate
camera angles in a deterministic state machine so person recognition pauses
while the camera checks the floor. Treat uncertain, stale, dark, or obstructed
vision as blocked and stop. Mapping and Nav2 are deliberately deferred.

---

### Phase 6 — Find the selected recipient in one room

Create the single-room mission:

```text
scan chassis and camera
 ↓
detect people
 ↓
identify the target
 ↓
check the floor and route
 ↓
approach in short segments
 ↓
stop at a safe distance
```

If the target disappears or the route becomes uncertain, stop, rescan, and
choose another visible route instead of continuing blindly.

---

### Phase 7 — Voice and intelligence for one room

Add:

- voice commands and speech recognition
- intent translation such as `FindPerson(target=selected_caregiver)`
- spoken mission status and result
- deterministic mission execution beneath the intelligence layer

Reviewed typed/transcribed messages and supervised find-and-deliver playback are
already implemented. Keep voice-command starts and software acknowledgement
separate from this existing feature. A `played` result records playback
completion; the home demonstration's human acknowledgement is an operator
observation.

The LLM or intent layer must not command motors directly.

---

### Phase 8 — PWA

Build the phone interface after the single-room voice mission works. Include
face registration, live camera/status, mission start/stop, and diagnostics.

---

### Phase 9 — Mapping and localization (deferred)

Run visual SLAM while manually driving the robot:

```text
camera + odometry
        ↓
map + robot pose
```

The RTAB-Map runtime may be installed earlier for experiments, but it should not
be active or required by the single-room mission.

---

### Phase 10 — Mapped indoor navigation (deferred)

Introduce Nav2 only after localization is reasonably stable. Test selected map
destinations, obstacle recovery, doorways, and return paths.

---

### Phase 11 — Multi-room autonomous search (deferred)

Extend the proven single-room find-person state machine with mapped rooms,
search coverage, room status, and navigation between rooms.

---

## Development Principles

Prefer working vertical slices over building the entire architecture before testing.

A useful progression is:

```text
remote-control robot
        ↓
robot that sees
        ↓
robot that knows where it is
        ↓
robot that navigates
        ↓
robot that recognizes people
        ↓
robot that searches autonomously
```

Keep hardware-specific code isolated.

Keep perception independent from navigation where practical.

Log enough information for replay and diagnosis:

- camera timestamps
- robot pose
- motor commands
- sensor readings
- detections
- mission state

Keep development notes crisp: record the result, essential measurement, issue
cause, and next action only. Prioritize implementation and real-world testing
over lengthy documentation.

Use ROS bags when useful.

Do not optimize prematurely for perfect autonomy.

---

## Testing Philosophy

Prefer small real-world tests.

The EV3 battery drains quickly. Complete coding, simulation, and Jetson-only
checks before powering it. Ask the user to turn the EV3 on only for a short,
prepared hardware test, execute that test immediately, then explicitly tell the
user to turn the EV3 off when it is no longer needed.

Examples:

- command the EV3 from the Jetson
- stop in front of a wall
- rotate approximately 90°
- detect a person from several distances
- recognize the target under different lighting
- create a map of one room
- navigate across one room
- navigate through one doorway
- search two rooms
- perform a complete target-person search

Simulation can be useful but is not required before physical testing.

---

## Safety

The robot should default toward stopping when uncertain.

Useful safeguards include:

- stop on lost Jetson connection
- stop on stale movement commands
- stop on very close IR reading
- limit maximum speed during development
- expose an easy manual stop mechanism

These should remain simple and should not complicate experimentation.

---

## Long-Term Architecture

The eventual system can look like:

```text
Voice / App / User Command
          ↓
 Mission / Intent Layer
          ↓
 Search & Task Planner
          ↓
      ROS 2 / Nav2
       ↙       ↘
 Perception    SLAM
      ↓          ↓
      Robot State
          ↓
       EV3 Bridge
          ↓
         EV3
          ↓
   Motors + Sensors
```

The most important architectural boundary is:

```text
HIGH-LEVEL AUTONOMY → Jetson
LOW-LEVEL ACTUATION → EV3
```

Everything else can evolve as the project develops.
