# AGENTS.md

## Project Goal

Build an autonomous indoor LEGO EV3 robot that uses a Jetson Orin Nano and USB camera to map a house, navigate safely, detect people, identify a target person, and search for them room by room.

The Jetson should act as the main compute and autonomy layer. The EV3 should remain the low-level motion and sensor controller.

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
                   │  Localization / Mapping      │
                   │  - Visual SLAM initially     │
                   │  - Depth/LiDAR later if used │
                   │                              │
                   │  Navigation                  │
                   │  - Global planning           │
                   │  - Local obstacle avoidance  │
                   │                              │
                   │  Mission Logic               │
                   │  - Search rooms              │
                   │  - Find target person        │
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
                   │  IR sensor                   │
                   │  Medium motor                │
                   │  Emergency local stop        │
                   └──────────────┬───────────────┘
                                  │
                       Tracks / LEGO hardware
```

---

## Suggested Technology Stack

### Jetson

Prefer ROS 2 as the main integration framework.

Likely components:

- ROS 2
- Nav2 for navigation
- OpenCV for camera handling
- PyTorch / TensorRT for perception
- YOLO or another lightweight detector for person detection
- InsightFace or similar embeddings for target-person recognition
- ORB-SLAM3, RTAB-Map, or another visual SLAM system for initial mapping experiments
- ros2_control concepts where useful, without forcing ROS onto the EV3 itself

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
- medium motor
- IR sensor
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

### Initial option

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

Nav2 is a good candidate for the navigation layer.

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

The IR sensor should be treated as a local safety/proximity signal rather than the primary mapping sensor.

If the EV3 detects something dangerously close, stopping locally is preferable to waiting for the Jetson.

---

## Mission Layer

The top-level behavior should remain easy to understand.

Example state flow:

```text
IDLE
 ↓
LOCALIZE
 ↓
SEARCH
 ↓
NAVIGATE_TO_SEARCH_AREA
 ↓
SCAN
 ↓
PERSON_DETECTED
 ↓
IDENTIFY
 ├── not target → continue search
 └── target
       ↓
    APPROACH
       ↓
     FOUND
```

This can initially be implemented as a normal state machine.

Do not introduce an LLM or autonomous-agent framework for basic movement decisions unless there is a clear reason.

Later, an LLM can sit above the deterministic robotics stack for commands such as:

> Find Mom and tell her dinner is ready.

The LLM should translate intent into robot missions, not directly control motors.

---

## Search Strategy

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

The existing medium-motor mechanism moves vertically.

For the first version, the webcam can remain mostly fixed and the tracked robot can rotate its entire chassis to scan horizontally.

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

### Phase 5 — Mapping

Run visual SLAM while manually driving the robot.

Goal:

```text
camera + odometry
        ↓
map + robot pose
```

Do not add autonomous navigation until localization is reasonably stable.

---

### Phase 6 — Autonomous navigation

Introduce Nav2.

Test:

```text
click destination on map
        ↓
robot drives there
```

Then test obstacle avoidance and recovery.

---

### Phase 7 — Autonomous search

Create the search state machine.

Example:

```text
navigate to room
 ↓
rotate 360°
 ↓
run person detection
 ↓
run target identification
 ↓
continue or finish
```

---

### Phase 8 — Higher-Level Intelligence

Only after the robotics stack is reliable, consider:

- voice commands
- speech recognition
- LLM mission planning
- room semantics
- object search
- conversational interaction
- remembering where people were last observed

For example:

```text
"Find Mom"
       ↓
LLM / intent parser
       ↓
FindPerson(target=mom)
       ↓
deterministic robotics mission
```

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

Log enough information to replay failures:

- camera timestamps
- robot pose
- motor commands
- sensor readings
- detections
- mission state

Use ROS bags when useful.

Do not optimize prematurely for perfect autonomy.

---

## Testing Philosophy

Prefer small real-world tests.

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
