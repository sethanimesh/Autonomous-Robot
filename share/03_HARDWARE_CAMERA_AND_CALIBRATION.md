# Hardware, camera control, and calibration

This guide presents the implementation and evaluation status. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md).

**C3. Hardware, camera control, and calibration**

The architecture is supported by concrete motor and sensor code:

- B and C drive the tracks.
- A drives a single vertical camera mechanism.
- Horizontal scanning rotates the chassis.
- EV3 feedback contains encoder positions, measured speeds, motor state, generation identifiers, and commanded speeds.
- The current EV3 service uses direct Linux sysfs and the Python standard library, not `ev3dev2`.
- No working IR reading or local IR obstacle-stop path was found. Hardware notes explicitly defer the absent/nonworking sensor.

See motor implementation, lines 44–159 — `robot/ev3/server/ev3_server.py:44` and [hardware inventory and sensor limitation, lines 56–87](<../docs/HARDWARE_NOTES.md>) (source line 56).

The recorded chassis envelope is **20 × 25 cm**. Effective odometry geometry is:

- Drive radius: **0.0144504 m**
- Track width: **0.182557 m**
- Encoder counts/revolution: **360**
- Bridge motor-speed cap: **240 counts/s**
- Feedback/update rate: **10 Hz**

These are effective tracked-chassis parameters derived from floor tests, not necessarily physical sprocket dimensions. Track slip remains unmodelled. See robot configuration, lines 5–15 — `config/robot.yaml:5` and calibration measurements, lines 1200–1251 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 1200).

The repository contains contradictory motor inventory descriptions: the supplied AGENTS guidance says A was physically identified as large; hardware notes call A medium; earlier server notes identify all three as large. The generic sysfs control path tolerates this disagreement, but it cannot establish the physical motor type.

Camera positioning measures the **motor encoder**, not optical pitch. Current EV3 safeguards include:

- Absolute raw position bound: ±180 counts.
- Maximum position-command step: 15 counts.
- Head-move deadline: 8 seconds.
- Completion tolerance: 4 counts, speed ≤5 counts/s, and stable position for 0.4 seconds.
- Stall criterion: less than 2 counts of progress in one second.
- One retry of the same target, then stop and hold.
- Both tracks stopped during a head movement.

See head completion and stall handling, lines 302–426 — `robot/ev3/server/ev3_server.py:302` and head/track interlocks, lines 429–524 — `robot/ev3/server/ev3_server.py:429`.

Named positions are valid only within a reference epoch. Boot changes, motor re-enumeration, or lost reference can invalidate old counts. Saved runtime limits override static values. Consequently, historical poses such as floor `0`, forward `−27`, face `−42` should not be presented as permanent optical angles.

The camera has two separate forms of calibration:

1. **ChArUco intrinsic calibration:** image geometry and lens distortion.
2. **Useful-view calibration:** head encoder positions that show suitable floor/room/upper views.

ChArUco uses a 5×7 board, 25 mm squares, 18 mm markers, and `DICT_5X5_100`. The accepted YAML is 640×480 with approximately `fx=416.371`, `fy=413.608`, `cx=338.723`, `cy=235.303`. Capture publishes raw images and matching `CameraInfo`; it does not rectify all camera pixels. Ranging explicitly undistorts rays; person/face and segmentation paths consume raw imagery.

A consequential exception must remain visible: the retained calibration report says **`accepted: false`**. Its 30-view fit had RMS **0.5958 px**, maximum per-view error **1.3512 px**, and a 609×450 valid ROI, but did not meet vertical-edge coverage and close-view-count requirements. Documentation records **manual promotion after live visual and timestamp checks**. It did not pass every automatic gate. See capture report — `docs/calibration/camera_calibration_capture_report_20260903.json:1` and [acceptance explanation, lines 5–22](<../docs/calibration/README.md>) (source line 5).

Current useful-view discovery is bounded: approximately ±60 encoder counts from the starting reference, 10-count exploration steps, at most 20 observations, bounded provider cooldown, and endpoint return checks. It can acknowledge the current encoder position without mechanical homing. Its result establishes usable operating views, not measured optical tilt or proven mechanical clearance. See discovery policy, lines 63–205 — `robot/jetson/mission/camera_visual_setup.py:63`.

[Back to reading guide](README.md)
