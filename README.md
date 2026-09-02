# Echora Robo

Echora Robo is an indoor tracked robot built from a LEGO EV3 chassis, a Jetson
Orin Nano, and a USB camera. The Jetson is the autonomy computer; the EV3 is the
low-level motor and sensor controller.

The project is being developed in small, testable stages. The immediate goal is
Phase 1: prove safe Jetson-to-EV3 motor control with encoder and IR feedback.

## Working rules

- Default to stopping when communication or sensor data is uncertain.
- Test one physical capability at a time.
- Do not move motors until their ports, orientation, and safe test setup are
  confirmed.
- Record every meaningful test in `docs/DEVELOPMENT_LOG.md`.
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

See:

- [Development log](docs/DEVELOPMENT_LOG.md)
- [Hardware notes](docs/HARDWARE_NOTES.md)
- [Existing EV3 server notes](docs/EXISTING_EV3_SERVER.md)
- [Proposed fail-safe EV3 service](robot/ev3/server/README.md)
- [Jetson EV3 client](robot/jetson/ev3_bridge/README.md)
