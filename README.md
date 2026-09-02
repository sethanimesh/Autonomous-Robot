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

The Mac, Jetson, and EV3 have been inventoried over SSH. ROS 2 Humble is already
installed on the Jetson, and all three EV3 output ports A-C currently report
large motors. Phase 1 motor work is paused for physical port confirmation and a
safe wheels-off-ground test setup.

The Jetson previously detected the USB `Arducam_8mp`, but it is not currently
enumerated. Kernel messages report repeated USB errors and suggest a bad cable or
connection. No EV3 sensor is currently detected.

See:

- [Development log](docs/DEVELOPMENT_LOG.md)
- [Hardware notes](docs/HARDWARE_NOTES.md)
- [Existing EV3 server notes](docs/EXISTING_EV3_SERVER.md)
