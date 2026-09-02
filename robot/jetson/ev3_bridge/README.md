# Jetson EV3 Client

`ev3_client.py` is the first Jetson-side Phase 1 bridge component. It speaks the
EV3 service's newline-delimited JSON protocol using only Python's standard
library.

It is deployed at `/home/animesh/echora/ev3_client.py` and has passed ping,
status, paired-track, and differential-turn tests against the managed EV3
service.

## Safety behavior

- Uses a finite request timeout.
- Refreshes active pulse commands every 100 ms, comfortably inside the EV3's
  independent 500 ms watchdog.
- Always attempts an explicit stop at the end of a pulse, including after an
  exception.
- Limits development pulses to five seconds or less.
- Closes and discards a connection after framing or transport failure.
- Relies on the EV3 watchdog if the network disappears before a stop arrives.

This is not yet a ROS 2 node and does not convert `/cmd_vel` into track speeds.
That conversion requires physical forward-direction and chassis-geometry
calibration first.

## Commands

```text
python3 ev3_client.py ping
python3 ev3_client.py status
python3 ev3_client.py stop
python3 ev3_client.py pulse --left 80 --right 80 --duration 0.25
```

The client defaults to `192.168.1.25:9999`.
