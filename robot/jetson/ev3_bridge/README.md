# Jetson EV3 Client

`ev3_client.py` is the first Jetson-side Phase 1 bridge component. It speaks the
EV3 service's newline-delimited JSON protocol using only Python's standard
library.

It is deployed at `/home/animesh/echora/ev3_client.py` and has passed ping,
status, paired-track, and differential-turn tests against the managed EV3
service.

The ROS node is managed by enabled `echora-bridge.service` on the Jetson. While
it is running, it intentionally owns the EV3's single allowed control connection;
use `/robot_status` instead of opening a second direct client.

## Safety behavior

- Uses a finite request timeout.
- Refreshes active pulse commands every 100 ms, comfortably inside the EV3's
  independent 500 ms watchdog.
- Always attempts an explicit stop at the end of a pulse, including after an
  exception.
- Limits development pulses to five seconds or less.
- Closes and discards a connection after framing or transport failure.
- Relies on the EV3 watchdog if the network disappears before a stop arrives.

`ros_node.py` provides the first `/cmd_vel` vertical slice and publishes raw
JSON EV3 feedback on `/robot_status`, encoder odometry on `/odom`, track angles
on `/joint_states`, and the `odom → base_link` transform. Wheel radius, track
width, and motor signs are parameters. The checked-in `config/robot.yaml` values
are provisional until physical measurement and floor calibration are complete.

`odometry.py` also contains helpers to estimate effective wheel radius from a
measured straight run and effective track width from a measured turn. Encoder
odometry deliberately starts at `(0, 0, 0)` whenever the bridge service starts.

Run after sourcing ROS 2 Humble:

```text
python3 ros_node.py --ros-args --params-file robot.yaml
```

## Commands

```text
python3 ev3_client.py ping
python3 ev3_client.py status
python3 ev3_client.py stop
python3 ev3_client.py pulse --left 80 --right 80 --duration 0.25
```

The client defaults to `192.168.1.25:9999`.
