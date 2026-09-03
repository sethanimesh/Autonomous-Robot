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

The same bridge owns the vertical camera head so it never competes for the
EV3's single control connection. It accepts `std_msgs/String` commands on
`/camera_head/command` and publishes JSON state on `/camera_head/status`.
During initial calibration, named moves are disabled and only a maximum
15-degree jog is accepted:

```text
ros2 topic pub --once /camera_head/command std_msgs/msg/String \
  "{data: '{\"action\":\"jog\",\"degrees\":10}'}"
```

After physical calibration, supported named commands are `look_forward` and
`look_down`. A `zero` command is available only while the configuration remains
uncalibrated. Every camera command stops or locks both tracks at zero, and the
EV3 rejects non-zero chassis motion until the camera move finishes.

`odometry.py` also contains helpers to estimate effective wheel radius from a
measured straight run and effective track width from a measured turn. Encoder
odometry deliberately starts at `(0, 0, 0)` whenever the bridge service starts.

`calibrate_odometry.py` performs bounded calibration captures through the live
ROS bridge. It saves a JSON report even when a run fails, verifies that both
encoders moved, sends repeated stop commands, and requires stopped feedback.
Use a straight capture first, measure the actual floor distance, calculate the
effective wheel radius, then use that radius for an in-place turn capture.

```text
python3 calibrate_odometry.py capture --motion straight --speed 0.08 --duration 3 \
  --report calibration/straight-01.json
python3 calibrate_odometry.py calculate --report calibration/straight-01.json \
  --measured-distance-m 0.42

python3 calibrate_odometry.py capture --motion turn --speed 0.5 --duration 3 \
  --wheel-radius-m 0.0318 --report calibration/turn-01.json
python3 calibrate_odometry.py calculate --report calibration/turn-01.json \
  --measured-yaw-degrees 185
```

If a later straight run refines the wheel radius, recalculate an existing turn
capture with `--wheel-radius-m VALUE`; the raw report remains unchanged.

The calculated values are printed for review; the tool deliberately does not
rewrite `robot.yaml` automatically. Use the measured floor displacement or yaw,
not the commanded value.

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
python3 ev3_client.py tool-zero
python3 ev3_client.py tool-move --position 10 --speed 40
```

The client defaults to `192.168.1.25:9999`. Direct camera-head commands are
diagnostic/recovery tools and require the managed ROS bridge to be stopped so
there is only one EV3 client. Normal operation uses the ROS topic.
