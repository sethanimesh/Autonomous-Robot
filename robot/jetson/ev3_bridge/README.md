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

The bridge service sets `FASTRTPS_DEFAULT_PROFILES_FILE` to
`/home/animesh/echora/control_transport.xml`; deploy the adjacent XML there with
the service. Before ROS starts, `ros_node.py` also selects the sibling XML when
neither Fast DDS profile environment variable is set, so an existing service
can use it without a privileged unit update. Explicit operator profiles take
precedence. This selects UDPv4 for this bridge alone after live probes found
that local subscribers could discover its topics but received no status on
the shared-memory path, while UDP delivered status. The profile follows the
[Fast DDS 2.6 UDP transport configuration](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html)
and disables built-in transports; it does not change perception services.

## Safety behavior

- Uses a finite request timeout.
- Refreshes active pulse commands every 100 ms, comfortably inside the EV3's
  independent 500 ms watchdog.
- Always attempts an explicit stop at the end of a pulse, including after an
  exception.
- Limits development pulses to five seconds or less.
- EV3 protocol v6 rejects raw camera-motor speed commands and any absolute
  camera target more than 15 encoder counts from its current position.
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

After a successful boot calibration, named commands are `look_floor`,
`look_person`, and `look_face` (with `look_down`, `look_forward`, and `look_up`
aliases). The `home` command drives at 300 counts/s toward the positive lowered
mechanical end, sets it to encoder 0, and reissues active hold. The real head
remained exactly at 0 during the hold check.

Autonomous moves use targets no larger than 15 counts. The brick watches the
encoder and, after one second without two counts of progress, retries the same
absolute target once. Difficult negative lifting gets the near-rated
1500-count/s recovery; lowering remains at normal speed. A retry never adds
another 15 counts. Encoder
progress is paired with before/after/settled camera frames, and failure means
stop and hold. Every camera command keeps both tracks at zero, and the EV3
rejects non-zero chassis motion until the camera move finishes.

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
python3 ev3_client.py tool-home --speed 300
python3 ev3_client.py tool-move --position 10 --speed 40
```

The client defaults to `192.168.1.25:9999`. Direct camera-head commands are
diagnostic/recovery tools and require the managed ROS bridge to be stopped so
there is only one EV3 client. Normal operation uses the ROS topic.

### Reusing the camera range after a reboot

The physical range is saved, but EV3's relative encoder starts a new reference
after power loss. In the existing UI, return the camera to the same saved lower
view and choose **Use saved range from this lower view**. This translates both
saved endpoints to the new encoder reference while retaining their separation
and the configured inward margins. It does not move or zero the motor. Save
both limits again only if the mounting or desired range has changed.

The recovery action requires explicit lower-view confirmation and stopped,
matching feedback. It saves the translated range atomically and acknowledges
the UI request. It never assumes that a rebooted camera is already at a known
physical angle. This path is covered by controller/UI tests; a physical reboot
recovery trial is still pending.
