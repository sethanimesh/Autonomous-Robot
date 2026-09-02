# Jetson USB Camera Source

`camera_node.py` is the Phase 3 camera acquisition node. It captures frames
from the Jetson's USB camera and publishes them as ROS 2 messages. It does
nothing else: no detection, no recognition, no recording, and no motor
command. It holds no connection to the EV3.

## Topics

| Topic | Type | QoS |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` (`bgr8`) | sensor data, best effort, depth 5 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | sensor data, best effort, depth 5 |
| `/camera/status` | `std_msgs/msg/String` (JSON) | reliable, transient local, depth 1 |

Image and CameraInfo are published in the same call and always carry the same
ROS timestamp and the same `frame_id`.

Because the image topics use best-effort sensor QoS, a subscriber must also
use best-effort QoS or it will receive nothing. A slow subscriber will drop
frames by design; measure publication with `max_frame_gap_sec` on
`/camera/status` rather than with a heavy subscriber. Note that `ros2 topic hz`
on Humble has no QoS flags and cannot subscribe to these topics.

## Hardware findings that shaped this node

Measured on the Jetson Orin Nano against USB camera `0c45:6366`:

- `/dev/video0` is the capture node. `/dev/video1` is a metadata node with no
  capture capability and cannot be opened by OpenCV.
- The camera offers MJPG at 640x480/30, and YUYV at 640x480/**10 only**.
  OpenCV negotiates YUYV by default, so `fourcc: MJPG` is required for a
  usable rate.
- The sensor needs about two seconds of auto-exposure settling after every
  open. Frames captured before that are dark and duplicated, so the node
  discards frames for `warmup_sec` before it publishes anything.
- Filling `sensor_msgs/Image.data` with `bytes` costs ~152 ms per frame on
  this build of rclpy, and a numpy array is rejected outright. The node uses
  `array.array("B", ...)`, which costs ~0.1 ms per frame.
- MJPG 640x480 sustains about **27.3 fps** against a requested 30.

## Parameters

Configured in `config/camera.yaml`, deployed as
`/home/animesh/echora/camera.yaml`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `video_device` | `/dev/video0` | Device path, or a bare number for an index |
| `image_width` / `image_height` | 640 / 480 | Requested and validated frame size |
| `requested_fps` | 30.0 | Requested rate; also the capture timer period |
| `fourcc` | `MJPG` | Pixel format requested from V4L2 |
| `frame_id` | `camera_optical_frame` | ROS frame on both image and camera info |
| `reconnect_interval_sec` | 2.0 | Minimum delay between open attempts |
| `warmup_sec` | 2.0 | Frames discarded after each open |
| `max_read_failures` | 15 | Consecutive failures before release and reopen |
| `read_failure_pause_sec` | 0.02 | Pause after a single failed read |
| `status_interval_sec` | 5.0 | `/camera/status` period |
| `calibration_file` | `''` | Optional ROS calibration YAML |

## Calibration

**This camera has not been calibrated.** While `calibration_file` is empty the
node publishes an explicitly uncalibrated `CameraInfo`: `D`, `K`, `R` and `P`
are all zeroed and `distortion_model` is empty, which is the documented
`sensor_msgs/CameraInfo` marker for an uncalibrated camera (`K[0] == 0.0`).
`/camera/status` reports `"calibrated": false` with the reason.

Intrinsics are never invented. A calibration file is refused, with the reason
reported, when it is missing, unparseable, zeroed, or recorded at a different
resolution than the configured one. Nothing in this milestone may be used for
metric vision until a real calibration is performed.

## Failure handling

- Every frame is validated for shape, channel count, dimensions, dtype, and
  non-emptiness before publication.
- A single failed read is counted and briefly paused, not treated as fatal.
- After `max_read_failures` consecutive failures the device is released and
  reopened, paced by `reconnect_interval_sec` so a missing camera cannot
  produce a busy loop. Measured CPU while disconnected is 1.5% of one core
  against 24.6% while streaming.
- Recovery from a disconnect is automatic and was verified three times by
  deauthorizing and reauthorizing the USB device.
- Shutdown releases the camera. After Ctrl-C the rclpy signal handler has
  already invalidated the context, so the final summary goes to stdout.

## Running

Managed by enabled `echora-camera.service` on the Jetson. To run by hand,
stop the service first so the device is free:

```text
source /opt/ros/humble/setup.bash
python3 camera_node.py --ros-args --params-file camera.yaml
```

## Hardware-free logic

`camera_config.py`, `frame_health.py`, and `camera_calibration.py` import no
ROS, OpenCV, or numpy, so configuration validation, frame validation, failure
counting, reconnect timing, rate and gap measurement, duplicate detection,
status generation, and calibration handling are all covered by the repository
test suite.
