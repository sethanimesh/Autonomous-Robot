# Requirements, deployment context, and limits of reproduction

This document explains what the robotics system depends on and what can be established from its repository and technical dossier. It is a description of the inspected implementation, not an operating procedure. It contains no executable source, launch instructions, credentials, enrolled identities, or camera images.

Evidence references use repository-relative filenames and line numbers from the inspected working tree. The investigation identified branch `main` at commit prefix `1e8fbb63`, with uncommitted changes. Consequently, a statement about the inspected source does not establish that the same revision was running on the robot during a historical demonstration. The dossier's snapshot and coverage records should accompany this document.

## What the shared Markdown documents provide

The documents preserve the system's architecture, interfaces, controlling parameters, error handling, recorded observations, and the distinction between implemented behavior and demonstrated results. They let a reader trace why the system needs a camera reference, why it stops between movement segments, how a delayed visual result is tied to the robot's state, and which missing evidence limits stronger claims.

The documents cannot execute the robot, rebuild inference engines, recreate private enrollments, recover a physical camera pose, or reproduce a historical mission. Even a complete source copy would need the hardware, software environments, model assets, runtime configuration and physical preparation described below. A Markdown-only evidence package provides substantially less than a runnable source release: its purpose is technical understanding and evaluation of claims.

## The three local computers and the cloud

The implementation distributes work across a Jetson, an EV3 brick and a companion Mac. The Jetson acquires camera frames, runs person and face processing, coordinates the mission, and emits motion commands. The EV3 executes bounded motor operations and supplies encoder feedback. The Mac hosts heavier scene/depth processing and acts as the intermediary for cloud visual requests. Cloud responses are advisory inputs to deterministic local decisions; they are not an EV3 motor interface.

The source is organized under `robot/jetson`, `robot/ev3` and `robot/mac`. Runtime deployment does not mirror that tree exactly. The Jetson build collects top-level Python modules from the camera, bridge, mission, navigation and perception directories into a single runtime directory. Many modules therefore use bare imports that are meaningful in that flattened installation. The Mac backend is invoked as a Python module using the repository as its package root. These different import arrangements are a reproducibility dependency in their own right. Evidence: `scripts/phase6/phase6.py:27–47`; `robot/mac/com.echora.route-perception.plist:7–24`.

The checked-in deployment descriptions contain machine-specific working directories, user accounts and network endpoints. Those values identify the development arrangement; they are not portable defaults for a new installation. Account names, private addresses and cloud project identifiers are intentionally not reproduced here.

## Hardware requirements and physical assumptions

### Jetson and USB camera

The documented development platform is an NVIDIA Jetson Orin Nano developer kit. The inventory records Ubuntu 22.04.5 LTS, NVIDIA Linux for Tegra 36.4.3, approximately 7.4 GiB of RAM and Python 3.10.12. These are historical observations rather than a freshly verified environment manifest. Evidence: `docs/HARDWARE_NOTES.md:15–32`.

The camera path expects a V4L2 device capable of MJPG at 640 × 480. The requested rate is 30 frames per second, with two seconds of discarded warm-up frames after each open. The software checks frame dimensions rather than silently resizing an unexpected capture mode. The selected resolution is also the resolution of the stored intrinsic calibration. Evidence: `config/camera.yaml:11–24`; `robot/jetson/camera/camera_node.py:138–193`; `robot/jetson/camera/frame_health.py:22–41`.

The observed rate is lighting-dependent. The hardware notes distinguish approximately 27.3 frames per second under a bright ceiling view from 16–18 frames per second in dimmer room conditions. Those observations should not be converted into a guaranteed camera or end-to-end mission rate. Evidence: `docs/HARDWARE_NOTES.md:230–236`.

USB cabling is an operating constraint. Historical short stability checks were followed by disconnects and prolonged re-enumeration failures. The documented cable/support acceptance conditions include strain relief, a slack loop clear of moving parts, and a marked neutral orientation. A copied configuration cannot verify those conditions. Evidence: `docs/HARDWARE_NOTES.md:182–202`; `docs/CABLE_AND_CAMERA_SUPPORT.md:3–40`.

### EV3, tracks and camera linkage

The EV3 implementation expects an ev3dev Linux environment exposing tacho motors through sysfs. It requires motor A for the camera head, B for the left track and C for the right track. The server accesses sysfs directly and is written for Python 3.5 using its standard library; ev3dev2 is not required by this server implementation. Evidence: `robot/ev3/server/ev3_server.py:1–6,44–48,59–159,871–885`.

The source and documents do not agree consistently on physical motor type. The project instructions describe motor A as physically large, the hardware table later describes a medium-motor driver, and legacy inventory notes describe all three motors as large. This may reflect hardware changes or probe behavior, but the inspected material does not settle it. A reader should retain the port mapping while treating the physical motor-type claim as unresolved. Evidence: `AGENTS.md:403–404`; `docs/HARDWARE_NOTES.md:68–70`; `docs/EXISTING_EV3_SERVER.md:27–28`.

The chassis envelope is documented as 20 cm wide by 25 cm long. A later lens-height measurement is 15.24 cm at one lower view; that does not establish lens height or optical pitch at raised views. The effective drive radius and track width in configuration are 0.0144504 m and 0.182557 m. They were derived from short displacement and rotation measurements and are not literal sprocket dimensions. Surface-dependent track slip remains a limitation. Evidence: `docs/HARDWARE_NOTES.md:75–77`; `config/robot.yaml:5–10`; `docs/DEVELOPMENT_LOG.md:1200–1251`.

The infrared sensor was absent and explicitly deferred in the inspected hardware notes. Neither the current motor server nor the bridge path inspected here implements an infrared proximity stop. A recipient should not assume that an additional physical obstacle sensor accompanies the camera-based checks. Evidence: `docs/HARDWARE_NOTES.md:71–72,147`; `robot/ev3/server/ev3_server.py:648–670,871–885`.

## Required software environments

| Execution location | Dependencies supported by the inspected material | Reproduction limit |
| --- | --- | --- |
| EV3 | Python 3.5-compatible standard library, ev3dev tacho-motor sysfs and the expected motor ports | A generic desktop Python environment cannot reproduce motor behavior, driver re-enumeration, loaded movement or physical stopping. |
| Jetson camera and bridge | ROS 2 Humble Python environment; `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`; OpenCV, NumPy and PyYAML | The repository's Python files do not include the installed ROS distribution or all native dependencies. |
| Jetson image and calibration consumers | `cv_bridge`; OpenCV with ArUco/ChArUco support for calibration tools | A different OpenCV installation may lack the required calibration module or behave differently. |
| Jetson neural inference | NVIDIA CUDA/TensorRT environment and separately provisioned model weights and engine assets | One model's TensorRT configuration does not establish the backend or precision of every other model in the system. |
| Mac route/depth backend | Python, PyTorch, Transformers, Pillow, OpenCV and NumPy; configured Apple MPS execution path | The Python environment and downloaded model cache are external to Git. Hardware and package versions affect availability and timing. |
| Cloud requests | `google-auth[requests]`, separately configured Vertex application-default authentication, project/quota access and the selected available model | Credentials, cloud entitlement, provider behavior, latency and billing are not reproduced by documentation or source. |

Evidence for these dependencies appears in the imports of `robot/jetson/camera/camera_node.py:14–37`, `robot/jetson/camera/calibrate_charuco.py:11–17`, `robot/jetson/ev3_bridge/ros_node.py:11–24`, and `robot/jetson/mission/camera_head_calibration.py:837–840`; in `robot/mac/route_perception.py:82–105`; in `robot/mac/person_range.py:247–259`; and in `robot/mac/README.md:88–95`.

The historical inventory records a CPU-only PyTorch installation on the Jetson despite CUDA runtime files being present. That observation does not contradict separately implemented TensorRT inference, but it makes a generic statement such as “all inference uses GPU PyTorch” unsupported. Evidence: `docs/HARDWARE_NOTES.md:29–31`.

No complete lockfile for every native, ROS, Jetson and Mac dependency was established by this hardware/runtime review. The listed environment requirements explain dependencies; they are not an independently validated compatibility matrix.

## Model assets that the documents do not contain

The checked-in configuration names three Jetson inference assets:

- YOLOX-s for person detection, using a 640 × 640 model input and a TensorRT engine path. An ONNX path exists for the explicit CPU fallback implementation, while the checked-in runtime selection requires TensorRT.
- YuNet 2023mar for face detection, with its own 640 × 640 engine path.
- AntelopeV2 Glint360K ResNet-100, identified in configuration as `antelopev2_glintr100`, for face embeddings.

Evidence: `config/perception.yaml:27–45,72–78,119–124`. ONNX files, TensorRT engines, engine sidecars and model directories are explicitly ignored by Git. The repository points to engine-build tooling and model provenance separately. A model identifier, hash or filename does not include the numerical weights, prove the engine was loaded during a particular run, or make an engine portable across runtime/hardware combinations. Evidence: `.gitignore:10–17`.

The Mac backend names `nvidia/segformer-b0-finetuned-ade-512-512` for segmentation and `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf` for depth. The latter is the precise configured model family and variant; its name alone is not validation of physical distance accuracy on this camera and room. Weights are loaded through Transformers, and the environment/model cache are deliberately outside the repository. Evidence: `robot/mac/route_perception.py:26,82–105`; `robot/mac/person_range.py:10,247–259`; `robot/mac/README.md:13–19`.

Cloud model choice is centralized in `robot/cloud_models.py`. Historical recorded trials used earlier selections and combinations, so their timing or success cannot automatically be assigned to whichever model identifier is present in a later source snapshot. Model access and current availability were not tested in the read-only investigation.

## Calibration and runtime state that cannot be transferred as generic truth

### Lens intrinsics

The stored calibration describes one camera at 640 × 480, with approximately fx = 416.371 px, fy = 413.608 px, cx = 338.723 px and cy = 235.303 px, using a five-coefficient `plumb_bob` distortion model. Missing, invalid or differently sized calibration produces explicitly uncalibrated camera information. Evidence: `config/camera_calibration.yaml:1–55`; `robot/jetson/camera/camera_calibration.py:153–185`.

The recorded calibration has an important qualification: its automatic report says `accepted: false`. Thirty views produced RMS reprojection error of approximately 0.596 px, but the dataset did not meet vertical edge coverage and the required number of close views. The development log explains that the model was subsequently promoted after live timestamp and visual rectification checks. Both the numeric report and this override must remain visible in any explanatory package. Evidence: `docs/calibration/camera_calibration_capture_report_20260903.json:2,34–38,60–61,94–107`; `docs/calibration/README.md:5–22`; `docs/DEVELOPMENT_LOG.md:940–950`.

The capture process publishes raw images with camera information. Publication of calibration metadata is not the same as rectifying every image used downstream. The person-range implementation explicitly undistorts image points, whereas the camera node publishes raw BGR frames. Evidence: `robot/jetson/camera/camera_node.py:280–301`; `robot/mac/person_range.py:38,103`.

### Motor reference and useful camera views

Camera-head coordinates are measured motor encoder counts tied to a reference epoch. They are not direct measurements of optical tilt. Rebooting, re-zeroing or re-enumerating the motor can invalidate old coordinates. Runtime saved limits may override the historical values in `config/robot.yaml`.

The server retains a reference only when the boot/device identity matches its saved record. The bridge invalidates named camera views when the reference or admissible range no longer matches. Evidence: `robot/ev3/server/ev3_server.py:120–125,241–258,325–334`; `robot/jetson/ev3_bridge/camera_head.py:715–760`; `robot/jetson/ev3_bridge/ros_node.py:218,286–289`.

Useful-view discovery establishes operating views for floor/room inspection and higher person framing. It checks a bounded sequence of stopped camera poses, classifies fresh images, and verifies return views before saving both endpoints. It does not infer mechanical stops or establish full geometric camera-to-chassis calibration. Evidence: `robot/jetson/mission/camera_visual_setup.py:63–205`.

A concrete historical example illustrates the distinction: an encoder midpoint faced the ceiling, while the lower verified view showed the room and floor. The implementation then used the verified lower view for forward search. Copying a midpoint formula or old named angles would not reproduce the useful view. Evidence: `docs/DEVELOPMENT_LOG.md:2321–2338`.

### Identity and appearance data

Private identity enrollment, face embeddings, retained enrollment crops, identity/appearance database state and session-specific confirmation evidence are separate from application source. This document does not include any of them. An empty installation cannot recognize a specified recipient merely because it has the recognition model: it needs deliberately established reference identities and the configuration selecting the intended recipient.

The older target store path is explicit in `config/perception.yaml:124`; later family identity and appearance behavior is covered separately in the dossier. Historical runtime data should not be presented as an interchangeable example dataset. Its timestamps, consent context, model version and image provenance affect what can be reproduced.

## Communication and deployment context

Within the Jetson, camera publication and robot coordination use ROS topics. The bridge subscribes to velocity and camera-head commands and publishes robot/head status, odometry and joint state. It emits the odometry-to-base transform. The hardware/runtime search did not find a camera-to-base extrinsic transform or a complete URDF model in the inspected paths. Evidence: `robot/jetson/ev3_bridge/ros_node.py:305–321,357–419`.

Between Jetson and EV3, the implementation uses sequential newline-delimited JSON over TCP. Commands are motor speeds or bounded camera targets; the bridge converts chassis velocity to wheel speeds. The server accepts one client and uses a configured network-address restriction. These defaults must correspond to the actual deployment network. Evidence: `robot/ev3/server/ev3_server.py:20–48,674–718,753–815`; `robot/jetson/ev3_bridge/kinematics.py:26–35`.

The Mac backend and Jetson communicate using HTTP endpoints, with a recorded reverse SSH tunnel exposing the backend to Jetson through loopback. Reproducing that relationship requires separately configured SSH identity, trust and reachability. Neither those assets nor cloud credentials belong in a shareable explanatory package. Evidence: `robot/mac/com.echora.jetson-backend-tunnel.plist:7–18`; `scripts/phase6/README.md:104–111`.

The existing Phase 6 bundle is an update mechanism for an already prepared Jetson. Its manifest covers flattened Jetson Python files, transport descriptions and the runner. It omits the EV3 implementation, Mac implementation, initial model assets, enrollments and physical camera limits. Optional configuration changes merge into existing runtime YAML. Therefore, even that original code bundle is not a complete installation image. Evidence: `scripts/phase6/phase6.py:27–47,75–97`; `scripts/phase6/README.md:67–89`.

## What evidence can be reproduced at each level

| Material available | Reasonable conclusion | Conclusion that still needs additional evidence |
| --- | --- | --- |
| Markdown dossier and cited excerpts | Understand architecture, algorithms, constraints and recorded outcomes | Execute code or independently rerun the original tests |
| Source and matching software dependencies | Inspect or exercise pure logic, mocks and replay tools | Reproduce GPU timing, hardware behavior or successful physical arrival |
| Historical test logs | Establish that the recorded test invocation reported its result | Establish that the present dirty tree passes, or that a mocked condition occurred physically |
| Numeric calibration/run reports | Recalculate stated metrics and inspect recorded decisions | Independently verify unrecorded ground truth or recover missing image context |
| Hardware, current calibration and private enrollment | Conduct a new prepared trial under documented conditions | Assume that historical results transfer unchanged to new lighting, room layout, battery state or participants |

Some original odometry captures were retained only on the Jetson and excluded from Git; the local documentation provides summarized physical measurements. Their absence should remain an explicit evidence limit. Evidence: `docs/DEVELOPMENT_LOG.md:1167–1169,1200–1251`.

Recorded images also have limits even when privately available: replay can evaluate processing against the recorded observations, but those images cannot show what a different camera movement or route would have seen. The dossier distinguishes numerical replay, mocked simulations, recorded software execution and physical demonstrations.

## Reliability claims that copying cannot establish

The current code implements a Jetson command timeout and an EV3-local motion watchdog, head/chassis exclusion, camera reference checks and explicit stop paths. It does not establish a universal physical stopping guarantee. The EV3 watchdog runs in the same process loop as request handling and sysfs operations; it depends on that process continuing to execute. A copied file cannot verify driver behavior, battery capacity, braking distance or response under every fault. Evidence: `robot/ev3/server/ev3_server.py:127–145,623–646,827–868`; `robot/jetson/ev3_bridge/ros_node.py:445–488`.

Similarly, a copied metric-depth model and calibration file cannot establish correct person distance. The development record reports strong scene-dependent distance/pitch discrepancies despite a high fitted plane-inlier rate. That observation limits claims of physical standoff accuracy until independent validation is available. Evidence: `docs/DEVELOPMENT_LOG.md:2581–2587`.

The hardware/runtime evidence supports a development robot with bounded movement, layered software checks and explicit recovery mechanisms. Reproduction of those mechanisms, reproduction of a particular demonstration, and validation for an assistance scenario are separate undertakings. No claim of hospital deployment, measured clinical benefit or validated emergency response follows from the requirements documented here.

## Scope of the privacy review

A read-only pattern scan covered 77 selected hardware/runtime source, document and configuration files for recognizable API keys, refresh tokens, private-key headers, credential-bearing URLs and literal sensitive assignments. It found no candidate secret in that scope. This was a limited heuristic review, not an exhaustive security audit of the repository, caches, recordings or all run artifacts.

Original runtime descriptions and historical notes contain personal account paths, private network topology, a cloud project identifier, family labels and links to camera images. Those are omitted here. A Markdown-only share should make clear when an original reference points to intentionally excluded imagery or machine-specific state, so an absent attachment is not mistaken for evidence that never existed.

[Back to reading guide](README.md)
