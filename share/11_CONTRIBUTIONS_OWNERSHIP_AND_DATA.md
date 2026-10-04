# Engineering contributions, attribution, and data handling

This guide describes the implementation and evaluation status as documented on 2026-10-04. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md). Source line citations refer to the original investigation unless identified as a current-source review.

**C12. Data handling, evaluation infrastructure, and attribution**

Enrollment defaults to numerical templates; optional aligned face crops are separate. Clothing memory retains cropped images and descriptors, and selected crops are sent to Gemini. New recording tools can retain visible people in full camera frames. Therefore the older statement “full camera frames are never stored” is not a project-wide current guarantee.

The recorder is subscriber-only and bounded by duration, frame rate, and storage. It records source/receipt timing, camera/head/encoder state, detections, observations, and observed commands; enrollment vectors are excluded. ROS replay uses an isolated domain and does not republish recorded motion commands. It is a recorded-trajectory replay: a new command cannot change the recorded images or encoders.

The first retained real recording contains 408 frames and 6,882 events. Its mission stopped when the camera head did not move; the operator reported a depleted battery. Since the head never moved, that recording cannot validate cross-view continuity, even though it supports a useful stationary tracker regression.

Engineering attribution distinguishes the project-specific system from its external components:

- **Third-party:** pretrained detectors, embedding model, semantic segmentation, depth model, cloud models, ROS/OpenCV/PyTorch/TensorRT.
- **Repository-specific engineering:** decoders and adapters, synchronization, enrollment workflow, profile/appearance lifecycle, camera and motor coordination, bounded mission logic, result binding, recovery, diagnostics, and evaluation tooling.
- **Project owner:** responsibility for project-specific design, implementation, integration, physical testing, and evaluation, as reported on 2026-10-04. Development notes document coding-agent assistance.

**G. Engineering contributions**

The hospital scenario makes the coordination problem concrete: reach the selected caregiver, obtain usable identity evidence, inspect the route, and decide whether movement or playback is justified. The principal engineering contribution is **coordinating evidence and control across changing camera roles and delayed computation**. It consists of several concrete mechanisms:

1. **Separating identity from current spatial evidence.** Face anchors and durable outfit ownership can survive longer than image coordinates and range. Motion invalidates the latter without automatically erasing all identity history. This directly addresses reacquisition after camera movement.

2. **Binding delayed results to the state that produced them.** Frame keys, image hashes, profile revisions, track IDs, motion epochs, motor generations, head references, and post-response feedback prevent many classes of cross-frame or changed-state result reuse.

3. **A bounded shared-camera control procedure.** Camera leases, motor interlocks, cancellation-aware command ownership, useful-view restoration, floor inspection, short movement, and reacquisition create an executable solution to having one sensor serve incompatible viewing roles.

4. **Motor and feedback integration.** Stop acknowledgements versus telemetry, head backdrive/settling, queued-command timing, source-message arrival order, and physical encoder references required custom handling beyond connecting model APIs.

5. **Recording and replay diagnostics.** The latest recording tools and baseline comparison make a concrete issue reproducible, while keeping encoder travel, model estimates, generated tests, and physical truth distinct.

These contributions concern systems engineering: the integration and coordination of existing perception models, state validation, actuation, and diagnostic tools.

The central design question is **which evidence remains usable after time, motion, viewpoint changes, and asynchronous work**, and how the robot responds when observations are insufficient.

## Deletion boundary in the current implementation

SQLite profile deletion cascades through its clothing records. That does not establish deletion of every historical biometric copy: the preserved legacy enrollment JSON can remain, and the console's cleanup retains face-crop directories referenced by that legacy file. The database migration deliberately preserves that older enrollment. Clothing crops and cloud comparison are independent of the enrollment option for aligned-face-photo retention. These are different data lifecycles.

Source: `robot/jetson/perception/family_store.py:52–63,99–118` and `robot/jetson/perception/enrollment_console.py:1815–1829`. This is a code-derived boundary; no private identity database or image was copied into this folder.

[Back to reading guide](README.md)
