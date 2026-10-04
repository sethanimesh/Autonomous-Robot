# Hospital scenario and approved request delivery

Echora is presented as **On-Call Hospital Assistance with Recipient Directed Care Coordination**. The motivating scenario is a patient seeking urgent help from an assigned nurse or doctor who is occupied elsewhere in the same room and does not have a phone in hand. The robot should locate that named, pre-enrolled caregiver and bring the approved request directly to their attention. The prototype is evaluated in a home setting; hospital deployment and emergency-response performance remain future evaluation areas.

## Recipient-directed robotics

The prototype selects a profile rather than simply approaching the nearest visible person. YOLOX-s detects people, YuNet locates faces and landmarks, and InsightFace embeddings support repeated identity comparisons on the Jetson. TensorRT FP16 engines accelerate these stages; other parts of the pipeline use different backends. SQLite stores enrolled profiles and appearance memory.

A single motorized webcam provides person views and floor views. The mission coordinates camera preparation, bounded search, identity assessment, floor inspection, turning and rechecking, short movement, and target reacquisition. Clothing supports a previously associated identity without becoming a fresh facial confirmation. Current family mode can nevertheless authorize acquisition, approach, arrival, and delivery using qualifying clothing identity, including a saved outfit without a new face observation in that mission. The mission therefore distinguishes current facial confirmation from appearance-supported identity.

ChArUco calibration supplies intrinsics and distortion parameters, with the recorded manual-acceptance exception preserved in the [calibration guide](03_HARDWARE_CAMERA_AND_CALIBRATION.md). Mac-side Depth Anything V2 and SegFormer-B0 provide approximate range and candidate floor corridors; Gemini supplies structured hazard advice. These are camera-based estimates and route heuristics with known range discrepancies, not complete three-dimensional clearance guarantees.

The Jetson retains mission and movement authority. The companion Mac handles heavier visual computation and cloud calls asynchronously; returned assessments use source-image and applicable identity, camera-reference, and motion-state checks. The serialized camera-command worker allows feedback processing during head movement and fences cancelled actuator calls. The EV3 executes low-level commands and locally stops when drive refreshes expire. See the [distributed execution guide](07_DISTRIBUTED_EXECUTION_AND_SAFETY.md) for the limits of those checks.

## Current approved-message path

The 2026-10-04 source review identifies an implemented supervised delivery path:

1. Select an enrolled profile and enter exact text or record speech for transcription.
2. Review/edit the transcript and approve the message bound to profile and message revisions.
3. Obtain synthesized audio through the Mac service and preview it on the Jetson speaker.
4. Start **Find & deliver** with the normal readiness and cable-neutral checks.
5. After the mission, repeat the identity, validated measured-range, matching-track, and stopped tracks/head checks before permitting playback.

Search-only **Find person** never speaks. Approximate/estimated arrival cannot authorize delivery. Changed approvals, stale or ambiguous identity, unsupported final distance, unavailable speech, or uncertain stopped feedback leave the robot silent. The delivery gate permits identity sourced from face or clothing; it does not require a fresh face at playback.

`played` records completion of the audio playback process. It does not establish that the recipient heard or understood the message. The code has no recipient acknowledgement mechanism, voice-command mission start, or open conversation feature. See [runtime contract](../docs/ROBOT_MESSAGE_DELIVERY.md), [console integration](../robot/jetson/perception/enrollment_console.py), [approval/playback/gate code](../robot/jetson/perception/speech_delivery.py), and [Mac ASR/TTS service](../robot/mac/voice_delivery.py).

## Demonstration and evaluation status

The project owner reported a complete home demonstration on 2026-10-04, covering selected-recipient finding, approach, request playback, and human acknowledgement. Retained artifacts provide additional measurements of partial approach and reacquisition trials. Trial count, timing, independently measured stopping distance, and a recording of the complete demonstration are not documented. Hospital participants, patient outcomes, emergency-response timing, and care-coordination benefit remain future evaluation areas.

[Back to reading guide](README.md)
