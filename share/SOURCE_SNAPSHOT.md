# Source snapshot and documentation revision provenance

## Original packaging record

- Documentation packaged at: 2026-09-26T19:49:52+00:00.
- Inspected branch: `main`.
- Inspected HEAD: `1e8fbb63b7bb09c54e2c065aa39fd336e6843a63`.
- The original investigation found 54 modified tracked files plus substantial untracked work. This is a working-tree investigation, not a description of HEAD alone.
- The Markdown report was exported from the assistant's completed dossier, not reconstructed from a shortened summary. Only local citation links were adapted for portability.
- Topic guides are extracted from that complete dossier. The data-handling guide adds one explicit deletion-boundary clarification from the source review. The reproduction guide explains external requirements; it does not claim a verified installation procedure.
- The investigation did not run project tests, models, cameras, motors, cloud calls, or services. This packaging step created documentation copies only.
- The source repository's original files were left unchanged.
- No credentials, databases, face images, raw embeddings, model weights, videos, or camera recordings are bundled. Original Markdown notes retain historical machine paths, device addresses, model names, and family labels; this is documentation rather than an anonymized dataset.

## Current framing and source review (current)

The project is now presented as **On-Call Hospital Assistance with Recipient Directed Care Coordination**. This documentation revision preserves the original snapshot, commit history, reported measurements, and packaging hashes below. It updates the intended application, repository navigation, and current supervised robot-delivery findings; it does not retroactively alter the original investigation or establish a new physical result.

Current source review includes `docs/ROBOT_MESSAGE_DELIVERY.md`, the console delivery integration, `speech_delivery.py`, `robot/mac/voice_delivery.py`, and the bridge camera-command worker. The current console has a reviewed-message → selected-profile → supervised mission → gated Jetson playback path, superseding the original finding that no connected delivery path was present. Recipient acknowledgement is not implemented; playback completion is not proof of hearing or understanding.

The project lead confirmed responsibility for project-specific design, implementation, integration, physical testing, and evaluation, with a complete home demonstration covering finding, approach, playback, and human acknowledgement. These details supplement the historical artifact-backed results. Trial count, timing, independently measured stopping distance, and a recording of the complete demonstration are not documented. The software does not record recipient acknowledgement. Third-party pretrained models and frameworks are attributed separately.

The old `share/reference_docs/` export is absent here. Current links point to existing repository documentation and source; the ignored adjacent `communication/` app and local raw artifacts are cited as unavailable public evidence. Source line numbers can shift when files change. The current project revision passed 138 offline checks through [the reproduction harness](../scripts/diagnostics/reproduce_checks.py) under Python 3.14.7 on macOS using the standard library with `-S` (0.175 seconds). This validation is separate from the original no-test investigation. No live hardware, camera, model inference, network service, or remote runtime was exercised; see the [evaluation guide](../evaluation/README.md) for check coverage.

## Recent committed history

The original investigation counted 43 reachable commits under one author name. The selected history below ends before much of the newer uncommitted functionality; contribution attribution is described in the engineering contributions guide.

```text
1e8fbb6 2026-09-04 sethanimesh add automatic camera head calibration
d84052d 2026-09-04 sethanimesh expand camera head face search range
871fd9e 2026-09-04 sethanimesh document short EV3 test windows
8db6e19 2026-09-04 sethanimesh make vertical person search robust
2c612d7 2026-09-04 sethanimesh add intelligent vertical face search
ef574fc 2026-09-04 sethanimesh scan high camera views before chassis turns
6e7bcbc 2026-09-04 sethanimesh guide camera upward from person detections
0b0cbd6 2026-09-04 sethanimesh add autonomous single-room find mission
63b18fd 2026-09-04 sethanimesh add cable-safe target scan runner
d537411 2026-09-04 sethanimesh add single-room target mission core
f5d0c4a 2026-09-04 sethanimesh execute first closed-loop camera detour
2a25417 2026-09-04 sethanimesh detect blocked routes from floor segmentation
69e3974 2026-09-04 sethanimesh keep camera operator page active
97772ee 2026-09-04 sethanimesh anchor monocular depth to floor geometry
bf822c9 2026-09-04 sethanimesh start camera-only local route planning
```

## Historical reference-copy manifest

The original packaging record described these references as byte-for-byte copies. The manifest retains those historical bytes and hashes; it is not a checksum list for the currently edited repository files. The old export is not bundled here. Historical notes may contain superseded settings or links to local images, logs, and remote-device paths.

| Original repository path | Bytes | SHA-256 of copied content |
| --- | ---: | --- |
| `README.md` | 12773 | `ab67ada6cad30740d9b5a56a948cab6cd2283eab8d92301e6f937a1f4bfc41cf` |
| `AGENTS.md` | 15943 | `0e52479946de065d455b835c4455aaae905a894a4702bca99c1b86ad4f91ad28` |
| `docs/ASSISTIVE_COMMUNICATION_PLAN.md` | 60145 | `97687a4376ab7923aa1584a5ab98b8268cb716793b1c4119dfa36c88e5eedc38` |
| `docs/CABLE_AND_CAMERA_SUPPORT.md` | 2216 | `ff6f9e17934c0b4e83d22d7a0971a678aff892c77172e39cc7044bb27e508af4` |
| `docs/CAMERA_VISUAL_SETUP.md` | 6422 | `0e4a6eaf68baee78d26159a1f3a7797ca88c745ca32ed0986938321c8bc67ff7` |
| `docs/DEVELOPMENT_LOG.md` | 277851 | `17c7c1a646958146e90ae051db1c4fcbcaf16382248630341d42380c4c8e01a3` |
| `docs/EXISTING_EV3_SERVER.md` | 2415 | `b48a7673c14e19a0c95ad66082152484c66e54169061e99fb14839b36f51665a` |
| `docs/FAMILY_CLOTHING_MEMORY.md` | 24154 | `9c6c67f7d22230210c3973f5721964af64c39291b7ad984ad0c0d5f628ac05eb` |
| `docs/GEMINI_NAVIGATION.md` | 5380 | `e018a2b32a4348313d86b07f7e0d36ffc783af5f2bfb44d0eb9be11127e737bf` |
| `docs/HARDWARE_NOTES.md` | 16924 | `112d75facd0fd2eee47d025e36f7210dd24c3052198110e551d627bbdb1eee6f` |
| `docs/PHASE6_GEMINI_DECISIONS.md` | 5808 | `9b861cddfb68a8a1a5192d658acdade8fb58c830b795177b94d8234dfea50422` |
| `docs/PHASE6_PERSON_SEARCH_SCENARIOS.md` | 6158 | `1cf778c3fc6c86d59a0fd14cb5a09adff0c820060b9db3c8215d7daa85729523` |
| `docs/PHONE_ENROLLMENT.md` | 3482 | `2edb49b07a45d5430c5bdff49c98126716bc575d7d86372c00d9a751fab81641` |
| `docs/calibration/README.md` | 1221 | `990e124673feed7896e9975f7ea16c540c8710841892aefd7ef34c6bc86a2137` |
| `robot/ev3/server/README.md` | 4572 | `6f4a51f6ee18cc70cddb3cc0d328e94802b73677e42e21468124d512da1b47ed` |
| `robot/jetson/camera/README.md` | 8540 | `f77d3f5f36d8dcb2aaefb8c3b78762bd143923994727f4abfe5c605897940c52` |
| `robot/jetson/ev3_bridge/README.md` | 6652 | `aaa9fbd453df7e7f27b1e8dd0f355d0fb1ed517da11ec400385620b58715b9a9` |
| `robot/jetson/navigation/README.md` | 2745 | `f803e40f870a6cc2dae3795baae9a29c4568346733fa8251f15b1a94eb04013b` |
| `robot/jetson/perception/README.md` | 20774 | `8b1b24f546b67d2b4d44c79ea7caeff14bfb385c1f0728fb8bbfe198130cc2aa` |
| `robot/jetson/perception/tools/README.md` | 4535 | `3bfc2866d04245758d4702d4619c4302f835e70b94d21a805880d0f5f16deb5a` |
| `robot/mac/README.md` | 5531 | `1f7f012ee01c693d22c62de451c7efcc98d0a8859d097571dc8a27da83281b5f` |
| `scripts/phase6/README.md` | 8495 | `ede7d49ed254c69c0522105d5729394d1a383d7614b57a22069fdb7ce1db4ab9` |
| `communication/README.md` | 35951 | `39b856076e84afbac3879bd9138caa17e081ed0a6bfcd9e9bd1a7ff78ed0433a` |

[Back to reading guide](README.md)
