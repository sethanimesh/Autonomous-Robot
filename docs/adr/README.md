# Architecture decision records

These records explain consequential choices in the current single-room prototype. They are retrospective: implementation and retained tests support the selected behavior, while unrecorded historical motivations are identified as engineering rationale inferred from present constraints.

| Record | Decision | Main cost |
|---|---|---|
| [0001](0001-recipient-identity-and-continuity.md) | Selected enrolled identity, repeated face evidence and qualified clothing continuity | Appearance ambiguity and limited identity evaluation |
| [0002](0002-shared-camera-and-segmented-motion.md) | One movable camera with stopped inspection and segmented approach | Target-visibility gaps and mission delay |
| [0003](0003-offboard-inference-local-authority.md) | Offboard interpretation with Jetson authority and EV3 local stop supervision | Network latency, result invalidation and partial failure |

See the [project framework](../PROJECT_FRAMEWORK.md) for the problem, alternatives, success criteria and evidence boundaries.
