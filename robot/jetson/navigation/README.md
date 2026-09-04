# Camera-only local navigation

Phase 5 uses a rolling local view rather than a persistent map. Perception must
prove a corridor at least 30 cm wide: the measured 20 cm chassis plus a 5 cm
margin on both sides. The planner then emits only one turn-and-drive primitive,
initially 5–10 cm, before the robot stops and observes again.

The route selector accepts clearance, model confidence, and known-pixel
coverage for candidate headings. Low confidence, too much unknown image area,
or insufficient clearance produces `blocked`; unknown never means free.

Because a long external cable is attached, the chassis scan pattern is bounded:
0° to +180°, unwind to 0°, 0° to -180°, and unwind to 0°. It covers the room
without accumulating cable twist.

Depth backends will be compared on real Echora frames before one is selected:

- UniDepth V2 Small for metric depth and uncertainty.
- YOLO26 nano depth for a fast current deployment path.
- Depth Anything V2 Small metric for the mature TensorRT/Core ML fallback.

The Mac may run the heavier backend. A stale result is rejected. Groq or another
LLM may describe a scene but does not produce motor commands or safety clearance.

Raw monocular metres are not trusted directly. `ground_plane.py` uses the known
15 cm lens height, calibrated focal length, and the visible flat floor to fit
camera pitch plus a per-frame relative-depth scale. A poor plane fit is rejected.
Pixels substantially closer than the fitted plane are obstacle candidates;
farther or invalid pixels remain unknown rather than being declared clear.
