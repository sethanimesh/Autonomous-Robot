# Camera-only local navigation

Phase 5 uses a rolling local view rather than a persistent map. Perception must
prove a corridor at least 30 cm wide: the measured 20 cm chassis plus a 5 cm
margin on both sides. The planner then emits only one turn-and-drive primitive,
initially 5–10 cm, before the robot stops and observes again.

The route selector accepts clearance, model confidence, and known-pixel
coverage for candidate headings. Low confidence, too much unknown image area,
or insufficient clearance produces `blocked`; unknown never means free.

Because a long external cable is attached, a full chassis scan is bounded to
±90° around a physically marked neutral pose. An independent absolute guard
rejects any planned or measured heading outside ±115° (the ±120° envelope minus
a 5° margin), and each scan returns to its origin. The main mission refuses to
move the chassis until the operator explicitly confirms the neutral pose.

Depth backends will be compared on real robot frames before one is selected:

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

The first real obstacle gate also uses semantic segmentation on the Mac's MPS
device, exposed to the Jetson as a small HTTP service on port 8091. It checks
the live-camera health endpoint before and after acquiring every frame and
warms the model before accepting requests. Three broad,
overlapping perspective corridors represent left, straight, and right routes.
A corridor must be at least 95% traversable floor before it receives any clear
distance. In the first slipper frame, SegFormer-B0 labelled the slipper mostly
as non-floor while retaining the surrounding floor; the left corridor was
97.6% floor, straight 86.5%, and right 80.4%, so the deterministic planner chose
the left detour. A closed-loop Jetson runner turns only after a fresh route,
stops, obtains a different post-turn frame, and drives no more than 10 cm only
if the new straight corridor is at least 95% floor. These image trapezoids are
provisional until a measured floor homography replaces them.
