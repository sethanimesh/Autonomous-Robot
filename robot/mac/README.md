# Mac route-perception offload

`route_perception.py` fetches a live Jetson snapshot, runs the ADE20K
SegFormer-B0 model on Apple MPS, and returns conservative left/straight/right
floor evidence plus one bounded route decision. It listens on port 8091 so the
Jetson remains the mission and motion authority.

The launch agent keeps the service available while this Mac user is logged in.
Its Python environment and model cache live under `~/Library/Caches/Echora` and
are intentionally outside Git. Logs live under `~/Library/Logs/Echora`.

The route endpoint refuses inference unless the Jetson reports live camera
frames both before and after snapshot capture. Model weights and the MPS path
are warmed before the HTTP listener starts, so the first accepted decision
fits the Jetson's one-second freshness limit.
