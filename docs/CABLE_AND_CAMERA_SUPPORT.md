# Cable and camera support acceptance

Unattended Phase 6 chassis motion stays locked until both physical items below
are fitted. Supervised bounded tests still require an explicit visual cable-
neutral/track-clear check before every new mission.

## External cable harness

- Fasten the cable to the chassis near its center with strain relief. The plug
  and camera/Jetson sockets must never carry pulling force.
- Leave one smooth slack loop for the full ±90° scan and keep it outside both
  tracks, gears, and the camera arm at every tested angle.
- Mark the chassis and floor at the untwisted neutral heading. Start each
  mission there and use the explicit cable-neutral confirmation only after a
  visual check.
- Stop physically before ±120°. Software uses ±115° after margin, but the
  harness must remain safe if odometry is several degrees wrong.

## Loaded camera support

- Add a light elastic, spring, or counterweight that carries most of the static
  camera weight while preserving motor leverage through the difficult arc near
  mechanical arc. If assistance blocks the lens, reduces usable travel, or
  makes lifting harder, change the linkage/pivot geometry; do not compensate
  with repeated stalled motor commands.
- Route the camera wire as a separate loose service loop. It must not act as
  the counterbalance or pull the lens when the head moves.
- With power off, move the linkage through its range by hand. It must not drop,
  snap through the dead point, bind, touch a track, or place the support in
  front of the lens at any required floor/person/face angle.
- Re-run the boot sweep with a hand ready to support the camera. Acceptance
  requires three distinct stable floor/person/face views, at least 30 encoder
  counts of useful span, a successful lower-then-lift check, and unchanged
  track encoders.

The optical and loaded-motion checks now pass at floor 0, person -27, face -42,
and maximum height -54. A supervised cable-bounded search also found the target
at +24.46° and stopped. The camera USB lead nevertheless disconnected during
movement and repeatedly re-enumerated, so physical lead strain relief or
replacement remains the outstanding unattended-operation gate.
