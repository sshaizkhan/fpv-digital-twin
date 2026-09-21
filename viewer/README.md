# viewer/ — Godot 4 FPV view

Phase 3. Empty until the physics↔SITL loop closes in Phase 2; there is nothing
worth looking at before then.

Planned:

- UDP listener on `sim.net.viewer_pose_port` (9100), consuming the 120 Hz pose
  packet: position, quaternion (**scalar-first `[w,x,y,z]`** on the wire),
  velocity, battery voltage, armed flag, throttle.
- Axis conversion from our NED/FRD to Godot's Y-up frame at the boundary, in one
  place, with a test per axis — see `docs/coordinate_frames.md` §8.
- FPV camera at `camera.position`, pitched up by `camera.tilt`, with
  `camera.fov_horizontal`.
- Ground plane, a grid, and a few gates and poles purely as depth cues.
- Minimal OSD: throttle, pack voltage, armed state.
- Chase-cam toggle for debugging.

Godot 4 is a separate download: https://godotengine.org/download/macos/
(not a Homebrew dependency of the C++ build).
