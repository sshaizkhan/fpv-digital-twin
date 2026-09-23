# viewer/ — 3D view of the simulator

Two viewers, for two different jobs.

## `web/` — browser viewer (built)

For **watching**: arm/disarm, take-off, attitude, motor RPM, battery. Runs
anywhere, needs no Godot install.

```sh
./tools/pose_bridge.py --serve          # terminal 1
./build/make/physics/fdt_sim --profile althold --duration 60 --viewer   # terminal 2
open http://127.0.0.1:9102/
```

Or with the real firmware in the loop:

```sh
./tools/run_sitl.sh &
./tools/load_config_sitl.py --restart-container fdt-sitl
./tools/pose_bridge.py --serve &
./build/make/physics/fdt_sitl_hover --duration 25
```

Controls: drag to orbit, scroll to zoom, `c` chase cam, `f` FPV cam, `r` reset.

### Why there is a bridge

**A browser cannot open a UDP socket.** The physics publishes pose over UDP
(`sim.net.viewer_pose_port`, 9100), so `tools/pose_bridge.py` decodes it and
re-emits it as JSON over a WebSocket. The bridge is a dumb pipe holding no
state, so it can be restarted at any time; it validates the packet's magic and
version so a stray packet from one of SITL's nearby ports is rejected rather
than rendered.

### Frames

The wire format is world NED with a scalar-first quaternion, exactly as
`docs/coordinate_frames.md` defines. Three.js is Y-up right-handed — the same
convention as Godot — so the viewer applies the mapping from section 8 of that
document:

```
three.x =  ned.y      three.y = -ned.z      three.z = -ned.x
```

The quaternion is transformed by **conjugation** with that rotation, not by
shuffling its components. Shuffling would silently mirror the model, which
looks almost right and is very hard to spot.

## Godot — still the plan for flying

The browser viewer is a spectator. For actually **flying FPV with the radio**,
Phase 3's acceptance criterion is measured end-to-end input latency at a steady
frame rate, and a browser tab's compositor makes that hard to control or
measure honestly. Godot stays the plan for that.

Nothing here blocks it: the radio path is
`controller -> SDL2 -> physics -> SITL -> motors -> physics`, entirely inside
the C++ process. Both viewers are read-only consumers of the same UDP pose
feed, so a Godot viewer can be added later without touching any of it — or run
alongside this one.

Godot 4 is a separate download: https://godotengine.org/download/macos/
