# Betaflight SITL interface

**Status: NOTHING HERE IS VERIFIED YET. This is the Phase 2 work list, not a
specification.** Every row below must be filled in by reading the Betaflight
source at the tag pinned in `third_party/betaflight`, with a file-and-line
citation, before any code depends on it. Ports and layouts have changed between
Betaflight releases, so a number found in a blog post or in another simulator is
not evidence.

Where to read (paths are as of recent Betaflight; confirm at the pinned tag):

- `src/main/target/SITL/` — the SITL target: `sitl.c`, `target.h`, `target.mk`
- `src/main/drivers/serial_tcp.c` — the TCP serial link the Configurator uses
- The `dyad` event loop and the UDP helpers SITL uses for its packet links

## 1. Firmware pin

| Item | Value | Verified |
|------|-------|----------|
| FC firmware version (from Configurator / CLI `version`) | **UNKNOWN — user must supply** | ❌ |
| Submodule tag in `third_party/betaflight` | not yet added | ❌ |
| Target name | `SPEEDYBEEF405V4` | ❌ |

`config/quad.yaml: firmware.betaflight_version` is `null` until this is known.
Phase 2 cannot start without it — SITL must match the firmware the real quad
flies, or the tune under test is not the tune on the quad.

## 2. Transport and ports

| Link | Direction | Proto | Port | Verified |
|------|-----------|-------|------|----------|
| FDM / state | physics → SITL | UDP | ? (placeholder 9003) | ❌ |
| Motor / servo outputs | SITL → physics | UDP | ? (placeholder 9002) | ❌ |
| RC channels | physics → SITL | UDP | ? (placeholder 9004) | ❌ |
| Configurator (MSP over TCP serial) | Configurator → SITL | TCP | ? (placeholder 5761) | ❌ |

To verify: find the port constants in the SITL target source; confirm which side
binds and which side connects; confirm whether the RC channels ride in their own
packet or inside the FDM packet.

## 3. Packet structs

Copy the struct definitions **verbatim** from the Betaflight source into
`physics/include/fdt/sitl_packets.hpp`, with the source path and line noted, then
`static_assert` the sizes. Do not re-type them from a table.

For each packet record:

- [ ] exact field order and C types
- [ ] `sizeof` and any padding / packing attribute
- [ ] endianness on the wire
- [ ] a `timestamp` field, if any: units (s? µs?) and epoch
- [ ] whether the packet is sent every step or on request

## 4. Semantics that must be checked field by field

These are the ones that silently produce a plausible-but-wrong sim:

- [ ] **Angular rate units** — rad/s or deg/s?
- [ ] **Angular rate signs** — per axis, versus our FRD convention
      (docs/coordinate_frames.md §4). Test each axis separately.
- [ ] **Accelerometer** — specific force or acceleration? In g or m/s²? Sign of
      the down axis at rest?
- [ ] **Attitude** — does SITL want a quaternion, a rotation matrix, or Euler
      angles? Which order, which handedness, scalar-first or scalar-last?
- [ ] **Position / velocity frame** — NED, ENU, or something local? Metres?
- [ ] **Motor output range** — `[0, 1]`, `[-1, 1]`, or raw DSHOT? Does it
      already have the idle offset / `motor_output_limit` applied?
- [ ] **Motor index → physical position**, and whether SITL applies the mixer
      ordering or expects us to. This is the mapping flagged
      `verified: false` in `config/quad.yaml: motors.betaflight_order`.
- [ ] **RC channel order and range** — AETR vs TAER, 1000–2000 vs something else.
- [ ] **Time** — does SITL free-run, or does it step when we send a packet? This
      decides whether the project's "faster than real time, deterministic
      replay" requirement is achievable, and how.

## 5. Tests to write alongside (Phase 2)

Per the project working rules, each of these gets a test that fails on a sign or
index swap:

1. Round-trip pack/unpack of each struct against a captured golden byte buffer.
2. One test per gyro axis: spin the model about one body axis, assert the sign
   and magnitude of the value SITL receives.
3. Accelerometer at rest reads `+1 g` on the correct axis with the correct sign.
4. One test per motor: command motor `N` only, assert the expected physical
   motor produces thrust and the expected roll/pitch/yaw torque signs follow.
5. RC channel mapping: full-right roll stick lands in the channel Betaflight
   reads as roll, at the right end of the range.

## 6. macOS build

Preferred: native build of the SITL target on Apple Silicon.

- [ ] `make TARGET=SITL` (or the current equivalent) at the pinned tag
- [ ] record every patch needed, and why
- [ ] if native fails after reasonable patching, add `docker/Dockerfile.sitl`
      with the UDP ports and TCP 5761 mapped, and note the loopback/latency
      implications of the Docker network for the Phase 3 latency measurement
