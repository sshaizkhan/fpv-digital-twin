# fpv-digital-twin

A digital twin of one specific 5" quad: the **real Betaflight firmware** (SITL
target) flying a physics model parameterised from measurements of that quad,
flown with the real radio, seen through an FPV camera.

The point is not to be a nice simulator. The point is that the sim's gyro
response to a given stick input matches Blackbox logs from the real quad closely
enough that rates, tune changes, and later MSP/autonomy code can be tested in
sim before touching hardware.

macOS / Apple Silicon.

## Status: Phase 0 complete

| Phase | What | State |
|---|---|---|
| 0 | Repo scaffold, CMake build, `quad.yaml` schema, README | ✅ done |
| 1 | Physics core standalone, with unit tests | ✅ done |
| 2 | Betaflight SITL builds and runs, loop closed, stable hover | unblocked, not started |
| 3 | Radio input live, Godot viewer, acro flight from FPV | not started |
| 4 | Blackbox replay, sim-vs-real overlay, parameter fitting | not started |

**40 of the parameters in `config/quad.yaml` are estimates, not measurements.**
That is expected at Phase 0, and every one of them is flagged in the file and
listed by `fdt_config_dump`. See [`docs/parameters_to_measure.md`](docs/parameters_to_measure.md)
for what to measure and how.

## Layout

```
physics/     C++17 core. Phase 1 ships the 6DOF integrator, motor,
             battery, aero, ground-contact and IMU models, assembled in
             Multirotor, plus the quad.yaml loader. The SITL / SDL2 /
             viewer I/O follows in later phases.
third_party/ Betaflight submodule (Phase 2), pinned to the FC's firmware tag.
viewer/      Godot 4 FPV view (Phase 3).
tools/       Python: Blackbox parsing, stick replay, fitting (Phase 4).
config/      quad.yaml — every physical parameter, with provenance.
             diff_all.txt / dump_all.txt — from the real FC (not yet supplied).
docs/        Conventions and interface notes. Read coordinate_frames.md first.
```

## Prerequisites

```sh
xcode-select --install
brew install cmake ninja eigen sdl2 git python
```

- **Eigen** and **SDL2** are used from Homebrew when present. `yaml-cpp` and
  `googletest` are fetched and built automatically by CMake, so a clean clone
  builds with nothing else installed.
- **Ninja** is optional — there is a `make` preset that needs only CMake.
- Later phases also need: [Godot 4](https://godotengine.org/download/macos/)
  (Phase 3), [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases)
  (Phase 2), and Docker Desktop *only* as the fallback if the native SITL build
  cannot be made to work (Phase 2).

## Build and test

```sh
cmake --preset default        # Ninja; use --preset make if ninja is not installed
cmake --build --preset default
ctest --preset default
```

Presets: `default` (RelWithDebInfo + Ninja), `debug`, `release`
(`-Werror`), `make` (Unix Makefiles, no Ninja needed). Build trees go in
`build/<preset>/`.

Verified on macOS 15.1 / Apple Silicon with Apple clang 16 and CMake 4.2.3:
**125/125 tests pass, clean at `-Wall -Wextra -Wpedantic -Wconversion -Werror`.**

## Run what exists today

Inspect and validate the quad parameters:

```sh
./build/make/physics/fdt_config_dump config/quad.yaml
```

```
quad: custom 3D-printed 5in X  (schema v1, frames FRD_NED)
firmware: betaflight 4.5.1 target SPEEDYBEEF405V4
mass: 0.7200 kg   inertia diag [0.0018 0.0018 0.0034] kg m^2
motors:
  FL  pos [ 0.0780 -0.0780  0.0000] m  spin CW (from above)
  ...
  betaflight index -> motor: 1=RR 2=FR 3=RL 4=FL [UNVERIFIED]

41 parameter(s) are NOT measured:
  - mass.auw
  ...
  ! motors.betaflight_order is unverified
```

Other flags: `--list-unmeasured` (dotted paths only, for scripting) and
`--strict` (exit non-zero while anything is unmeasured — turn this on in CI once
the real measurements are in, so they cannot quietly regress to guesses).

Fly it headless, with no Betaflight and no viewer:

```sh
./build/make/physics/fdt_sim --profile althold --duration 10 --out /tmp/trace.csv
```

```
profile      : althold
steps        : 80000 at 8000.000 Hz (10.000 s)
wall clock   : 0.386 s  ->  25.9x real time, 4.82 us/step
hover cmd    : 34.603 %
final alt    : 9.971 m
battery      : 16.728 V, 1.792 A, soc 99.617 %
```

The step rate comes from `sim.physics_rate` in the config, which is the single
source of truth; `--rate HZ` overrides it for a one-off. Note the `us/step`
figure above is dominated by the CSV write (~4.4 us/row), not the physics: drop
`--out` and the same run reports **0.34 us/step, ~364x real time** at 8 kHz.

Profiles: `hover` (open loop, sinks slowly as the pack sags — that is correct),
`althold` (a proportional altitude hold on top of it), `freefall`, `takeoff`,
`rollstep`. `--out` writes a 30-column CSV trace: pose, quaternion, body rates,
synthesised gyro and accel, per-motor RPM, thrust, pack voltage/current/SoC and
the ground-contact flag.

## How parameters work

Every number in `config/quad.yaml` is a block that must declare where it came
from:

```yaml
  auw:
    value: 0.720
    units: kg
    measured: false          # false => estimate/placeholder
    source: "estimate: ... KITCHEN SCALE THIS."
```

The loader **rejects** a parameter block that omits `units:` or `measured:`.
That is enforced by the test suite, not just by convention, so an estimate can
never quietly pass itself off as a measurement. `QuadConfig::unmeasured` carries
the dotted path of every unmeasured parameter through to anything that wants to
report on it.

The unit tests load the *shipped* `config/quad.yaml`, so the suite doubles as a
schema check on the real file.

## Conventions

Read [`docs/coordinate_frames.md`](docs/coordinate_frames.md) before writing any
code that touches a sign or an axis, and
[`docs/physics_model.md`](docs/physics_model.md) for what the model actually
does and which assumptions are load-bearing. Summary:

- World **NED**, body **FRD** (x forward, y right, z down), origin at the CG.
- Attitude as a quaternion `q_wb` (body → world), Hamilton, scalar-first on the
  wire.
- Positive `p` = roll right, positive `q` = pitch nose up, positive `r` = yaw
  right.
- **SI everywhere** inside the core. The only degrees in `quad.yaml` are the
  camera angles, converted to radians at load.
- Prop spin direction is stated **as seen from above**.

Conversions happen only at boundaries — the SITL packet layer and the Godot
viewer — each in one function with a test per axis.

Docs mark every claim **DEFINED** (our choice), **VERIFIED** (checked against a
cited source), or **UNVERIFIED** (assumed, do not build on it yet).
[`docs/sitl_interface.md`](docs/sitl_interface.md) is currently all UNVERIFIED:
it is the Phase 2 work list of things that must be read out of the Betaflight
source, not a specification.

## What is needed from the pilot

The FC config is in: `config/diff_all.txt` and `config/dump_all.txt` from a real
**Betaflight 4.5.1** / `SPEEDYBEEF405V4`. What they settle — mixer, ESC
protocol, filters, tune, rates, modes, battery, logging — is written up in
[`docs/fc_config.md`](docs/fc_config.md). Re-capture both after any tune change.

Still needed, in rough order of how much each one distorts the gyro match:
all-up weight on a kitchen scale, the inertia tensor from CAD, and the motor
positions. Full list and method in
[`docs/parameters_to_measure.md`](docs/parameters_to_measure.md).

One easy win: `dshot_bidir` is **on**, so RPM is already in your Blackbox logs.
That means `load_factor`, `thrust_coeff` and `time_constant` can be fitted from
a real flight in Phase 4 without a thrust stand.
