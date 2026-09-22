# What to measure, and how

`config/quad.yaml` currently ships **40 unmeasured parameters**. They are all
plausible for a 5" quad, and all of them are guesses. Run:

```sh
./build/make/physics/fdt_config_dump config/quad.yaml
```

to see the live list at any time (`--list-unmeasured` for just the paths).

The table below is ordered by how much each one distorts the thing the project
is actually trying to match: the gyro response to a stick input.

---

## Blocking — Phase 2 cannot start without these

| YAML path | What | How |
|---|---|---|
| `firmware.betaflight_version` | Firmware version on the real FC | Configurator welcome screen, or `version` in the CLI. The SITL submodule gets pinned to this tag. |
| `config/diff_all.txt` | `diff all` from the real FC | Connect the Configurator, CLI tab, `diff all`, save the output verbatim to `config/diff_all.txt`. |
| `config/dump_all.txt` | `dump all` from the real FC | Same, `dump all` → `config/dump_all.txt`. |

## Tier 1 — dominates the match; measure before believing any overlay

| YAML path | What | How |
|---|---|---|
| `mass.auw` | All-up weight, flight-ready | Kitchen scale, with the flight battery, props, and everything you actually fly with. Grams → kg. |
| `mass.inertia_diag` | Ixx, Iyy, Izz about the CG | From CAD: assign the real densities, add point masses for motors (~47 g each), stack, battery, camera, VTX. Export the inertia tensor about the CG in the body frame. This is the single biggest driver of angular acceleration, so a CAD number beats a guess by a lot. |
| `motors.geometry.*` | Motor XY positions from the CG | Measure motor-shaft to motor-shaft diagonals with calipers, halve them, and offset by the CG. Currently assumed as a symmetric 220 mm diagonal. Enter in **metres**, body FRD (+x fwd, +y right). |
| `motors.model.thrust_coeff` | kT, thrust = kT·ω² | Thrust stand, or fit it in Phase 4 from a hover: at a steady hover, 4·kT·ω² = m·g, and the RPM is in the Blackbox log if bidirectional DSHOT telemetry is on. |
| `motors.model.time_constant` | Motor spin-up lag | Fit in Phase 4 from a step input in a real log, or a thrust stand with a step command. This sets how much of your D-term behaviour the sim can reproduce. |
| `motors.model.torque_coeff` | kQ, drag torque = kQ·ω² | Sets yaw authority almost entirely. Thrust stand with a torque cell, or fit against a real yaw step. |
| `motors.model.load_factor` | Loaded RPM at full throttle, as a fraction of the no-load `kv·V` | A prop is a load, so the motor never reaches `kv·V`. Read RPM straight off a Blackbox log with bidirectional DSHOT telemetry on, at full throttle, and divide by `kv · V_pack`. Or a thrust stand with a tachometer. Sets your top speed and your full-throttle thrust. |

## Tier 2 — visible in the overlay, fit in Phase 4

| YAML path | What | How |
|---|---|---|
| `mass.cg_offset` | CG relative to the motor-square centre | Balance the quad on a straight edge in both axes. Mostly matters if the battery sits well off-centre. |
| `aero.quadratic_drag` | Body drag | Fit from a real log: forward flight at a known attitude and a steady speed pins it. |
| `aero.induced_drag_k` | Thrust-dependent in-plane drag | Fit in Phase 4. This is why a real quad sinks in fast forward flight. |
| `aero.angular_damping` | Aerodynamic rate damping | Fit from the tail of a real rate step. |
| `battery.internal_resistance` | Pack DC IR | Fit from the sag in a real log: `V_sag / I` at a punch-out. Or a pack IR meter. |
| `battery.ocv_curve` | Cell OCV vs SoC | Generic LiPo curve for now; a discharge test on the actual pack if the sag behaviour matters. |
| `imu.sample_rate` | Gyro rate | Read off `diff_all.txt` — must match the real `gyro_hardware_lpf` / loop rate or the noise and delay will not match. |
| `imu.gyro_noise_density`, `imu.accel_noise_density` | IMU noise floor | Fit from a **stationary, armed, motors-off** Blackbox log: take the PSD of the gyro trace. |

## Tier 3 — cosmetic or sim-only, but fix them before you judge the feel

| YAML path | What | How |
|---|---|---|
| `camera.tilt` | Camera up-tilt from the body x axis | Protractor, or a phone level against the camera's front face, quad sat level. Degrees. |
| `camera.fov_horizontal` | Camera horizontal FOV | Camera spec sheet. Degrees. |
| `camera.position` | Lens position from the CG | Calipers. Affects the apparent rotation centre in the FPV view, which is a real part of how a quad feels. |
| `ground.stand_height` | CG height above ground when parked | Sit the quad on a table and measure up to the CG. |
| `motors.model.resistance` | Motor phase resistance | Milliohm meter, or leave for the fitter. |
| `motors.spin` + `motors.betaflight_order` | Prop directions and BF motor numbering | Configurator motor tab: spin each motor one at a time and write down which one moves and which way. Also check `yaw_motors_reversed` in `diff_all.txt`. Set `verified: true` once done. |

## Not parameters

`ground.*` stiffness/damping/friction, `sim.*` rates, `motors.model.max_rpm_safety`,
`battery.initial_soc` and `imu.vibration_enabled` are simulator settings, not
properties of the quad. They are flagged `measured: false` because they are not
measurements — do not go looking for a tape measure for them.

## Marking something as measured

Edit the parameter block in `config/quad.yaml`:

```yaml
  auw:
    value: 0.687          # <- your number, in SI units
    units: kg
    measured: true        # <- flip this
    source: "kitchen scale, flight-ready with 4S 1300, 2026-09-21"
```

Then re-run `fdt_config_dump` and watch the list get shorter. The unit tests
load this same file, so a typo that breaks the schema fails the build, not the
first flight.
