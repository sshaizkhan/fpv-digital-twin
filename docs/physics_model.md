# The physics model

What Phase 1 actually implements, and — more importantly — the modelling
assumptions baked into it. Every one of these is a place where the sim can
disagree with the real quad, so each says how it would be found out and fixed.

Frames, signs and units throughout: [`coordinate_frames.md`](coordinate_frames.md).

## Rigid body (`physics/src/rigid_body.cpp`)

State is 13 numbers: position and velocity in world NED, attitude as a
Hamilton quaternion `q_wb`, and body rates.

```
dp/dt = v                                   (world)
dv/dt = R_wb * F_body / m + g_world          (world)
dq/dt = 0.5 * q (x) [0, omega]               (body rates, q on the left)
dw/dt = I^-1 * (tau - omega x (I omega))     (body)
```

Classical RK4, fixed step. The quaternion is renormalised once at the end of
the step, never inside the stages, so the stage weights stay exact. Gravity is
added by the integrator in the world frame, which is why every force model
returns a body-frame wrench that excludes it.

The `omega x (I omega)` term is what makes a tumbling body nutate. There is a
test that fails if it is dropped (the rates freeze and angular momentum still
appears conserved, which is why "it looks fine" is not evidence).

A semi-implicit Euler stepper is also provided, unused so far, in case the
ground contact spring ever needs a symplectic integrator.

## Motors (`physics/src/motor.cpp`)

```
omega_target = clamp(command, 0, 1) * kv_rad * load_factor * V_pack
omega(t)     = omega_target + (omega_0 - omega_target) * exp(-t / tau)
thrust_i     = kT * omega_i^2                      along body -z
torque_i     = r_i x thrust_i                      roll and pitch
yaw_i        = -s_i * (kQ * omega_i^2 + J * omega_dot_i)
```

`s_i` is +1 for a prop that turns clockwise seen from above. The second yaw
term is the rotor-inertia reaction — the kick you feel when the props change
speed, as distinct from the steady aerodynamic drag reaction.

The lag is **solved analytically, not integrated**. Over one step the command
and the supply voltage are constant, so the exponential above is exact and can
be evaluated at any sub-step time. That is what lets each RK4 stage see the
true rotor speed at `t`, `t+dt/2` and `t+dt` instead of a value frozen at the
start of the step.

**Assumptions to revisit in Phase 4:**

- *Rotor speed is linear in throttle.* Real ESCs are close to this with DShot,
  but not exactly. Would show up as a hover throttle that matches at one point
  and drifts at others. Fix: fit an exponent, or a lookup.
- *`load_factor` is a single constant.* The loaded/unloaded RPM ratio really
  varies with airspeed and throttle. Without it the model spins to ~28500 rpm
  at full throttle, which contradicts the ~21400 rpm `thrust_coeff` was
  derived at — so a constant is much better than nothing, and is the first
  thing a thrust stand or a DShot-telemetry log should replace.
- *No prop unloading in fast forward or descending flight.* A real prop makes
  less thrust in a climb and more in a descent. Partly absorbed by the induced
  drag term; would show up as altitude errors in fast vertical manoeuvres.
- *Current from a steady-state power balance*, `V*I = kQ*omega^3 + I^2*R`,
  solved per motor for the low-current root. Ignores ESC efficiency and
  switching losses, so the absolute amps are optimistic even if the shape is
  right.

## Battery (`physics/src/battery.cpp`)

Per-cell OCV curve against state of charge, linearly interpolated and clamped,
times the cell count, minus `I * R_internal`.

The sag is computed with the **previous** step's current rather than solved
simultaneously with the motors. At 1–2 kHz that lag is orders of magnitude
below anything measurable, and it avoids an implicit solve in the hot loop.

State of charge is a straight coulomb count, clamped to `[0, 1]`. No
temperature, no recovery, no cell imbalance.

Visible consequence, and it is the correct one: at a fixed throttle the sim
slowly sinks as the pack droops. There is a test asserting exactly that.

## Aerodynamics (`physics/src/aero.cpp`)

```
F_body = -(linear .* v) - (quadratic .* v .* |v|) - k_induced * T * v_inplane
tau    = -(angular_damping .* omega)
```

Drag is applied **per body axis** (`v_i * |v_i|`, not `v_i * |v|`) so the three
axes stay independent and each can be fitted separately against a real log.
The induced term scales with total thrust because it is the rotor disc, not
the frame, that produces it — this is why a real quad sinks in fast forward
flight.

**Not modelled:** wind, ground effect, prop wash over the frame, the drag
asymmetry between an upright and an inverted quad, or any Reynolds-number
dependence. No blade-element theory: the whole rotor is `kT` and `kQ`.

## Ground contact (`physics/src/ground.cpp`)

Four penalty contact points, one under each motor at `stand_height` below the
CG. Each is a spring-damper in the surface normal, clamped so it can only
push, plus Coulomb friction bounded by `mu * F_normal` and softened near zero
tangential speed (a fixed 0.05 m/s regulariser, numerical rather than
physical, which is why it is not in `quad.yaml`).

Four points rather than one is what lets a parked quad sit level and a landing
on one arm tip the way a real one does.

This is **tuned for stability, not realism**. With the shipped numbers the
contact resonance is ~34 Hz against a 1–2 kHz step, damping ratio ~0.8. Do not
read anything into a crash the sim produces.

## IMU (`physics/src/imu.cpp`)

The accelerometer reports **specific force**: every force except gravity, per
unit mass, in the body frame. Level at rest it reads `[0, 0, -9.80665]`; in
free fall it reads zero. Since the wrench handed to the integrator already
excludes gravity, this is simply `F_body / m`.

Gaussian white noise from the configured densities (`sigma = density /
sqrt(dt)`), plus a bias that random-walks. Drawn from a seeded `mt19937_64`,
so a run is reproducible bit for bit — Phase 4's log replay depends on that,
and there is a test pinning it.

**Not modelled:** the gyro/accel lever arm from the CG (the IMU is assumed at
the CG — worth adding once `camera.position`-style offsets are measured),
scale-factor and cross-axis errors, temperature drift, and frame vibration.
Vibration is the significant omission for matching real Blackbox noise floors;
`imu.vibration_enabled` is the hook for it.

## Assembly and determinism (`physics/src/multirotor.cpp`)

Per step: hold the pack voltage, RK4 the rigid body (with the motor lag
sampled analytically at each stage), commit the motor speeds, charge the pack
for the step's current, then refresh telemetry and draw one IMU sample.

Given the same config, seed, initial state and command sequence, `step()` is
bit-identical. Measured throughput is **~0.33 us/step, about 1500x real time**
at 2 kHz on an M-series Mac, so the "deterministic and faster than real time"
requirement for replay has a lot of headroom.

## What Phase 1 deliberately leaves out

No Betaflight, no SITL, no radio, no viewer. Motor commands come from test
code or from `fdt_sim`. The mixer lives in Betaflight, so there is no mixer
here — the model takes four throttles and nothing else.
