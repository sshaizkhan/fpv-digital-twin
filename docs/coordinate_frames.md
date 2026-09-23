# Coordinate frames, units, and sign conventions

**This file is the single source of truth for conventions in this project.** Every
other component converts *explicitly* at its boundary. If you are about to write a
sign or an axis swap anywhere, it belongs in a documented boundary conversion with a
test, not scattered through the code.

Status legend used throughout the docs:

- **DEFINED** — our choice, binding on our own code.
- **VERIFIED** — checked against external source; the check is cited.
- **UNVERIFIED** — assumed/asserted, must be checked before code depends on it.

---

## 1. World frame — NED (DEFINED)

Right-handed, fixed to the ground at the arming point.

| Axis | Direction |
|------|-----------|
| `x`  | North (arbitrary; the sim's "forward" at spawn) |
| `y`  | East |
| `z`  | **Down** |

Gravity is `g_world = [0, 0, +9.80665]` m/s². Altitude above ground is `-z`.

Ground plane is `z = 0`. A quad resting on the ground has `z ≈ -(landing gear height)`.

## 2. Body frame — FRD (DEFINED)

Right-handed, origin **at the CG**, fixed to the airframe.

| Axis | Direction |
|------|-----------|
| `x`  | Forward (out the nose, the direction the FPV camera looks at 0° tilt) |
| `y`  | Right |
| `z`  | **Down** |

All inertia, motor positions, and IMU quantities in `config/quad.yaml` are expressed
in this frame, relative to the CG, in **metres**.

## 3. Attitude (DEFINED)

Stored as a unit quaternion `q_wb` that **rotates a body-frame vector into the world
frame**: `v_world = q_wb * v_body`.

- Hamilton convention (not JPL).
- `Eigen::Quaterniond` internally. Eigen's constructor is `(w, x, y, z)`; its
  `.coeffs()` storage order is `(x, y, z, w)` — **never** memcpy a quaternion across
  a boundary, always name the components.
- On the wire (UDP to the viewer, and in logs) quaternions are serialised
  **scalar-first: `[w, x, y, z]`**.

Euler angles, when used (OSD, logs, tests only — never in the integrator), are
intrinsic **Z-Y-X yaw-pitch-roll**: `R_wb = Rz(ψ) · Ry(θ) · Rx(φ)`.
Positive `φ` = right wing down, positive `θ` = nose up, positive `ψ` = nose right.

## 4. Angular rates (DEFINED)

Body-frame angular velocity `ω_b = [p, q, r]` rad/s, right-handed about the body axes
above. The resulting senses are:

| Rate | Axis | Positive means |
|------|------|----------------|
| `p`  | body `+x` | roll **right** (right wing down) |
| `q`  | body `+y` | pitch **nose up** |
| `r`  | body `+z` | yaw **right** (nose right, CW seen from above) |

Derivation for `q`, since the FRD down-axis trips people up: `ω = ŷ`, nose point
`r = x̂`, velocity `ω × r = ŷ × x̂ = -ẑ`, and `-z` is up. Nose up. ✅

This matches the intuitive stick sense (right stick right → `p > 0`). The SITL
gyro packet takes these rates as-is; Betaflight itself is FLU (+Y nose down,
+Z yaw left) and SITL does that conversion. See [`sitl_interface.md`](sitl_interface.md) 5c-5d.

## 5. Motors (DEFINED, except ordering)

Motors are named `FL`, `FR`, `RL`, `RR` (front-left, front-right, rear-left,
rear-right, as seen from above with the nose pointing away from you). Positions are
vectors from the **CG** in the body frame.

**Spin direction is defined as seen from ABOVE the quad** (looking down, i.e. looking
along body `+z`). Because `+z` points away from that viewer, a prop that looks
clockwise from above has rotor angular velocity `ω_r = +|ω| ẑ_body`.

Forces and torques on the airframe from motor `i` with rotor speed `ω_i` (rad/s):

```
thrust        F_i   = [0, 0, -kT·ω_i²]              (up is -z)
torque arm    τ_i   = r_i × F_i
yaw reaction  τ_z,i = -s_i · kQ · ω_i²               (aero drag reaction)
rotor inertia τ_z,i = -s_i · J_rotor · ω̇_i          (spin-up reaction)
```

where `s_i = +1` for a prop that is **CW seen from above**, `s_i = -1` for CCW.

Sanity check of the sign of `τ_i`: the `FR` motor is at `r = [+d, +d, 0]`, thrust
`F = [0,0,-T]`, so `r × F = [-dT, +dT, 0]` → negative roll (rolls **left**, away from
the motor that is pushing up) and positive pitch (**nose up**). Correct. ✅

**Betaflight's motor index → physical position mapping, and the stock spin
directions, are UNVERIFIED.** They must be read out of the Betaflight source and the
user's `config/diff_all.txt` in Phase 2 and recorded in `docs/sitl_interface.md`.
`config/quad.yaml` therefore keeps our physical `FL/FR/RL/RR` naming separate from
the Betaflight index mapping, and flags the mapping `verified: false`.

## 6. Units (DEFINED)

**SI everywhere inside the physics core**: kg, m, s, rad, rad/s, N, N·m, V, A,
kg·m². No exceptions, no mm, no degrees, no grams.

`config/quad.yaml` is also SI, with one deliberate exception: angles that a human
measures with a protractor (camera tilt, camera FOV) are given in **degrees** with
`units: deg`, and the loader converts them to radians at parse time. Every value in
the YAML carries an explicit `units:` field so a mismatch is visible in review.

## 7. Boundary: physics ↔ Betaflight SITL (UNVERIFIED — Phase 2)

Betaflight SITL has its own frame, units, and packet layout. **Nothing about it is
assumed here.** The conversion, and the evidence for it, goes in
[`sitl_interface.md`](sitl_interface.md), which currently contains only the list of
things that must be read out of the Betaflight source.

Required: one conversion function in each direction, and a unit test per axis and
per motor that fails if a sign or an index is swapped.

## 8. Boundary: physics ↔ Godot viewer (DEFINED, Phase 3)

Godot 4 uses a **right-handed Y-up** frame: `+X` right, `+Y` up, `-Z` forward.

Mapping from our NED world / FRD body to Godot:

```
godot.x =  ned.y      (East  → right)
godot.y = -ned.z      (Down  → up)
godot.z = -ned.x      (North → -Z forward)
```

The same mapping applies to the body axes, which means the Godot node's basis is
obtained from `q_wb` by conjugating with that permutation, **not** by feeding the
quaternion components across directly. One conversion function, one test per axis.

## 9. Where the bugs will be

Per the project rules, each of these gets its own test:

1. Motor index ↔ physical position mapping at the SITL boundary.
2. Prop spin direction → yaw torque sign.
3. Gyro sign per axis at the SITL boundary.
4. Accelerometer sign and whether it reports specific force (it should: `+1 g` up
   reading of `[0,0,-9.81]` in FRD when at rest) or acceleration.
5. Quaternion component order at every wire boundary.
6. Godot axis permutation and handedness.
