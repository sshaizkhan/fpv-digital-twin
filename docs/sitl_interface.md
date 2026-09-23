# Betaflight SITL interface

**Everything below was read out of the Betaflight source at tag `4.5.1`**,
commit `77d01ba3b76a22909d5f09cb0628820141f95eaa` — which is byte-for-byte the
commit string in the FC's own `diff all` header, so the source being cited is
the firmware the quad actually flies. Every claim carries a file and line.

Status legend: **VERIFIED** (read in source, cited) · **UNVERIFIED** (not yet
checked — do not build on it) · **MEASURED** (confirmed against a running SITL).

Paths are relative to `third_party/betaflight/`.

---

## 1. Firmware pin — VERIFIED

| Item | Value | Evidence |
|---|---|---|
| Firmware | Betaflight 4.5.1 | `config/diff_all.txt` header |
| Commit | `77d01ba3b76a2290…` | `git rev-parse HEAD` in the submodule matches the dump header |
| Target | `SPEEDYBEEF405V4` | `board_name` in `config/dump_all.txt` |

The broader read of the FC config — mixer, ESC protocol, filters, tune, rates,
modes — is in [`fc_config.md`](fc_config.md).

## 2. Transport and ports — VERIFIED

`src/main/target/SITL/sitl.c:80-83`:

```c
#define PORT_PWM_RAW    9001    // Out
#define PORT_PWM        9002    // Out
#define PORT_STATE      9003    // In
#define PORT_RC         9004    // In
```

| Link | Direction | Proto | Port | Who binds | Evidence |
|---|---|---|---|---|---|
| State / FDM | physics → SITL | UDP | 9003 | **SITL binds** (server) | `sitl.c:316` `udpInit(&stateLink, NULL, PORT_STATE, true)` |
| RC channels | physics → SITL | UDP | 9004 | **SITL binds** (server) | `sitl.c:319` `udpInit(&rcLink, NULL, PORT_RC, true)` |
| Motor output | SITL → physics | UDP | 9002 | SITL sends (client) | `sitl.c:310`, sent at `sitl.c:599` |
| Motor raw (RealFlight) | SITL → physics | UDP | 9001 | SITL sends (client) | `sitl.c:313`, sent at `sitl.c:601` |
| Configurator (MSP) | Configurator → SITL | TCP | **5761** | SITL listens | `serial_tcp.c:41` `BASE_PORT 5760`, `:113` `dyad_listenEx(..., BASE_PORT + id + 1, ...)` → UART1 = 5761 |

The placeholder ports in `config/quad.yaml` were right, and `sim.net.sitl_verified`
can now be flipped to `true`.

**RC is a separate packet on its own port**, not part of the FDM packet.

**The destination address for the outbound motor packets is `argv[1]`**
(`sitl.c:26-36`, default `127.0.0.1`), and it is passed straight to
`inet_addr()` (`udplink.c:36`) — **dotted IPv4 only, no hostname resolution.**
This is what the Docker entrypoint has to work around.

## 3. Packet structs — VERIFIED

`src/main/target/SITL/target.h:255-277`. Sizes and offsets below were confirmed
by compiling the structs and printing `sizeof`/`offsetof` (LP64, natural
alignment, no packing attributes — all fields are naturally aligned, so there
is no padding surprise except the 2 bytes after `motorCount`).

### `fdm_packet` — physics → SITL, 144 bytes

| Offset | Field | Type | Units / frame |
|---|---|---|---|
| 0 | `timestamp` | `double` | **seconds** |
| 8 | `imu_angular_velocity_rpy[3]` | `double[3]` | **rad/s** |
| 32 | `imu_linear_acceleration_xyz[3]` | `double[3]` | **m/s², body frame** |
| 56 | `imu_orientation_quat[4]` | `double[4]` | **w, x, y, z — scalar first** |
| 88 | `velocity_xyz[3]` | `double[3]` | m/s, earth frame |
| 112 | `position_xyz[3]` | `double[3]` | m, **NED** from origin |
| 136 | `pressure` | `double` | Pa (fed to the virtual baro, `sitl.c:149`) |

The quaternion is **scalar-first**, matching our own wire convention, and
position is **NED**, matching our world frame. Those two cost us nothing.

### `rc_packet` — physics → SITL, 40 bytes

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `timestamp` | `double` | seconds |
| 8 | `channels[16]` | `uint16_t[16]` | `SIMULATOR_MAX_RC_CHANNELS = 16` (`target.h:238`) |

Channel values are used **raw, as microseconds** — `readRCSITL` returns
`rcPkt.channels[channel]` with no scaling (`sitl.c:228-232`). So 1000–2000,
1500 centre. The debug print at `sitl.c:249-251` labels channels 0-3 as
**AETR** (roll, pitch, throttle, yaw) and 4-7 as AUX1-4.

### `servo_packet` — SITL → physics, 16 bytes

| Offset | Field | Type | Range |
|---|---|---|---|
| 0 | `motor_speed[4]` | `float[4]` | `[0.0, 1.0]` normal, `[-1.0, 1.0]` with FEATURE_3D |

### `servo_packet_raw` — SITL → physics, 68 bytes

`uint16_t motorCount` at 0, `float pwm_output_raw[16]` at **4** (2 bytes of
padding). Raw PWM 1100–1900. This is the RealFlight bridge format; we use
`servo_packet` on 9002 and can ignore 9001.

## 4. Semantics — the parts that silently produce a wrong sim

### 4.1 Motor ordering is PERMUTED — VERIFIED, and this is the big one

`sitl.c:592-595`:

```c
pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;   // BF motor 1 -> slot 3
pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;   // BF motor 2 -> slot 0
pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;   // BF motor 3 -> slot 1
pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;   // BF motor 4 -> slot 2
```

The comment at `sitl.c:585` says this remap exists "for gazebo8
ArduCopterPlugin". **The packet is NOT in Betaflight motor order.** Slot `i` of
`motor_speed` carries Betaflight motor index `(i + 1) mod 4`, i.e.

| packet slot | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| Betaflight motor (1-based) | 2 | 3 | 4 | 1 |

This is on top of, not instead of, Betaflight's own index → physical position
mapping, which is still **UNVERIFIED** (see §7).

### 4.2 Motor scaling — VERIFIED

`sitl.c:557` `motorsPwm[index] = value - idlePulse`, then `/ outScale` where
`outScale = 1000.0` (`sitl.c:587`, or 500.0 with FEATURE_3D at `:588-590`).
`idlePulse` comes from `motorPwmDevInit` (`sitl.c:629-637`). So the value on
the wire is throttle above idle, normalised — **the idle offset is already
subtracted** and we must not apply it again.

### 4.3 Gyro signs — VERIFIED (roll straight through, pitch and yaw negated)

`sitl.c:142-144`:

```c
x = constrain( pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE * RAD2DEG, ...);
y = constrain(-pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE * RAD2DEG, ...);
z = constrain(-pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE * RAD2DEG, ...);
```

`GYRO_SCALE = 16.4` and `RAD2DEG = 180/π` (`sitl.c:105-106`), i.e. the packet
is rad/s and SITL converts to deg/s at the 16.4 LSB/(deg/s) scale of a ±2000
deg/s gyro.

X passes through, Y and Z are flipped. That is exactly an FRD ↔ FLU handedness
flip on two axes, which says the packet is **not** in Betaflight's internal
axis convention. Our FRD body frame is the natural candidate for the packet
side, but **which of our axes ends up where is still UNVERIFIED** — it must be
confirmed empirically per axis (§7).

### 4.4 Accelerometer — VERIFIED as written, semantics UNVERIFIED

`sitl.c:136-138` negates **all three** axes:

```c
x = constrain(-pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE, ...);
y = constrain(-pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE, ...);
z = constrain(-pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE, ...);
```

`ACC_SCALE = 256 / 9.80665` (`sitl.c:105`) — Betaflight's 1 g = 256 counts.

Note this is a **different** sign pattern from the gyro (all three, versus two).
Whether the field is specific force or kinematic acceleration is not stated
anywhere beyond the field name, and the two sign patterns cannot both be a pure
frame rotation. **Do not guess this — settle it empirically** (§7).

### 4.5 Attitude comes from the packet, NOT from the IMU — VERIFIED

`target.h:48` is `#undef USE_IMU_CALC`, so the `#if !defined(USE_IMU_CALC)`
block at `sitl.c:150-179` is live, and `SET_IMU_FROM_EULER` is not defined, so
the active line is `sitl.c:177`:

```c
imuSetAttitudeQuat(pkt->imu_orientation_quat[0], ..., [3]);
```

**Betaflight does not run its attitude estimator under SITL — it takes our
quaternion directly.** Consequences:

- Our quaternion must be correct in Betaflight's own convention, and an error
  there will not be "corrected" by the accelerometer.
- The accelerometer matters much less than it would on real hardware. It is
  still fed to the virtual device, but attitude does not depend on it.
- Attitude-mode (`ANGLE`/`HORIZON`, both configured — see `fc_config.md`) will
  behave off our quaternion.

### 4.6 Timing — VERIFIED

- `timestamp` is seconds, and SITL derives `deltaSim` from successive packets
  (`sitl.c:130`). Negative deltas are dropped (`:131-133`).
- A gap over **500 ms** resets the clock and returns early (`sitl.c:122-128`).
- SITL expects the simulator to **run faster than 50 Hz**: `deltaSim < 0.02`
  gates the rate tracking (`sitl.c:187`).
- **Lock-step: "get one fdm_packet can only send one servo_packet"**
  (`sitl.c:597`). The motor send is gated on `pthread_mutex_trylock(&updateLock)`
  (`:598`), unlocked by `updateState` (`:201`). So the loop is driven by our
  state packets — good for deterministic replay.
- `SIMULATOR_GYROPID_SYNC` / `SIMULATOR_IMU_SYNC` are **commented out**
  (`target.h:50-53`), so SITL free-runs its scheduler rather than stepping per
  packet.

### 4.7 RC is required before anything works — VERIFIED

`sitl.c:240-265`: the RX provider is only installed on the **first** RC packet
(`if (!rc_received)`, `:248`), which sets `rxRuntimeState.rcReadRawFn`,
`channelCount = 16` and `rxProvider = RX_PROVIDER_UDP`. Until then Betaflight
has no receiver at all. **Send RC before expecting to arm.**

## 5a. MEASURED against a running SITL

`fdt_sitl_probe` streams known values into SITL and reads back what Betaflight
actually received via MSP. Reproduce with `./tools/run_sitl.sh` then
`./build/make/physics/fdt_sitl_probe`.

### Accelerometer — MEASURED, matches the source

Sending 9.80665 m/s² on one `imu_linear_acceleration_xyz` axis at a time:

| fdm axis | Betaflight acc[X,Y,Z] | Result |
|---|---|---|
| `xyz[0]` | **-256**, 0, 0 | X → X, **negated** |
| `xyz[1]` | 0, **-256**, 0 | Y → Y, **negated** |
| `xyz[2]` | 0, 0, **-256** | Z → Z, **negated** |

No axis swapping: identity mapping, all three negated, exactly as `sitl.c:136-138`
says. 1 g = 256 counts confirmed (`ACC_SCALE`).

### Attitude quaternion — MEASURED, pitch comes back negated (correctly)

Sending a 30° rotation about one axis and reading MSP_ATTITUDE:

| Sent | roll | pitch | yaw |
|---|---|---|---|
| identity | 0.0 | 0.0 | 0.0 |
| +30° about fdm x | **+30.0** | 0.0 | 0.0 |
| +30° about fdm y | 0.0 | **-30.0** | 0.0 |
| +30° about fdm z | 0.0 | 0.0 | **+30.0** |

Roll and yaw pass straight through; **pitch comes back negated**. That is
Betaflight's convention, not a defect: under SITL, `imuComputeRotationMatrix`
patches `rMat[1][0]` and `rMat[2][0]` (`imu.c:162-165`, the
`SIMULATOR_BUILD && !USE_IMU_CALC && !SET_IMU_FROM_EULER` block), which turns
our NED/FRD quaternion into Betaflight's nose-DOWN-positive pitch. See 5d.

### Gyro — RESOLVED, and now MEASURED

Sending 1 rad/s on one `imu_angular_velocity_rpy` axis at a time:

| fdm axis | Betaflight gyro[X,Y,Z] | Result |
|---|---|---|
| `rpy[0]` | **+939**, 0, 0 | X → X, same sign |
| `rpy[1]` | 0, **-939**, 0 | Y → Y, **negated** |
| `rpy[2]` | 0, 0, **-939** | Z → Z, **negated** |

Identity axis mapping; X passes through, Y and Z are negated — exactly what
`sitl.c:142-144` says. 939 counts per rad/s confirms `GYRO_SCALE * RAD2DEG`
(16.4 × 57.2958). **All three conventions in this section are now measured.**

#### Why it read zero for so long: the realtime tasks never executed

**The gyro was never the problem. The three realtime tasks did not run at all.**

Betaflight's own task table, read over the CLI while SITL was streaming
(`tasks`):

```
Task list             rate/hz  max/us  avg/us maxload avgload  total/ms
02 - (           GYRO)      0       0       0    0.0%    0.0%         0
03 - (         FILTER)      0       0       0    0.0%    0.0%         0
04 - (            PID)      0       0       0    0.0%    0.0%         0
05 - (            ACC)    224     188       0    4.2%    0.0%        26
06 - (       ATTITUDE)     27      45       0    0.1%    0.0%         3
14 - (           BARO)     15      67       1    0.1%    0.0%         7
```

GYRO, FILTER and PID have executed for a total of **0 ms** since boot. Every
other task runs normally. That single fact explains BOTH Phase 2 blockers:

- **No gyro.** `virtualGyroRead` is only called from `gyroUpdateSensor`, inside
  TASK_GYRO. It never runs, so `gyroADCRaw` is never written and the whole
  chain downstream reads zero.
- **No motor packets.** `pwmCompleteMotorUpdate` — which is what sends the
  servo_packet (`sitl.c:582-602`) — is driven by the PID loop. It never runs,
  so SITL has nothing to send. This was previously blamed on
  `motor_pwm_protocol` being DISABLED; that is a real issue too, but it is not
  why the packets were missing.

The accelerometer works because TASK_ACCEL is an ordinary-priority task
scheduled through the normal path, while GYRO/FILTER/PID are all
`TASK_PRIORITY_REALTIME` (`fc/tasks.c:361-363`) and run **only** inside the
`if (gyroEnabled)` block at `scheduler.c:488-533`.

#### The actual cause: a 50 us sleep that costs milliseconds in a VM

`gyroEnabled` was never the problem — it is true. `cliTasks` only prints tasks
where `taskInfo.isEnabled` (`cli.c:4855-4858`), so GYRO/FILTER/PID appearing in
the table proves `fc/tasks.c:500-508` ran, which means `schedulerEnableGyro()`
was called. The block at `scheduler.c:488` **is** entered.

It is the timing test at `scheduler.c:515` that never passes, and the reason is
`src/main/main.c:49-54`:

```c
while (true) {
    scheduler();
#ifdef SIMULATOR_BUILD
    delayMicroseconds_real(50); // max rate 20kHz
#endif
}
```

That 50 us assumes `nanosleep` is accurate to microseconds. On the macOS host
it costs ~64 us, which would be fine. **Inside Docker Desktop's VM it costs
milliseconds**, pinning the loop near 300 Hz — visible in the task table as
TASK_ACCEL achieving 224 Hz against a desired 1000 Hz.

Now the arithmetic. `SCHED_START_LOOP_MIN_US = 1` (`scheduler.h:37`), so
`schedLoopStartCycles` starts at **1** and is capped at 12 (`scheduler.c:359-361`;
for SITL `clockMicrosToCycles` is the identity, `sitl.c:445-448`). The gyro
period is 100 us (`target/SITL/target.h:64-65`). Each pass:

- `schedLoopRemainingCycles = nextTargetCycles - now` is hugely negative,
  because `lastTargetCycles` is only ever advanced at `scheduler.c:588`, inside
  the branch that actually runs the tasks.
- The gross-overrun recovery at `:498-506` pushes the target forward. Working
  it through, the new remaining is `P - (D mod P)`, i.e. somewhere in `(0, 100]`.
- `:515` then needs `remaining < 1` — a **1 us window in every 100 us**.

With ~3 ms between passes and a VM timer that quantises them, that window is
essentially never hit, and because `lastTargetCycles` never advances the state
repeats forever. The tasks are enabled, due, and permanently skipped.

#### The fix

`docker/Dockerfile.sitl` reduces that sleep to 1 us at build time. It is the
only modification to upstream, it is applied inside the image (the submodule
stays pristine), and the build **fails** if the line it patches ever changes
upstream, so it cannot silently stop applying.

Measured with the patch: gyro live and correct, PID `cycleTime` 76-147 us,
container CPU **6.3%** — the 1 us sleep still yields, so this is not a busy
spin.

This is a property of the host timer, not of Betaflight: on a Linux host where
`nanosleep(50us)` is accurate, stock SITL would run its loop near 20 kHz and
hit the window comfortably. It matters here because Docker is the only way to
run SITL on this machine (section 6).

#### Evidence gathered, so the next attempt does not repeat it

| Checked | Result |
|---|---|
| Gyro detected | YES — `status`: "Gyros detected: gyro 1, gyro 2", `GYRO=VIRTUAL`; MSP_STATUS sensors bit 5 set |
| `gyro_to_use` | `FIRST`, and `rawSensorDev` points at `gyroSensor1`, so no device mismatch |
| Both sensors' raw | `DEBUG_DUAL_GYRO_RAW` shows s1 **and** s2 both zero — data is not landing in the other slot |
| Calibration | Complete; and forced complete from boot for `GYRO_VIRTUAL` anyway (`gyro.c:174-182`) |
| `gyroADCRaw` at source | Zero — `DEBUG_GYRO_RAW`, so the break is upstream of all filtering and scaling |
| Betaflight's clock | **Advancing** — `System Uptime` climbs ~8 s per 4 s real. Not frozen. |
| `simRate` starvation | Ruled out: streaming with timestamps taken from real elapsed time changed nothing |
| `SystemCoreClock` | Set to 500 MHz (`sitl.c:292`); the cycle helpers are sane stubs (`sitl.c:430-452`) |
| `gyroSyncCheckUpdate` double-consume | Dead code — never called anywhere in the tree |
| `gyroSetSampleRate` | Returns 8000 Hz via the `default:` branch (`gyro_sync.c:87-91`), not 0 |
| `CPU: 0%` in `status` | **Cosmetic.** Hardcoded for `SIMULATOR_BUILD` (`scheduler.c:207-209`) — not a symptom |

### Superseded: earlier notes on this, kept because they were wrong



Sending 1 rad/s (and 10 rad/s) on one `imu_angular_velocity_rpy` axis at a time
produces `gyro[X,Y,Z] = 0, 0, 0` from MSP_RAW_IMU, while the accelerometer in
the same packets responds correctly.

Ruled out so far:

- **Not calibration — and calibration could never have been the cause.** The
  virtual gyro is calibration-complete from boot: `gyroSetCalibrationCycles`
  forces `calibration.cyclesRemaining = 0` when
  `gyroDev.gyroHardware == GYRO_VIRTUAL` (`gyro.c:174-182`), and
  `isGyroSensorCalibrationComplete` is just `cyclesRemaining == 0`, so
  `performGyroCalibration` is never reached at all.

  An earlier version of this bullet claimed Betaflight "freezes `gyro.gyroADC`
  at zero until calibration completes" and that the calibration samples come
  from our packets. **That was wrong** and it is recorded here because the
  wrong reasoning was load-bearing: it made `ARMING_DISABLED_CALIBRATING` look
  like a gyro gate. It is not one. `isCalibrating` (`fc/core.c:183-195`) ORs the
  gyro, ACC, BARO and MAG states and SITL compiles all four in
  (`target.h:72-82`), but on a fresh boot **only the baro is ever calibrating**:
  `accStartCalibration` runs at boot only for `MIXER_GIMBAL`
  (`init.c:821-824`), the mag calibrates only on a stick command or
  MSP_MAG_CALIBRATION, and the gyro is complete from boot as above — leaving
  `baroStartCalibration` (`init.c:827-829`) as the only one that actually runs.
  The ~0.3 s we measure is the baro settling on our `pressure` field.
  `test_msp_client.cpp` pins these source facts so this cannot silently rot.
- **Not packet framing.** The same packets drive the accelerometer correctly.
- **Not the scale.** `gyroRateDps` (`gyro_init.c:737-740`) divides the filtered
  dps back by `gyroDev.scale`, so MSP_RAW_IMU is in raw counts and 1 rad/s
  should read about 940.
- **Not sample starvation on its own.** `gyroUpdateSensor` returns early when
  `readFn` returns false (`gyro.c:383-386`), leaving `gyroDev.gyroADC` at its
  previous value — and `gyroUpdate` then re-publishes that value to
  `gyro.gyroADC` anyway, gated only on calibration completeness, which is
  always true here (`gyro.c:412-421`). So a starved gyro would report a **stale
  non-zero** reading, not a flat zero. A persistent zero means no non-zero
  sample ever reached `gyroDev.gyroADC` in the first place.

Next diagnostics to try, in order of what the above leaves standing:

1. Whether `TASK_GYRO` / `gyroUpdate` runs at all under SITL's free-running
   scheduler — a task that never fires explains a flat zero exactly.
2. `gyroDev.gyroAlign` and `gyro.gyroToUse`: an alignment or sensor-selection
   mismatch would zero or misroute the axes after the read succeeds.
3. Whether `gyro.gyroADCf` is populated (MSP_DEBUG with
   `debug_mode = GYRO_RAW`), to separate the read path from the filter path.

**Until this is resolved there is no bridge.** The gyro is what Betaflight
actually flies on.

### A timing fact worth keeping

`SIMULATOR_GYROPID_SYNC` is commented out (`target.h:50-53`), so **Betaflight
free-runs its scheduler on real wall-clock time**. Streaming packets faster
than real time does NOT deliver proportionally more sensor samples — later
packets simply overwrite the buffer before the gyro task reads it. An early
version of the probe flooded 30000 packets in under a second and produced
smeared, half-settled accelerometer readings for exactly this reason. Every
sweep is now paced in real time.

## 5c. The bridge

`physics/src/sitl_bridge.cpp` holds every conversion at this boundary and
nothing else does. Derived from the measurements above plus Betaflight's own
conventions (5d):

| Quantity | Conversion | Why |
|---|---|---|
| Gyro | **unchanged** | SITL's Y/Z negation (`sitl.c:142-144`) is the FRD → FLU conversion into Betaflight's body frame. |
| Accelerometer | negate **X** only | Betaflight wants `(fx, -fy, -fz)`; SITL negates all three (`sitl.c:136-138`). At rest `[0,0,-g]` still arrives as +256 on Z. |
| Attitude | **unchanged**, scalar first | The SITL `rMat` patch (`imu.c:162-165`) already yields Betaflight's roll, nose-down pitch and compass heading. |
| Position, velocity | **unchanged** | Both are NED already (`target.h:260-261`). |
| Motors | undo SITL's slot permutation, then `betaflight_order` | Two separate mappings, both must be right. |
| RC | microseconds, AETR then AUX1-4 | `readRCSITL` returns them unscaled (`sitl.c:228-232`). |

Verified end to end by `fdt_sitl_probe`, which drives the real conversions and
reads back over MSP. It exits non-zero if any check drifts, so it is a
regression check rather than a one-off observation. Output: see 5d.

### Correction (after 45a5406)

The first version of the bridge negated gyro pitch and yaw and the quaternion's
y component, and sent the accelerometer unchanged. It assumed Betaflight's pitch
is nose-up positive (from `imu.c:317` alone) and that its gyro frame is FRD.
Both are wrong. The probe passed anyway, because its expected values encoded
the same assumption. The measurements in 5a were right; only their
interpretation was wrong. Under that version pitch and yaw rate feedback would
have been positive, and acc X was inverted.

## 5d. Betaflight's own conventions — READ FROM SOURCE

A measurement only says what SITL does to our numbers. What Betaflight counts
as correct comes from the sign of the setpoint each stick produces, since the
rate PID drives the gyro toward it:

| Stick | `rcCommand` | So Betaflight's |
|---|---|---|
| roll right | positive (`rc.c:698`) | +gyro X = roll right |
| pitch forward (nose down) | positive (`rc.c:698`) | +gyro Y = nose **down** |
| yaw right | **negative** (`rc.c:705`) | +gyro Z = yaw **left** |

That is a right-handed FLU body frame (x forward, y left, z up). The
accelerometer shares it, so a level FC reads +1 g on Z, a nose-down tilt reads
negative X, and right-wing-down reads positive Y.

Attitude: angle mode drives `attitude.raw` toward a target that is positive on
forward stick (`pid.c:387-395`), so **pitch is nose-down positive**. Roll is
right positive, yaw is a 0-360 compass heading.

`fdt_sitl_probe` checks the bridge against these, not against the raw
measurements:

```
OK   attitude pitch, nose up 30 deg                -30.0  want -30.0
OK   gyro X, right roll 2 rad/s                   1879.0  want 1879.3
OK   gyro Y, nose-up pitch 3 rad/s               -2818.0  want -2819.0
OK   gyro Z, yaw right 1 rad/s                    -939.0  want -939.7
OK   combined: roll right 25                        25.0  want 25.0
OK   combined: nose up 15                          -15.0  want -15.0
OK   combined: heading 250                         250.0  want 250.0
OK   acc Z at rest (1 g = 256)                     256.0  want 256.0
OK   acc X, nose down 20 at rest                   -87.0  want -87.6
OK   acc Y, right wing down 20 at rest              82.0  want 82.3
```

## 5b. Still UNVERIFIED — settle empirically against a running SITL

These cannot be read off cleanly, and guessing them is how the sim ends up
plausible but wrong. Each gets a test that fails on a sign or index swap.

1. ~~Gyro axis mapping and signs~~ — **DONE**, sections 5a and 5d.
2. ~~Accelerometer semantics~~ — **DONE**: specific force; Betaflight wants it
   in FLU, sections 5a and 5d. The FLU claim for acc rests on it sharing the
   gyro's frame; the probe's tilted-at-rest checks confirm the X and Y signs.
3. ~~Quaternion convention~~ — **DONE**, sections 5a and 5d, including a
   combined roll/pitch/yaw attitude.
4. **Betaflight motor index → physical position** for `mixer QUADX` at 4.5.1,
   which is still unread and is what `motors.betaflight_order` in
   `config/quad.yaml` is flagged `verified: false` for. Combine with the §4.1
   permutation.
5. **Prop spin direction per index**, with `yaw_motors_reversed = OFF`.
6. **RC channel order** — the AETR labelling at `sitl.c:249` is a debug string,
   not a contract. Confirm against `rcmap` handling.

## 6. Building on macOS — native FAILS, Docker works

**Native Apple Silicon: all 257 objects compile, the LINK cannot work.**

Compilation needed no source changes, only flags:

```sh
make TARGET=SITL ARM_SDK_DIR=/usr \
  EXTRA_FLAGS="-Wno-unknown-warning-option -Wno-ignored-optimization-argument \
               -Wno-strict-prototypes -Wno-double-promotion \
               -Wno-unknown-pragmas -Wno-unneeded-internal-declaration"
```

Why each is needed:

| Flag | Reason |
|---|---|
| `ARM_SDK_DIR=/usr` | `mk/tools.mk` is included at `Makefile:104`, *before* the SITL target blanks `ARM_SDK_PREFIX` at `mk/mcu/SITL.mk:15`, so it demands `arm-none-eabi-gcc` even for a host build. Pointing it at any existing directory takes the branch that skips the check. |
| `-Wno-unknown-warning-option` | `Makefile:256` and `:249` pass GCC-only warnings (`-Wunsafe-loop-optimizations`, `-Wold-style-definition`). |
| `-Wno-ignored-optimization-argument` | `Makefile:153` passes GCC's `-fuse-linker-plugin`. |
| `-Wno-strict-prototypes` | Upstream headers declare `f()` without prototypes. |
| `-Wno-double-promotion` | `sitl.c:272-273` passes `float` literals to dyad's `double` parameters. |
| `-Wno-unknown-pragmas` | **Upstream bug:** `msp.c:343` has `#pragma GCC diagnostic ignored` with no matching `push`, but `:351` does `pop`. GCC tolerates it, clang does not. |
| `-Wno-unneeded-internal-declaration` | `voltage.c:150` `voltageMeterAdcChannelMap` is unused in a SITL build. |

Then the link fails, and this one is not a flag problem:

```
ld: unknown options: -gc-sections -Map --cref -T
```

`mk/mcu/SITL.mk:43-54` links with `-T src/main/target/SITL/pg.ld`, plus
`-gc-sections`, `--cref` and `-lrt`. `pg.ld` is a **GNU linker script** that
creates the `.pg_registry` section and the `__pg_registry_start` /
`__pg_registry_end` symbols that Betaflight's entire parameter-group config
system is built on. **Apple's `ld64` has no linker-script support**, so there
is nothing to translate these to; `-lrt` also does not exist on macOS. Making
this work would mean reimplementing the parameter-group section layout for
Mach-O — a port, not a patch, and one that would have to be redone against a
submodule we deliberately keep unmodified.

So: **Docker**, which CLAUDE.md names as the sanctioned fallback.

```sh
./tools/run_sitl.sh          # build the image if needed, then run it
```

See [`../docker/Dockerfile.sitl`](../docker/Dockerfile.sitl). It fetches
Betaflight **by commit** and asserts the checkout matches, and the image is
**tagged by that commit** (`fdt-betaflight-sitl:<12-char sha>`) so that bumping
the firmware is necessarily a cache miss — a fixed `:4.5.1` tag would let
`docker image inspect` succeed and silently keep running the old binary. The
entrypoint resolves the host to a numeric IP because of the `inet_addr()` limit
in §2, and **fails rather than falling back** to the default gateway: on Docker
Desktop that gateway is inside the Linux VM, not the macOS host, so guessing it
would send motor packets into a black hole with every check still passing.

**Ports published: `127.0.0.1:5761/tcp` (Configurator), `127.0.0.1:9003/udp`
and `127.0.0.1:9004/udp` — and nothing else.** Those three are the only ones
SITL listens on. 9001 and 9002 are its *outbound* direction (§2), so they need
no publishing at all: container-outbound UDP is NAT'd anyway, and publishing
them would make Docker take ownership of the host port that the **physics
process** has to bind in order to receive motor outputs — a host `bind()` on a
published port fails with `EADDRINUSE`. Loopback binding matters because MSP on
5761 is unauthenticated full control of the FC, and 9003/9004 accept state and
RC injection — none of *those three* needs to be reachable off-box.

That does **not** extend to the motor direction, and the distinction decides
whether the physics receiver works at all. Because 9001/9002 are NAT'd
container-outbound traffic, motor packets reach the macOS host **from the Docker
VM's address** (a `192.168.65.x` on Docker Desktop), *not* from `127.0.0.1`. So
the physics motor receiver must **`bind(0.0.0.0, 9002)`**; binding `127.0.0.1`
drops every packet with no error anywhere — the same silent "physics never
receives motors" symptom §6 is otherwise built to avoid. Narrow the exposure
with the host firewall, not with the bind address.

`/data` holds `eeprom.bin` — **mount it or the pasted config is lost** when the
container is removed.

### Docker latency caveat for Phase 3

The Phase 3 acceptance criterion is measured end-to-end input latency. Docker
Desktop on macOS runs containers inside a VM, and the UDP round trip crosses
that boundary, so it adds latency that the real FC does not have. Measure and
report the containerised number, but do not mistake it for the hardware figure.
