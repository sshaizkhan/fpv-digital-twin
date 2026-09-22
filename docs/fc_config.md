# What the real flight controller's config tells us

Everything here is read out of `config/diff_all.txt` and `config/dump_all.txt`,
captured from the FC on 2026-09-21. Each row cites the setting it came from, so
it can be re-checked when the config changes.

Re-capture both files after any tune change; `diff all` alone is not enough,
because it only shows settings that differ from defaults.

> `config/dump_all.txt` contains `mcu_id 003a00483433510d37373837`, a unique
> hardware serial for your FC. Harmless, but worth knowing it is in there if
> this repo ever goes public.

## Identity — VERIFIED

| | |
|---|---|
| Firmware | **Betaflight 4.5.1**, built Jul 27 2024, commit `77d01ba3b` |
| MCU / target | STM32F405 (S405), `SPEEDYBEEF405V4`, manufacturer `SPBE` |
| MSP API | 1.46 |
| Config rev | `fb15bf8` |

This is what `firmware.betaflight_version` / `betaflight_git_tag` in
`config/quad.yaml` now pin, and what `third_party/betaflight` must be checked
out at. A test asserts the version in `quad.yaml` actually appears in the dump
headers, so the pin cannot drift away from the hardware.

## Mixer and motors

| Setting | Value | What it means for us |
|---|---|---|
| `mixer` | `QUADX` | Stock quad-X. No custom mix. |
| `mixer_type` | `LEGACY` | Not the newer linear mixer. |
| `smix` | `reset` | No servo mixer overrides. |
| `yaw_motors_reversed` | `OFF` | The firmware expects the **stock** prop direction, and since the quad flies, the props must be mounted to match. **But which physical direction that is per motor index still has to be read out of the Betaflight mixer source** — that is Phase 2's job, so `motors.spin.verified` stays `false`. |
| `motor_output_limit` | `100` | No output derating to model. |
| `motor_pwm_protocol` | `DSHOT300` | |
| `dshot_idle_value` | `550` | 5.5% idle. Betaflight never commands below this when armed, so the motor model's usable command range starts here, not at 0. |
| `dshot_bidir` | `ON` | **Bidirectional DSHOT is on.** |
| `motor_poles` | `14` | Now in `quad.yaml` as `motors.model.poles`, `measured: true`. |

### Why `dshot_bidir = ON` is the best news in these files

RPM telemetry is being logged. That means **`load_factor`, `thrust_coeff` and
`time_constant` can all be fitted directly from a real Blackbox log** in Phase
4, instead of waiting on a thrust stand:

- `load_factor` — full-throttle RPM divided by `kv * V_pack`. Read it straight off.
- `thrust_coeff` — at a steady hover, `4 * kT * omega^2 = m*g`, and omega is logged.
- `time_constant` — fit the first-order lag to a throttle step.

Mechanical RPM is `eRPM / (poles / 2)` = `eRPM / 7`.

## Gyro, filtering and loop rate

| Setting | Value |
|---|---|
| `gyro_hardware_lpf` | `NORMAL` |
| `gyro_lpf1` | PT1, dynamic 250–500 Hz, expo 5 |
| `gyro_lpf2` | PT1, 500 Hz |
| `gyro_notch1/2` | off |
| `dyn_notch_count` / `dyn_notch_q` | 1 / 500 |
| `dterm_lpf1` / `dterm_lpf2` | 75 Hz / 150 Hz |
| `pid_process_denom` | `2` (PID loop runs at half the gyro rate) |
| `gyro_1_sensor_align` | `CW90` (`align_yaw = 900`) |

**The gyro sample rate is still not pinned down.** `acc_hardware` and the gyro
are both `AUTO`, so the dump never names the sensor, and the rate follows the
detected chip. `imu.sample_rate` therefore stays `measured: false` at its 8 kHz
placeholder. Resolve it from the Configurator's Setup tab (which names the
gyro) or CLI `status`.

`gyro_1_sensor_align = CW90` matters for Phase 2: the IMU is mounted rotated on
the board and Betaflight rotates it internally. Whether SITL expects
pre-rotation or post-rotation gyro data is one more thing the packet layer has
to get right, and it is now on the checklist in `sitl_interface.md`.

## Tune (profile 0) — the thing the sim exists to let you change safely

| Axis | P | I | D | F |
|---|---|---|---|---|
| Roll | 45 | 80 | 40 | 120 |
| Pitch | 47 | 84 | 46 | 125 |
| Yaw | 45 | 80 | 0 | 120 |

`anti_gravity_gain 80`, `iterm_relax RP/SETPOINT/15`,
`feedforward_transition 0`, `tpa_rate 65`, `tpa_breakpoint 1350`.

Nothing in the sim reproduces these — they live in Betaflight, which is the
entire point. They are recorded so Phase 2 can confirm SITL came up with the
same tune the real quad flies.

## Rates (rateprofile 0) — Phase 3

`rates_type = ACTUAL`, `rc_rate 7`, `srate 67`, `expo 0` on all three axes;
`thr_mid 50`, `thr_expo 0`.

For ACTUAL rates that should be a centre sensitivity of **70 deg/s** and a max
rate of **670 deg/s** per axis. **UNVERIFIED** — the `rc_rate * 10` / `srate *
10` formula is from memory, not from source. Phase 3 must confirm it against
`src/main/fc/rc.c` at the pinned tag before the stick mapping is trusted.

Also relevant to the radio work: `min_check 1050`, `max_check 1900`,
`rc_smoothing ON` (auto factor 30).

## Modes

| aux | mode id | channel | Range |
|---|---|---|---|
| 0 | 0 (ARM) | AUX1 | 900–1300 — **arms in the LOW position** |
| 1 | 1 (ANGLE) | AUX2 | 1300–1700 |
| 2 | 2 (HORIZON) | AUX2 | 1700–2100 |
| 3 | 13 | AUX3 | 1700–2100 |

Mode ids 0/1/2 are ARM/ANGLE/HORIZON; **id 13 is not resolved** — look it up in
the mode table at the pinned tag rather than guessing. Phase 3 needs all of
these to map the T-Pro's switches onto the right AUX channels.

## Battery

| Setting | Value | Note |
|---|---|---|
| `bat_capacity` | `0` | Not configured, so the FC does not corroborate the 1300 mAh pack. `battery.capacity` in `quad.yaml` stands on your statement of the hardware. |
| `vbat_min_cell_voltage` | 3.30 V | Matches the low end of our OCV curve. |
| `vbat_warning` / `full` / `max` | 3.50 / 4.10 / 4.30 V | Display thresholds, not pack measurements. |
| `vbat_sag_compensation` | `0` | **Off.** Good for us: nothing is masking sag on the real quad, so the sim's sag is directly comparable. |
| `current_meter` / `battery_meter` | `ADC` | `ibata_scale 400` |

## Logging — Phase 4

`blackbox_device = SDCARD`, `blackbox_sample_rate = 1/4`, nothing disabled.
So logs carry PID, RC, setpoint, battery and — with `dshot_bidir` on — RPM, at
a quarter of the PID loop rate.

## Features

`TELEMETRY`, `LED_STRIP`, `OSD`, `ESC_SENSOR`. `acc_calibration 55,161,8,1`
(the accelerometer trim already applied on the real FC).
