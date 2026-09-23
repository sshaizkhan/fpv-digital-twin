"""Generate a synthetic Blackbox log from KNOWN parameters.

This is how the fitters get validated without a real flight: build a log from
parameters we chose, fit it, and check the fitter recovers them. A fitter that
cannot round-trip its own model is not going to tell the truth about a real
quad.

The motor model here mirrors physics/src/motor.cpp exactly:

    omega_target = command * kv_rad * load_factor * V
    d(omega)/dt  = (omega_target - omega) / tau
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .blackbox import BlackboxLog


def make_log(
    *,
    kv_rpm_per_volt: float = 1700.0,
    load_factor: float = 0.75,
    time_constant: float = 0.025,
    thrust_coeff: float = 2.93e-6,
    mass_kg: float = 0.720,
    vbat: float = 16.0,
    motor_poles: int = 14,
    rate_hz: float = 2000.0,
    gyro_noise_rad_s: float = 0.01,
    rpm_noise_rad_s: float = 0.0,
    seed: int = 7,
) -> tuple[BlackboxLog, dict]:
    """Build a log containing a hover stretch and several throttle steps.

    Returns the log and the dict of truth values used to make it.
    """
    rng = np.random.default_rng(seed)
    gravity = 9.80665
    kv_rad = kv_rpm_per_volt * 2.0 * math.pi / 60.0

    # The command that hovers, from the same relation the fitter inverts.
    omega_hover = math.sqrt(mass_kg * gravity / (4.0 * thrust_coeff))
    hover_command = omega_hover / (kv_rad * load_factor * vbat)

    # Profile: settle, a long hover, then throttle steps that exercise the lag.
    segments = [
        (1.0, hover_command),
        (6.0, hover_command),   # the steady hover thrust_coeff needs
        (0.6, 0.85),
        (0.6, hover_command),
        (0.6, 0.95),            # high and steady: what load_factor needs
        (1.2, 0.95),
        (0.6, 0.45),
        (0.8, 0.80),
        (0.8, hover_command),
    ]
    command = np.concatenate([np.full(int(d * rate_hz), c) for d, c in segments])
    n = command.size
    t = np.arange(n) / rate_hz

    # Integrate the first-order lag exactly, as motor.cpp does.
    dt = 1.0 / rate_hz
    decay = math.exp(-dt / time_constant)
    omega = np.zeros(n)
    target = command * kv_rad * load_factor * vbat
    current = 0.0
    for i in range(n):
        current = target[i] + (current - target[i]) * decay
        omega[i] = current
    if rpm_noise_rad_s > 0:
        omega = omega + rng.normal(0.0, rpm_noise_rad_s, n)

    rpm = np.column_stack([omega] * 4)
    gyro = rng.normal(0.0, gyro_noise_rad_s, (n, 3))

    log = BlackboxLog(
        path=Path("synthetic.bbl"),
        time_s=t,
        gyro_rad_s=gyro,
        motor=np.column_stack([command] * 4),
        rpm_rad_s=rpm,
        vbat_v=np.full(n, vbat),
        amps_a=None,
        rc_command=None,
        headers={"synthetic": "1"},
    )
    truth = {
        "kv_rpm_per_volt": kv_rpm_per_volt,
        "load_factor": load_factor,
        "time_constant": time_constant,
        "thrust_coeff": thrust_coeff,
        "mass_kg": mass_kg,
        "vbat": vbat,
        "motor_poles": motor_poles,
        "omega_hover": omega_hover,
        "hover_command": hover_command,
    }
    return log, truth
