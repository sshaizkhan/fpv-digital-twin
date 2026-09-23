"""Least-squares fits of motor parameters from a Blackbox log.

These exist because `dshot_bidir` is ON on the real quad, so its logs carry
RPM. That turns three of the most important placeholders in quad.yaml into
measurements without a thrust stand:

    time_constant   from the rotor's response to a throttle step
    load_factor     from the steady rotor speed at a known throttle and voltage
    thrust_coeff    from a steady hover, where 4*kT*omega^2 == m*g

Every fit returns a FitResult carrying its own quality metrics and the number
of samples behind it, because a fit from three noisy points is not a
measurement and should not be written into quad.yaml as one.

NOT fitted here, deliberately: the drag coefficients. Drag needs AIRSPEED, and
a Blackbox log without GPS has no velocity -- integrating the accelerometer
drifts far too quickly to fit a quadratic term against. Doing it properly wants
either GPS, or a sim-vs-real gyro overlay that solves for drag as the residual.
Claiming a drag fit from this data would be fabricating a measurement.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .blackbox import BlackboxLog


@dataclass
class FitResult:
    name: str
    value: float
    units: str
    samples: int
    rms_residual: float = float("nan")
    r_squared: float = float("nan")
    note: str = ""

    @property
    def usable(self) -> bool:
        """Whether this is solid enough to write into quad.yaml as measured."""
        if self.samples < 200:
            return False
        if not math.isfinite(self.value) or self.value <= 0.0:
            return False
        if math.isfinite(self.r_squared) and self.r_squared < 0.9:
            return False
        return True

    def describe(self) -> str:
        bits = [f"{self.name:16} = {self.value:.6g} {self.units}", f"n={self.samples}"]
        if math.isfinite(self.r_squared):
            bits.append(f"R2={self.r_squared:.4f}")
        if math.isfinite(self.rms_residual):
            bits.append(f"rms={self.rms_residual:.4g}")
        bits.append("USABLE" if self.usable else "NOT usable")
        line = "  ".join(bits)
        return f"{line}\n    {self.note}" if self.note else line


def _r_squared(actual: np.ndarray, predicted: np.ndarray) -> float:
    ss_res = float(np.sum((actual - predicted) ** 2))
    ss_tot = float(np.sum((actual - np.mean(actual)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def fit_time_constant(log: BlackboxLog, motor: int = 0) -> FitResult:
    """Fit the first-order motor lag over the whole trace.

    The model is

        d(omega)/dt = (K * command - omega) / tau

    which is LINEAR in two unknowns once rearranged:

        omega_dot = (K/tau) * command - (1/tau) * omega

    so a single least-squares solve gives both tau and the steady-state gain K,
    with no need to hunt for a clean step (real logs rarely contain one).

    Fitting K rather than assuming it matters: an earlier version estimated the
    target speed as command * peak_omega, but the peak occurs at ~95% throttle,
    so the target was systematically low and tau came out 35% high against
    synthetic data whose true value was known.
    """
    if not log.has_rpm():
        return FitResult("time_constant", float("nan"), "s", 0,
                         note="no RPM in this log; enable dshot_bidir before flying it")

    omega = log.rpm_rad_s[:, motor]
    command = log.motor[:, motor]
    t = log.time_s
    if len(t) < 3:
        return FitResult("time_constant", float("nan"), "s", len(t), note="log too short")

    omega_dot = np.gradient(omega, t)
    scale = float(np.percentile(omega, 99.5))
    if scale <= 0:
        return FitResult("time_constant", float("nan"), "s", 0, note="rotor never span up")

    # Skip the parked rotor: it carries no information about the lag, and its
    # dead band would drag the regression toward the origin.
    mask = (omega > 0.02 * scale) | (command > 0.02)
    if mask.sum() < 200:
        return FitResult("time_constant", float("nan"), "s", int(mask.sum()),
                         note="not enough throttle activity; fly some throttle steps")

    design = np.column_stack([command[mask], -omega[mask]])
    coeffs, *_ = np.linalg.lstsq(design, omega_dot[mask], rcond=None)
    a, b = float(coeffs[0]), float(coeffs[1])
    if b <= 0:
        return FitResult("time_constant", float("nan"), "s", int(mask.sum()),
                         note="rotor response has the wrong sign; check the RPM decoding")

    tau = 1.0 / b
    predicted = design @ coeffs
    return FitResult("time_constant", tau, "s", int(mask.sum()),
                     rms_residual=float(np.sqrt(np.mean((omega_dot[mask] - predicted) ** 2))),
                     r_squared=_r_squared(omega_dot[mask], predicted),
                     note=f"two-parameter fit; implied steady-state gain {a / b:.0f} rad/s per unit command")


def fit_load_factor(log: BlackboxLog, kv_rpm_per_volt: float, motor: int = 0,
                    min_command: float = 0.55) -> FitResult:
    """Loaded rotor speed as a fraction of the no-load kv * V.

    Only near-steady, high-throttle samples are used: at low throttle the lag
    dominates, and the whole point of this number is the LOADED speed the motor
    actually reaches.
    """
    if not log.has_rpm():
        return FitResult("load_factor", float("nan"), "fraction", 0,
                         note="no RPM in this log")
    if log.vbat_v is None:
        return FitResult("load_factor", float("nan"), "fraction", 0,
                         note="no vbat in this log, so no reference for kv * V")

    omega = log.rpm_rad_s[:, motor]
    command = log.motor[:, motor]
    kv_rad = kv_rpm_per_volt * 2.0 * math.pi / 60.0

    omega_dot = np.gradient(omega, log.time_s)
    steady = np.abs(omega_dot) < 0.02 * max(float(np.percentile(omega, 99.5)), 1.0) / 0.025
    mask = steady & (command > min_command) & (log.vbat_v > 1.0)
    if mask.sum() < 200:
        return FitResult("load_factor", float("nan"), "fraction", int(mask.sum()),
                         note=f"not enough steady samples above {min_command:.0%} throttle")

    no_load = command[mask] * kv_rad * log.vbat_v[mask]
    ratio = omega[mask] / no_load
    value = float(np.median(ratio))
    return FitResult("load_factor", value, "fraction", int(mask.sum()),
                     rms_residual=float(np.std(ratio)),
                     note=f"median of omega / (command * kv_rad * vbat) over steady samples")


def fit_thrust_coeff(log: BlackboxLog, mass_kg: float, gravity: float = 9.80665,
                     hover_tolerance: float = 0.15) -> FitResult:
    """kT from a steady hover: 4 * kT * omega^2 == m * g.

    A hover is detected as a stretch where the gyro is quiet and all four
    rotors are turning at a similar, steady speed. That is necessary but NOT
    sufficient -- the quad could be climbing or descending at constant speed
    and look identical here. Without altitude there is no way to tell, so this
    is reported with that caveat rather than silently trusted.
    """
    if not log.has_rpm():
        return FitResult("thrust_coeff", float("nan"), "N/(rad/s)^2", 0,
                         note="no RPM in this log")

    omega = log.rpm_rad_s
    mean_omega = omega.mean(axis=1)
    spread = omega.max(axis=1) - omega.min(axis=1)
    gyro_mag = np.linalg.norm(log.gyro_rad_s, axis=1)

    scale = max(float(np.percentile(mean_omega, 99.5)), 1.0)
    mask = (gyro_mag < 0.5) & (spread < hover_tolerance * scale) & (mean_omega > 0.2 * scale)
    if mask.sum() < 200:
        return FitResult("thrust_coeff", float("nan"), "N/(rad/s)^2", int(mask.sum()),
                         note="no steady, level, four-rotors-matched stretch found")

    omega_hover = float(np.median(mean_omega[mask]))
    value = mass_kg * gravity / (4.0 * omega_hover ** 2)
    return FitResult("thrust_coeff", value, "N/(rad/s)^2", int(mask.sum()),
                     note=(f"from a {mask.sum() / max(log.sample_rate_hz, 1):.1f} s steady stretch at "
                           f"{omega_hover:.0f} rad/s, assuming mass {mass_kg:.3f} kg and that the quad "
                           f"was neither climbing nor descending. SCALES DIRECTLY WITH MASS -- "
                           f"measure AUW before trusting this."))
