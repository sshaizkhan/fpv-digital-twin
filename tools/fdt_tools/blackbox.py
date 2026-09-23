"""Load a Betaflight Blackbox log into tidy arrays, in OUR units and frames.

This is a BOUNDARY, like the SITL packet layer, and the same rule applies:
every unit and sign conversion happens here and nowhere else, and each one is
justified against a source.

What the log gives us and what we turn it into:

    time            microseconds        -> seconds, rebased so t[0] == 0
    gyroADC[0..2]   deg/s, FC axes      -> rad/s, our FRD body frame
    rcCommand[0..3] Betaflight units    -> kept raw; the mapping to setpoint is
                                           rate-profile dependent (Phase 3)
    motor[0..3]     protocol units      -> normalised [0, 1]
    eRPM[0..3]      electrical RPM      -> rad/s mechanical, via motor poles
    vbatLatest      0.01 V              -> volts
    amperageLatest  0.01 A              -> amps

Betaflight's gyro is already in the FC's body frame with board alignment
applied, and its axis convention is roll/pitch/yaw = x/y/z. Our FRD frame uses
the same axes, so the conversion is deg/s -> rad/s with NO sign change. That is
an assumption worth restating rather than burying: it is consistent with the
sim's own SITL boundary, where Betaflight's gyro matched our body rates once
SITL's own negation was accounted for.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

DEG_TO_RAD = math.pi / 180.0


@dataclass
class BlackboxLog:
    """One decoded Blackbox log, in SI units and our frames."""

    path: Path
    time_s: np.ndarray                      # seconds, rebased to start at 0
    gyro_rad_s: np.ndarray                  # (N, 3) rad/s, body FRD
    motor: np.ndarray                       # (N, 4) normalised 0..1, Betaflight order
    rpm_rad_s: np.ndarray | None = None     # (N, 4) rad/s mechanical, or None
    vbat_v: np.ndarray | None = None        # volts
    amps_a: np.ndarray | None = None        # amps
    rc_command: np.ndarray | None = None    # (N, 4) raw Betaflight units
    headers: dict = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1] - self.time_s[0]) if len(self.time_s) else 0.0

    @property
    def sample_rate_hz(self) -> float:
        if len(self.time_s) < 2:
            return 0.0
        dt = np.diff(self.time_s)
        median = float(np.median(dt))
        return 1.0 / median if median > 0 else 0.0

    def has_rpm(self) -> bool:
        return self.rpm_rad_s is not None

    def summary(self) -> str:
        lines = [
            f"{self.path.name}: {len(self.time_s)} frames, {self.duration_s:.1f} s "
            f"at {self.sample_rate_hz:.0f} Hz",
            f"  gyro peak : {np.abs(self.gyro_rad_s).max():.1f} rad/s",
            f"  motor     : {self.motor.min():.3f} .. {self.motor.max():.3f}",
        ]
        if self.has_rpm():
            rpm = self.rpm_rad_s * 60.0 / (2.0 * math.pi)
            lines.append(f"  rpm       : {rpm.min():.0f} .. {rpm.max():.0f}")
        else:
            lines.append("  rpm       : ABSENT -- was dshot_bidir on when this was logged?")
        if self.vbat_v is not None:
            lines.append(f"  vbat      : {self.vbat_v.min():.2f} .. {self.vbat_v.max():.2f} V")
        return "\n".join(lines)


def _column(frames: np.ndarray, names: list[str], candidates: list[str]) -> np.ndarray | None:
    """Pull one column by trying several possible field names."""
    for candidate in candidates:
        if candidate in names:
            return frames[:, names.index(candidate)].astype(float)
    return None


def _columns(frames: np.ndarray, names: list[str], pattern: str, count: int) -> np.ndarray | None:
    """Pull `count` columns named e.g. gyroADC[0]..gyroADC[3]."""
    out = []
    for i in range(count):
        column = _column(frames, names, [pattern.format(i)])
        if column is None:
            return None
        out.append(column)
    return np.column_stack(out)


def load(path: str | Path, motor_poles: int = 14, log_index: int = 1,
         motor_range: tuple[float, float] = (0.0, 1.0)) -> BlackboxLog:
    """Decode a Blackbox log.

    `motor_poles` converts the log's ELECTRICAL rpm to mechanical:
    mechanical = eRPM / (poles / 2). Getting this wrong scales every fitted
    rotor speed by a constant, which the fitter would silently absorb into
    thrust_coeff -- which is why quad.yaml keeps it as an explicit,
    provenance-tracked parameter.
    """
    from orangebox import Parser  # imported lazily so --help works without it

    path = Path(path)
    parser = Parser.load(str(path), log_index)
    names = list(parser.field_names)
    rows = np.array([list(frame) for frame in parser.frames], dtype=float)
    if rows.size == 0:
        raise ValueError(f"{path}: no frames decoded (is it a valid Blackbox log?)")

    time_us = _column(rows, names, ["time", "time (us)"])
    if time_us is None:
        raise ValueError(f"{path}: no time field; fields present: {names[:12]}")
    time_s = (time_us - time_us[0]) * 1e-6

    gyro_deg = _columns(rows, names, "gyroADC[{}]", 3)
    if gyro_deg is None:
        raise ValueError(f"{path}: no gyroADC fields -- the log has no gyro data")
    gyro_rad_s = gyro_deg * DEG_TO_RAD

    motor_raw = _columns(rows, names, "motor[{}]", 4)
    if motor_raw is None:
        raise ValueError(f"{path}: no motor fields")
    lo, hi = motor_range
    span = (hi - lo) if hi > lo else 1.0
    motor = np.clip((motor_raw - lo) / span, 0.0, 1.0) if (lo, hi) != (0.0, 1.0) else motor_raw

    erpm = _columns(rows, names, "eRPM[{}]", 4)
    rpm_rad_s = None
    if erpm is not None:
        mechanical_rpm = erpm / (motor_poles / 2.0)
        rpm_rad_s = mechanical_rpm * 2.0 * math.pi / 60.0

    vbat = _column(rows, names, ["vbatLatest", "vbat"])
    amps = _column(rows, names, ["amperageLatest", "amperage"])
    rc = _columns(rows, names, "rcCommand[{}]", 4)

    return BlackboxLog(
        path=path,
        time_s=time_s,
        gyro_rad_s=gyro_rad_s,
        motor=motor,
        rpm_rad_s=rpm_rad_s,
        vbat_v=vbat * 0.01 if vbat is not None else None,
        amps_a=amps * 0.01 if amps is not None else None,
        rc_command=rc,
        headers=dict(parser.headers),
    )
