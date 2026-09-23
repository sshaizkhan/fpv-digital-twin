"""Compare a replayed gyro trace against the real one, and plot it.

The numbers matter more than the picture. A plot that "looks close" is how a
model with a 20% thrust error gets signed off, so every overlay carries
per-axis RMS error, peak error, and correlation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

AXES = ["roll", "pitch", "yaw"]


@dataclass
class AxisComparison:
    axis: str
    rms_error_rad_s: float
    peak_error_rad_s: float
    correlation: float
    real_rms_rad_s: float

    @property
    def rms_error_pct_of_signal(self) -> float:
        return 100.0 * self.rms_error_rad_s / self.real_rms_rad_s if self.real_rms_rad_s > 0 else float("nan")

    def describe(self) -> str:
        return (f"  {self.axis:6} rms err {self.rms_error_rad_s:7.3f} rad/s "
                f"({self.rms_error_pct_of_signal:5.1f}% of signal)   "
                f"peak {self.peak_error_rad_s:7.3f}   r={self.correlation:6.3f}")


def compare(real_gyro: np.ndarray, sim_gyro: np.ndarray) -> list[AxisComparison]:
    if real_gyro.shape != sim_gyro.shape:
        raise ValueError(f"shape mismatch: real {real_gyro.shape} vs sim {sim_gyro.shape}")
    out = []
    for axis in range(3):
        a = real_gyro[:, axis]
        b = sim_gyro[:, axis]
        error = b - a
        real_rms = float(np.sqrt(np.mean(a ** 2)))
        if np.std(a) > 0 and np.std(b) > 0:
            correlation = float(np.corrcoef(a, b)[0, 1])
        else:
            correlation = float("nan")
        out.append(AxisComparison(
            axis=AXES[axis],
            rms_error_rad_s=float(np.sqrt(np.mean(error ** 2))),
            peak_error_rad_s=float(np.max(np.abs(error))),
            correlation=correlation,
            real_rms_rad_s=real_rms,
        ))
    return out


def summarise(comparisons: list[AxisComparison]) -> str:
    lines = ["sim vs real gyro:"] + [c.describe() for c in comparisons]
    worst = max(c.rms_error_pct_of_signal for c in comparisons
                if math.isfinite(c.rms_error_pct_of_signal))
    lines.append(f"  worst axis: {worst:.1f}% of signal RMS")
    return "\n".join(lines)


def plot(time_s: np.ndarray, real_gyro: np.ndarray, sim_gyro: np.ndarray,
         comparisons: list[AxisComparison], path: str, title: str = "") -> None:
    import matplotlib
    matplotlib.use("Agg")  # no display in a headless run
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for index, ax in enumerate(axes):
        ax.plot(time_s, real_gyro[:, index], linewidth=1.0, label="real (Blackbox)")
        ax.plot(time_s, sim_gyro[:, index], linewidth=1.0, label="sim", alpha=0.85)
        c = comparisons[index]
        ax.set_ylabel(f"{c.axis} (rad/s)")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"{c.axis}: rms err {c.rms_error_rad_s:.3f} rad/s "
                     f"({c.rms_error_pct_of_signal:.1f}% of signal), r={c.correlation:.3f}",
                     fontsize=9, loc="left")
    axes[-1].set_xlabel("time (s)")
    if title:
        fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
