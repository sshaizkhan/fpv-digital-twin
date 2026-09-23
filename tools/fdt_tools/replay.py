"""Replay a logged motor trace through the physics model and compare gyros.

The comparison this enables is the whole point of the project: same motor
inputs, real quad versus model, and the gyro difference is the model error with
the flight controller taken out of the loop.

MOTOR ORDER. Blackbox logs `motor[0..3]` in BETAFLIGHT's order. Our physics
wants physical motors (FL, FR, RL, RR). The mapping is
`motors.betaflight_order` in quad.yaml, and it is STILL UNVERIFIED -- nobody
has spun the real quad's motors one at a time and written down which moved.
Every function here that applies it says so, and `Mapping.verified` carries the
flag, because a wrong mapping produces a plausible-looking overlay with the
roll and pitch errors swapped.
"""
from __future__ import annotations

import re
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .blackbox import BlackboxLog

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MOTOR_NAMES = ["FL", "FR", "RL", "RR"]


@dataclass
class Mapping:
    """Betaflight motor index (0-based) -> our physical motor index (0-based)."""

    to_physical: list[int]
    verified: bool

    def apply(self, motors: np.ndarray) -> np.ndarray:
        """Reorder an (N, 4) array from Betaflight order into physical order."""
        out = np.empty_like(motors)
        for bf_index, physical in enumerate(self.to_physical):
            out[:, physical] = motors[:, bf_index]
        return out


def load_mapping(config_path: str | Path) -> Mapping:
    """Read motors.betaflight_order out of quad.yaml."""
    text = Path(config_path).read_text()
    start = text.index("  betaflight_order:")
    block = text[start:]
    end = block.index("\n  model:") if "\n  model:" in block else len(block)
    block = block[:end]

    verified = bool(re.search(r"^\s*verified:\s*true\s*$", block, re.MULTILINE))
    to_physical = [-1] * 4
    for match in re.finditer(r"^\s*([1-4]):\s*([A-Z]{2})\s*$", block, re.MULTILINE):
        bf_index = int(match.group(1)) - 1
        name = match.group(2)
        if name not in MOTOR_NAMES:
            raise ValueError(f"unknown motor name in betaflight_order: {name}")
        to_physical[bf_index] = MOTOR_NAMES.index(name)
    if any(v < 0 for v in to_physical) or sorted(to_physical) != [0, 1, 2, 3]:
        raise ValueError(f"betaflight_order is not a 1:1 mapping: {to_physical}")
    return Mapping(to_physical=to_physical, verified=verified)


def write_motor_csv(log: BlackboxLog, mapping: Mapping, path: str | Path,
                    start_s: float | None = None, end_s: float | None = None) -> int:
    """Write t,m0..m3 in PHYSICAL motor order for fdt_replay."""
    mask = np.ones(len(log.time_s), dtype=bool)
    if start_s is not None:
        mask &= log.time_s >= start_s
    if end_s is not None:
        mask &= log.time_s <= end_s
    if mask.sum() < 2:
        raise ValueError("selected window has fewer than two samples")

    t = log.time_s[mask]
    motors = mapping.apply(log.motor[mask])
    rows = np.column_stack([t, motors])
    with open(path, "w") as handle:
        handle.write("t,m0,m1,m2,m3\n")
        for row in rows:
            handle.write(",".join(f"{v:.9g}" for v in row) + "\n")
    return int(mask.sum())


def run_replay(motor_csv: str | Path, out_csv: str | Path, config_path: str | Path,
               binary: str | Path | None = None) -> str:
    """Invoke fdt_replay. The physics stays in C++ so there is one model, not two."""
    exe = Path(binary) if binary else REPO_ROOT / "build" / "make" / "physics" / "fdt_replay"
    if not exe.exists():
        raise FileNotFoundError(
            f"{exe} not built. Run: cmake --build --preset make")
    result = subprocess.run(
        [str(exe), "--config", str(config_path), "--in", str(motor_csv), "--out", str(out_csv)],
        capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"fdt_replay failed: {result.stderr.strip() or result.stdout.strip()}")
    return result.stdout


def read_sim_csv(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """-> (time_s, gyro_rad_s (N,3))"""
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = np.asarray(data["t"], dtype=float)
    gyro = np.column_stack([data["gyro_x"], data["gyro_y"], data["gyro_z"]]).astype(float)
    return t, gyro


def replay_log(log: BlackboxLog, config_path: str | Path,
               start_s: float | None = None, end_s: float | None = None,
               binary: str | Path | None = None) -> tuple[np.ndarray, np.ndarray, Mapping, str]:
    """Full path: log -> motor CSV -> fdt_replay -> sim gyro resampled onto the
    log's own timebase, so the two traces can be compared sample for sample."""
    mapping = load_mapping(config_path)
    with tempfile.TemporaryDirectory() as tmp:
        motor_csv = Path(tmp) / "motors.csv"
        sim_csv = Path(tmp) / "sim.csv"
        write_motor_csv(log, mapping, motor_csv, start_s, end_s)
        stdout = run_replay(motor_csv, sim_csv, config_path, binary)
        sim_t, sim_gyro = read_sim_csv(sim_csv)

    mask = np.ones(len(log.time_s), dtype=bool)
    if start_s is not None:
        mask &= log.time_s >= start_s
    if end_s is not None:
        mask &= log.time_s <= end_s
    real_t = log.time_s[mask]

    resampled = np.column_stack(
        [np.interp(real_t, sim_t, sim_gyro[:, axis]) for axis in range(3)])
    return real_t, resampled, mapping, stdout
