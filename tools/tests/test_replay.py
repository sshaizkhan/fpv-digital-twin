"""Replay round-trip: drive the physics with a known motor trace, dress the
result up as a Blackbox log, and check the pipeline reproduces it.

This exercises the motor-order mapping, the CSV boundary, the resampling onto
the log's timebase, and the comparison metrics. A test that only checked
"errors are small" would pass with a broken mapping on a symmetric input, so
there is an asymmetric input and an explicit wrong-mapping case.
"""
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fdt_tools import blackbox, overlay, replay  # noqa: E402

REPO = Path(__file__).resolve().parent.parent.parent
BINARY = REPO / "build" / "make" / "physics" / "fdt_replay"
CONFIG = REPO / "config" / "quad.yaml"

pytestmark = pytest.mark.skipif(not BINARY.exists(), reason="fdt_replay not built")


def asymmetric_motor_trace(n=4000, rate=1000.0):
    """Four DIFFERENT motor signals, so a swapped pair cannot go unnoticed."""
    t = np.arange(n) / rate
    base = 0.35
    return t, np.column_stack([
        base + 0.05 * np.sin(2 * np.pi * 1.1 * t),
        base + 0.05 * np.sin(2 * np.pi * 1.7 * t + 0.5),
        base + 0.05 * np.sin(2 * np.pi * 2.3 * t + 1.0),
        base + 0.05 * np.sin(2 * np.pi * 3.1 * t + 1.5),
    ])


def run_truth(tmp_path, t, motors_physical):
    """Run fdt_replay directly on a physical-order trace -> the 'real' gyro."""
    motor_csv = tmp_path / "truth_motors.csv"
    with open(motor_csv, "w") as handle:
        handle.write("t,m0,m1,m2,m3\n")
        for row in np.column_stack([t, motors_physical]):
            handle.write(",".join(f"{v:.9g}" for v in row) + "\n")
    sim_csv = tmp_path / "truth_sim.csv"
    replay.run_replay(motor_csv, sim_csv, CONFIG, BINARY)
    return replay.read_sim_csv(sim_csv)


def make_log_from(t, motors_physical, gyro_t, gyro, mapping):
    """Dress a physical-order trace up as a Blackbox log, i.e. in BF order."""
    inverse = np.empty((len(t), 4))
    for bf_index, physical in enumerate(mapping.to_physical):
        inverse[:, bf_index] = motors_physical[:, physical]
    resampled = np.column_stack([np.interp(t, gyro_t, gyro[:, a]) for a in range(3)])
    return blackbox.BlackboxLog(
        path=Path("roundtrip.bbl"), time_s=t, gyro_rad_s=resampled,
        motor=inverse, rpm_rad_s=None, vbat_v=None, amps_a=None,
        rc_command=None, headers={})


def test_replay_reproduces_its_own_trace(tmp_path):
    t, motors = asymmetric_motor_trace()
    gyro_t, truth_gyro = run_truth(tmp_path, t, motors)
    mapping = replay.load_mapping(CONFIG)
    log = make_log_from(t, motors, gyro_t, truth_gyro, mapping)

    real_t, sim_gyro, used, _ = replay.replay_log(log, CONFIG, binary=BINARY)
    comparisons = overlay.compare(log.gyro_rad_s, sim_gyro)

    assert used.to_physical == mapping.to_physical
    for c in comparisons:
        assert c.rms_error_rad_s < 1e-3, overlay.summarise(comparisons)
        assert c.correlation > 0.999, overlay.summarise(comparisons)


def test_a_wrong_motor_order_shows_up_as_error(tmp_path):
    """If a swapped mapping passed, the round-trip would prove nothing."""
    t, motors = asymmetric_motor_trace()
    gyro_t, truth_gyro = run_truth(tmp_path, t, motors)
    good = replay.load_mapping(CONFIG)
    log = make_log_from(t, motors, gyro_t, truth_gyro, good)

    # Swap two motors in the mapping used for the replay.
    broken = replay.Mapping(to_physical=list(good.to_physical), verified=False)
    broken.to_physical[0], broken.to_physical[1] = broken.to_physical[1], broken.to_physical[0]

    motor_csv = tmp_path / "bad.csv"
    replay.write_motor_csv(log, broken, motor_csv)
    sim_csv = tmp_path / "bad_sim.csv"
    replay.run_replay(motor_csv, sim_csv, CONFIG, BINARY)
    sim_t, sim_gyro = replay.read_sim_csv(sim_csv)
    resampled = np.column_stack([np.interp(t, sim_t, sim_gyro[:, a]) for a in range(3)])

    comparisons = overlay.compare(log.gyro_rad_s, resampled)
    worst = max(c.rms_error_rad_s for c in comparisons)
    assert worst > 1e-2, f"a swapped motor order must be visible:\n{overlay.summarise(comparisons)}"


def test_mapping_is_a_permutation_and_flags_unverified():
    mapping = replay.load_mapping(CONFIG)
    assert sorted(mapping.to_physical) == [0, 1, 2, 3]
    assert mapping.verified is False, (
        "motors.betaflight_order is still unverified; if this now passes, the "
        "real quad's motor tab was checked and the overlay can be trusted")


def test_compare_rejects_mismatched_shapes():
    with pytest.raises(ValueError):
        overlay.compare(np.zeros((10, 3)), np.zeros((9, 3)))


def test_metrics_are_zero_for_identical_traces():
    gyro = np.random.default_rng(1).normal(size=(500, 3))
    for c in overlay.compare(gyro, gyro):
        assert c.rms_error_rad_s == pytest.approx(0.0, abs=1e-12)
        assert c.correlation == pytest.approx(1.0, abs=1e-9)


def test_plot_writes_a_file(tmp_path):
    t = np.linspace(0, 1, 200)
    real = np.column_stack([np.sin(2 * np.pi * t)] * 3)
    sim = real * 1.05
    comparisons = overlay.compare(real, sim)
    out = tmp_path / "overlay.png"
    overlay.plot(t, real, sim, comparisons, str(out), title="test")
    assert out.exists() and out.stat().st_size > 5000
