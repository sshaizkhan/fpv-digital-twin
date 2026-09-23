#!/usr/bin/env python3
"""Replay a Blackbox log's motor outputs through the physics and overlay the gyros.

    ./tools/replay_log.py LOG00042.BFL --plot overlay.png
    ./tools/replay_log.py LOG00042.BFL --start 12 --end 20 --plot punch.png

What this compares: the motor outputs the FC actually produced, fed into our
model, against the gyro the real quad actually produced. The flight controller
is OUT of the loop, so any disagreement is the physics model -- there is
nowhere else for the error to hide. That is what makes it useful for fitting.

It is NOT a test of the tune. For that you want Betaflight in the loop, which
means SITL and real time (fdt_sitl_hover).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fdt_tools import blackbox, overlay, replay  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent


def read_poles(config_path: Path) -> int:
    import re
    text = config_path.read_text()
    start = text.find("    poles:")
    if start < 0:
        return 14
    match = re.search(r"value:\s*(\d+)", text[start:start + 800])
    return int(match.group(1)) if match else 14


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log")
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "quad.yaml"))
    parser.add_argument("--start", type=float, default=None, help="window start, seconds")
    parser.add_argument("--end", type=float, default=None, help="window end, seconds")
    parser.add_argument("--plot", default=None, help="write an overlay PNG here")
    parser.add_argument("--log-index", type=int, default=1)
    args = parser.parse_args()

    config_path = Path(args.config)
    log = blackbox.load(args.log, motor_poles=read_poles(config_path), log_index=args.log_index)
    print(log.summary())

    try:
        real_t, sim_gyro, mapping, stdout = replay.replay_log(
            log, config_path, args.start, args.end)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"\nreplay failed: {exc}", file=sys.stderr)
        return 2

    print()
    print(stdout.strip())

    real_gyro = log.gyro_rad_s[
        (log.time_s >= (args.start if args.start is not None else -1e30)) &
        (log.time_s <= (args.end if args.end is not None else 1e30))]

    comparisons = overlay.compare(real_gyro, sim_gyro)
    print()
    print(overlay.summarise(comparisons))

    if not mapping.verified:
        print("\nWARNING: motors.betaflight_order is UNVERIFIED. A wrong motor order")
        print("         produces a plausible overlay with the roll and pitch errors")
        print("         swapped. Spin the real quad's motors one at a time in the")
        print("         Configurator motor tab and set verified: true before trusting")
        print("         these numbers to drive a fit.")

    if args.plot:
        overlay.plot(real_t, real_gyro, sim_gyro, comparisons, args.plot,
                     title=f"{Path(args.log).name}: real vs model (motor replay)")
        print(f"\nwrote {args.plot}")

    worst = max(c.rms_error_pct_of_signal for c in comparisons)
    print(f"\nworst-axis RMS error is {worst:.1f}% of the signal.")
    if worst > 40.0:
        print("That is a long way off. Before tuning coefficients, check the motor")
        print("order and that AUW and the inertia tensor are real measurements.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
