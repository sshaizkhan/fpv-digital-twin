#!/usr/bin/env python3
"""Fit quad parameters from a real Blackbox log and optionally write them back.

    ./tools/fit_from_log.py LOG00042.BFL                    # fit and report
    ./tools/fit_from_log.py LOG00042.BFL --write            # write to quad.yaml
    ./tools/fit_from_log.py --self-test                     # no log needed

Needs RPM in the log, which means `dshot_bidir = ON` when it was flown. The
real quad already has it.

Nothing is written unless --write is passed, and then only fits that pass their
own quality checks. A fit that does not is reported with the reason rather than
quietly skipped -- knowing WHY a parameter is still a guess is the point.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fdt_tools import fit, quadyaml, synthetic  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
QUAD_YAML = REPO_ROOT / "config" / "quad.yaml"


def read_scalar(text: str, dotted: str) -> float | None:
    """Read one `value:` out of quad.yaml without a YAML dependency."""
    try:
        start, end, _ = quadyaml._find_block(text, dotted)
    except quadyaml.QuadYamlError:
        return None
    for line in text.splitlines()[start:end]:
        match = re.match(r"^\s*value:\s*([-\d.eE+]+)", line)
        if match:
            return float(match.group(1))
    return None


def run_fits(log, kv, mass, poles_note: str) -> list[tuple[str, fit.FitResult]]:
    return [
        ("motors.model.time_constant", fit.fit_time_constant(log)),
        ("motors.model.load_factor", fit.fit_load_factor(log, kv)),
        ("motors.model.thrust_coeff", fit.fit_thrust_coeff(log, mass)),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", nargs="?", help="Blackbox log (.BFL/.BBL)")
    parser.add_argument("--config", default=str(QUAD_YAML))
    parser.add_argument("--write", action="store_true", help="write usable fits into quad.yaml")
    parser.add_argument("--log-index", type=int, default=1)
    parser.add_argument("--self-test", action="store_true",
                        help="fit a synthetic log with known parameters and report the error")
    args = parser.parse_args()

    config_text = Path(args.config).read_text()
    kv = read_scalar(config_text, "motors.model.kv") or 1700.0
    mass = read_scalar(config_text, "mass.auw") or 0.720
    poles = int(read_scalar(config_text, "motors.model.poles") or 14)
    mass_measured = "measured: true" in quadyaml_block(config_text, "mass.auw")

    if args.self_test:
        log, truth = synthetic.make_log()
        print("self-test: fitting a synthetic log built from known parameters\n")
        results = run_fits(log, truth["kv_rpm_per_volt"], truth["mass_kg"], "")
        worst = 0.0
        for name, result in results:
            key = name.split(".")[-1]
            expected = truth[key]
            error = abs(result.value - expected) / expected
            worst = max(worst, error)
            print(f"  {key:16} fitted {result.value:.6g}  truth {expected:.6g}  "
                  f"error {error * 100:.1f}%  {'ok' if error < 0.2 else 'BAD'}")
        print(f"\nworst error {worst * 100:.1f}%")
        return 0 if worst < 0.2 else 1

    if not args.log:
        parser.error("give a log, or --self-test")

    from fdt_tools import blackbox
    log = blackbox.load(args.log, motor_poles=poles, log_index=args.log_index)
    print(log.summary())
    print(f"\nusing kv={kv:.0f} rpm/V, poles={poles}, AUW={mass:.3f} kg"
          f"{'' if mass_measured else '  <- AUW IS STILL A GUESS'}\n")

    if not log.has_rpm():
        print("This log has no RPM, so none of the motor fits are possible.")
        print("Check `set dshot_bidir = ON` on the FC and fly it again.")
        return 1

    results = run_fits(log, kv, mass, "")
    for _, result in results:
        print(result.describe())

    if not args.write:
        print("\n(nothing written; pass --write to update quad.yaml)")
        return 0

    if not mass_measured:
        print("\nREFUSING to write thrust_coeff: it scales directly with AUW, and AUW")
        print("is still an estimate. Weigh the quad first, or the fit just launders a")
        print("guess into something labelled 'measured'.")

    wrote = 0
    for dotted, result in results:
        if not result.usable:
            print(f"\nskipped {dotted}: {result.note or 'did not pass its quality check'}")
            continue
        if dotted.endswith("thrust_coeff") and not mass_measured:
            continue
        detail = f"n={result.samples}"
        if math.isfinite(result.r_squared):
            detail += f", R2={result.r_squared:.4f}"
        old = quadyaml.update_parameter(
            args.config, dotted, result.value,
            quadyaml.fitted_source(Path(args.log).name, detail))
        print(f"\n{dotted}: {old} -> {result.value:.6g}")
        wrote += 1

    print(f"\nwrote {wrote} parameter(s) into {args.config}")
    print("re-run ./build/make/physics/fdt_config_dump to see what is still unmeasured")
    return 0


def quadyaml_block(text: str, dotted: str) -> str:
    try:
        start, end, _ = quadyaml._find_block(text, dotted)
    except quadyaml.QuadYamlError:
        return ""
    return "".join(text.splitlines(keepends=True)[start:end])


if __name__ == "__main__":
    sys.exit(main())
