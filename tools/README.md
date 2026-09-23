# tools/ — Python side

Phase 4: Blackbox parsing, parameter fitting, and writeback into `quad.yaml`.

## Setup

The system Python on macOS is 3.9; use the Homebrew one.

```sh
brew install python
python3.13 -m venv .venv
.venv/bin/pip install numpy scipy matplotlib pyyaml pytest
.venv/bin/pip install orangebox            # SEPARATELY -- see below
```

**Install `orangebox` on its own.** Version 0.5.0 declares a malformed console
script (`bb2csv = scripts:None`) that modern pip rejects, and in a combined
`pip install -r` that error aborts the whole transaction, taking numpy and the
rest down with it. Installed alone it errors on the entry point but the library
itself installs and imports fine, which is all we use.

## What is here

| | |
|---|---|
| `fdt_tools/blackbox.py` | Decode a log into tidy arrays, in **our** units and frames |
| `fdt_tools/fit.py` | Least-squares fits for `time_constant`, `load_factor`, `thrust_coeff` |
| `fdt_tools/synthetic.py` | Build a log from KNOWN parameters, to validate the fitters |
| `fdt_tools/quadyaml.py` | Surgical writeback that preserves the file's comments |
| `fit_from_log.py` | The CLI |
| `check_sitl.py`, `load_config_sitl.py`, `run_sitl.sh` | Phase 2 SITL tooling |

## Fitting from a real log

```sh
.venv/bin/python tools/fit_from_log.py LOG00042.BFL           # fit and report
.venv/bin/python tools/fit_from_log.py LOG00042.BFL --write   # update quad.yaml
.venv/bin/python tools/fit_from_log.py --self-test            # no log needed
```

The log must contain RPM, which means `dshot_bidir = ON` when it was flown —
the real quad already has it. That single setting is what makes three of the
most important placeholders fittable without a thrust stand.

Nothing is written without `--write`, and then only fits that pass their own
quality checks (sample count, R², sign). A fit that fails is reported **with
the reason**, because knowing *why* a parameter is still a guess is the point.

### thrust_coeff is refused while AUW is a guess

`kT` comes from `4·kT·ω² = m·g`, so it scales **directly** with the mass it is
given. Writing it while `mass.auw` is still `measured: false` would launder an
estimate into something labelled "measured". The CLI refuses, and says so.
Weigh the quad first.

## Validating the fitters

There is no real log in the repo, so the fitters are validated by round-trip:
build a log from known parameters, fit it, check the values come back.

```sh
.venv/bin/python -m pytest tools/tests -q
```

The parametrised cases matter more than the single-value ones — a fitter that
always returned 25 ms would pass a test that only ever checks 25 ms. This
caught a real bias: the first `time_constant` fit estimated the target speed as
`command × peak_omega`, but the peak occurs at ~95% throttle, so the target was
systematically low and `tau` came out **35% high**. It is now a two-parameter
solve that fits the steady-state gain alongside `tau`, with no such guess.

## Not fitted, deliberately

**The drag coefficients.** Drag needs airspeed, and a Blackbox log without GPS
has no velocity — integrating the accelerometer drifts far too fast to fit a
quadratic term against. Doing it properly wants GPS, or a sim-vs-real gyro
overlay that solves for drag as the residual. Claiming a drag fit from this
data would be fabricating a measurement.

**The sim-vs-real overlay and stick replay** are the remaining Phase 4 pieces.

## Note on quad.yaml

The C++ loader (`physics/src/quad_config.cpp`) is the authoritative
implementation of the schema. `quadyaml.py` edits the file as text rather than
round-tripping it through a YAML library, because the comments record what is
known versus guessed and a dump would discard all of them. After any write,
`fdt_config_dump` (or the C++ test suite) confirms the file is still valid —
there is a test that does exactly that.
