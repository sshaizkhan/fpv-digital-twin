# tools/ — Python side

Phase 4. Nothing here yet beyond the environment definition; this directory is
scaffolded so the venv and dependency set are settled before the work starts.

Planned contents:

- `blackbox.py` — load a real Blackbox log via `orangebox`, into tidy arrays
  (time, gyro, RC, motor outputs, RPM, vbat), in **our** units and frames
  (see `docs/coordinate_frames.md`).
- `replay.py` — feed the logged RC commands into the headless sim and capture
  the resulting gyro trace.
- `overlay.py` — plot sim gyro against real gyro, per axis, with the residual.
- `fit.py` — least-squares fit of `thrust_coeff`, the drag coefficients and
  `time_constant`, writing the results back into `config/quad.yaml` with
  `measured: true` and a source string naming the log it came from.

## Setup

The system Python on macOS is 3.9; use the Homebrew one.

```sh
brew install python
python3.13 -m venv .venv
source .venv/bin/activate
pip install -r tools/requirements.txt
```

## Note on config

`config/quad.yaml` is validated by the **C++** loader
(`physics/src/quad_config.cpp`), which is the authoritative implementation of
the schema. Python tools should read it with `pyyaml` for convenience but must
not grow a second, divergent set of validation rules — if a tool writes the
file back, re-run `fdt_config_dump` (or the test suite) to confirm it is still
valid.
