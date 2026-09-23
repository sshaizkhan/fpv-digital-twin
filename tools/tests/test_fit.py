"""Round-trip the fitters against synthetic logs built from known parameters.

If a fitter cannot recover the values its own model was built from, it has no
business writing numbers into quad.yaml.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fdt_tools import fit, synthetic  # noqa: E402


def test_time_constant_is_recovered():
    log, truth = synthetic.make_log(time_constant=0.025)
    result = fit.fit_time_constant(log)
    assert result.usable, result.describe()
    assert result.value == pytest.approx(truth["time_constant"], rel=0.15), result.describe()


@pytest.mark.parametrize("tau", [0.012, 0.025, 0.045])
def test_time_constant_tracks_the_truth_across_values(tau):
    """A fitter that always returns 25 ms would pass a single-value test."""
    log, truth = synthetic.make_log(time_constant=tau)
    result = fit.fit_time_constant(log)
    assert result.usable, result.describe()
    assert result.value == pytest.approx(tau, rel=0.2), result.describe()


def test_load_factor_is_recovered():
    log, truth = synthetic.make_log(load_factor=0.75)
    result = fit.fit_load_factor(log, truth["kv_rpm_per_volt"])
    assert result.usable, result.describe()
    assert result.value == pytest.approx(0.75, rel=0.05), result.describe()


@pytest.mark.parametrize("lf", [0.65, 0.75, 0.88])
def test_load_factor_tracks_the_truth_across_values(lf):
    log, truth = synthetic.make_log(load_factor=lf)
    result = fit.fit_load_factor(log, truth["kv_rpm_per_volt"])
    assert result.value == pytest.approx(lf, rel=0.05), result.describe()


def test_thrust_coeff_is_recovered():
    log, truth = synthetic.make_log(thrust_coeff=2.93e-6, mass_kg=0.720)
    result = fit.fit_thrust_coeff(log, truth["mass_kg"])
    assert result.usable, result.describe()
    assert result.value == pytest.approx(2.93e-6, rel=0.05), result.describe()


def test_thrust_coeff_scales_with_the_mass_it_is_given():
    """kT is proportional to the assumed mass. If AUW is wrong, kT is wrong by
    the same factor -- the note on the result says so, and this pins it."""
    log, truth = synthetic.make_log(thrust_coeff=2.93e-6, mass_kg=0.720)
    nominal = fit.fit_thrust_coeff(log, 0.720)
    heavy = fit.fit_thrust_coeff(log, 1.440)
    assert heavy.value == pytest.approx(2.0 * nominal.value, rel=1e-6)


def test_a_log_without_rpm_is_not_usable_rather_than_wrong():
    log, truth = synthetic.make_log()
    log.rpm_rad_s = None
    for result in (fit.fit_time_constant(log),
                   fit.fit_load_factor(log, truth["kv_rpm_per_volt"]),
                   fit.fit_thrust_coeff(log, truth["mass_kg"])):
        assert not result.usable
        assert not math.isfinite(result.value)
        assert "RPM" in result.note or "rpm" in result.note


def test_a_too_short_log_is_not_usable():
    log, truth = synthetic.make_log()
    log.time_s = log.time_s[:50]
    log.motor = log.motor[:50]
    log.rpm_rad_s = log.rpm_rad_s[:50]
    log.gyro_rad_s = log.gyro_rad_s[:50]
    log.vbat_v = log.vbat_v[:50]
    result = fit.fit_time_constant(log)
    assert not result.usable, result.describe()


def test_noise_degrades_confidence_but_not_the_estimate():
    clean, truth = synthetic.make_log(time_constant=0.025, rpm_noise_rad_s=0.0)
    noisy, _ = synthetic.make_log(time_constant=0.025, rpm_noise_rad_s=8.0)
    a = fit.fit_time_constant(clean)
    b = fit.fit_time_constant(noisy)
    assert a.value == pytest.approx(0.025, rel=0.15)
    assert b.r_squared < a.r_squared, "noise must show up in the reported quality"


def test_synthetic_hover_command_is_physically_consistent():
    """Guard the generator itself: if its hover point is wrong, every fit that
    depends on it is validated against a lie."""
    log, truth = synthetic.make_log()
    gravity = 9.80665
    thrust = 4.0 * truth["thrust_coeff"] * truth["omega_hover"] ** 2
    assert thrust == pytest.approx(truth["mass_kg"] * gravity, rel=1e-9)
    assert 0.1 < truth["hover_command"] < 0.6, truth["hover_command"]
