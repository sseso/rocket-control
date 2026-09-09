"""Attitude controller and vacuum plant checks."""

import numpy as np
import pytest

from rocket_control.core import InfeasibleError, default_vehicle
from rocket_control.attitude.angles import wrap_angle
from rocket_control.attitude.config import AttitudeSettings
from rocket_control.attitude.feasibility import plan_manoeuvre
from rocket_control.attitude.simulate import run_simulation


def test_wrap_angle_radians():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(np.pi) == pytest.approx(np.pi) or wrap_angle(np.pi) == pytest.approx(-np.pi)
    assert wrap_angle(3.0 * np.pi / 2) == pytest.approx(-np.pi / 2)


def test_infeasible_without_fuel():
    v = default_vehicle()
    from dataclasses import replace
    dry = replace(v, m_fuel=0.0)
    with pytest.raises(InfeasibleError):
        plan_manoeuvre(0.0, np.deg2rad(40.0), 0.0, dry.T_max, dry, AttitudeSettings())


def test_short_slew_reaches_target():
    v = default_vehicle()
    settings = AttitudeSettings()
    theta0 = 0.0
    target = np.deg2rad(15.0)
    plan = plan_manoeuvre(theta0, target, 0.0, v.T_max, v, settings)
    result = run_simulation(
        theta0, 0.0, v.T_max, plan, v, settings, include_translation=False,
    )
    assert abs(wrap_angle(result.theta_unwrapped[int(result.final_time / settings.dt) - 1] - target)) < np.deg2rad(2.0)
