"""Landing quality metrics on constructed trajectories."""

import numpy as np
import pytest

from rocket_control.core.mass_properties import com_and_inertia
from rocket_control.landing import LandingProblem, evaluate_landing
from rocket_control.landing.types import Trajectory


def _perfect_touchdown() -> tuple[Trajectory, LandingProblem]:
    problem = LandingProblem()
    v = problem.vehicle
    d_com, _ = com_and_inertia(v.m_dry, v)
    n = 5
    states = np.zeros((n, 7))
    states[:, 1] = np.linspace(100.0, d_com, n)
    states[:, 6] = np.linspace(v.m0, v.m_dry, n)
    states[-1, 1] = d_com
    controls = np.zeros((n, 2))
    times = np.linspace(0, 10, n)
    traj = Trajectory(
        times=times,
        states=states,
        controls=controls,
        tf=10.0,
        success=True,
        initial_state=states[0],
    )
    return traj, problem


def test_perfect_landing_passes():
    traj, problem = _perfect_touchdown()
    metrics = evaluate_landing(traj, problem)
    assert metrics.ok
    assert metrics.issues == ()


def test_offset_landing_fails():
    traj, problem = _perfect_touchdown()
    states = traj.states.copy()
    states[-1, 0] = 5.0
    traj = Trajectory(
        times=traj.times,
        states=states,
        controls=traj.controls,
        tf=traj.tf,
        success=True,
        initial_state=traj.initial_state,
    )
    metrics = evaluate_landing(traj, problem)
    assert not metrics.ok
    assert any("X-position" in i for i in metrics.issues)
