"""Trapezoidal defect on a ballistic (thrust-off) trajectory is machine-zero.

Constant acceleration is integrated exactly by the trapezoidal rule.
"""

import numpy as np
import pytest

from rocket_control.core import IM, IVX, IVY, IX, IY, default_vehicle, rocket_dynamics
from rocket_control.core.mass_properties import com_and_inertia
from rocket_control.landing.nlp import regularize_initial_state


def test_regularize_zero_states():
    s = np.array([0.0, 100.0, 0.0, 0.0, 0.1, 0.0, 1000.0])
    out = regularize_initial_state(s, 1e-4)
    assert out[0] == pytest.approx(1e-4)
    assert out[2] == pytest.approx(1e-4)
    assert out[3] == pytest.approx(1e-4)
    assert s[0] == 0.0  # input not mutated


def test_trapezoidal_residual_free_fall():
    v = default_vehicle()
    d_com, _ = com_and_inertia(v.m0, v)
    x0 = np.array([10.0, 200.0 + d_com, 3.0, -5.0, 0.0, 0.0, v.m0])
    u = np.array([0.0, 0.0])
    n = 20
    tf = 2.0
    dt = tf / n
    times = np.linspace(0.0, tf, n + 1)
    states = np.zeros((n + 1, 7))
    states[0] = x0
    for k in range(n):
        # exact ballistic step (constant accel)
        acc = rocket_dynamics(states[k], u, v)
        states[k + 1] = states[k].copy()
        states[k + 1, IVX] = states[k, IVX] + acc[IVX] * dt
        states[k + 1, IVY] = states[k, IVY] + acc[IVY] * dt
        states[k + 1, IX] = states[k, IX] + 0.5 * (states[k, IVX] + states[k + 1, IVX]) * dt
        states[k + 1, IY] = states[k, IY] + 0.5 * (states[k, IVY] + states[k + 1, IVY]) * dt
        states[k + 1, IM] = states[k, IM]

    max_defect = 0.0
    for k in range(n):
        f_k = rocket_dynamics(states[k], u, v)
        f_kp1 = rocket_dynamics(states[k + 1], u, v)
        predicted = states[k] + 0.5 * dt * (f_k + f_kp1)
        max_defect = max(max_defect, float(np.max(np.abs(predicted - states[k + 1]))))
    assert max_defect < 1e-12
    assert times[-1] == pytest.approx(tf)
