"""Internal plant uses radians; degree conversion is an I/O concern."""

import numpy as np

from rocket_control.core import ITHETA, default_vehicle, rocket_dynamics
from rocket_control.core.mass_properties import com_and_inertia


def test_ten_degrees_is_not_ten_radians():
    v = default_vehicle()
    d_com, _ = com_and_inertia(v.m0, v)
    base = np.array([0.0, 200.0 + d_com, 0.0, 0.0, 0.0, 0.0, v.m0])
    u = np.array([v.T_max, 0.0])

    x_deg_as_rad = base.copy()
    x_deg_as_rad[ITHETA] = 10.0  # wrong: 10 rad
    x_correct = base.copy()
    x_correct[ITHETA] = np.deg2rad(10.0)

    wrong = rocket_dynamics(x_deg_as_rad, u, v)
    right = rocket_dynamics(x_correct, u, v)
    assert not np.allclose(wrong, right)
