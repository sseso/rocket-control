"""Grid axis construction; does not run IPOPT."""

import numpy as np

from rocket_control.landing.grid import grid_axes
from rocket_control.landing.scenarios import GridSpec


def test_grid_axes_include_endpoints():
    spec = GridSpec(x_min=-10, x_max=10, x_step=5, h_min=0, h_max=100, h_step=25)
    xs, hs = grid_axes(spec)
    np.testing.assert_allclose(xs, [-10, -5, 0, 5, 10])
    np.testing.assert_allclose(hs, [0, 25, 50, 75, 100])
