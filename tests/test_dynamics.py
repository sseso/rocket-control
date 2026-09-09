"""Plant identities: hover, free-fall, torque sign, mass flow."""

import numpy as np
import pytest

from rocket_control.core import (
    IM,
    IOMEGA,
    IVX,
    IVY,
    default_vehicle,
    rocket_dynamics,
)
from rocket_control.core.mass_properties import com_and_inertia


def _state(vehicle, *, theta=0.0, omega=0.0, vx=0.0, vy=0.0, y=100.0):
    d_com, _ = com_and_inertia(vehicle.m0, vehicle)
    return np.array([0.0, y + d_com, vx, vy, theta, omega, vehicle.m0])


def test_hover_zero_vertical_accel():
    v = default_vehicle()
    u = np.array([v.m0 * v.g, 0.0])
    xdot = rocket_dynamics(_state(v), u, v)
    assert xdot[IVY] == pytest.approx(0.0, abs=1e-12)
    assert xdot[IVX] == pytest.approx(0.0, abs=1e-12)
    assert xdot[IOMEGA] == pytest.approx(0.0, abs=1e-12)


def test_free_fall():
    v = default_vehicle()
    u = np.array([0.0, 0.0])
    xdot = rocket_dynamics(_state(v), u, v)
    assert xdot[IVY] == pytest.approx(-v.g)
    assert xdot[IM] == pytest.approx(0.0)


def test_mass_flow_is_minus_T_over_ve():
    v = default_vehicle()
    T = 10_000.0
    xdot = rocket_dynamics(_state(v), np.array([T, 0.0]), v)
    assert xdot[IM] == pytest.approx(-T / v.v_e)


def test_positive_gimbal_produces_negative_pitch_accel():
    """Torque is -(d_com T / I) sin(alpha); alpha > 0 => omega-dot < 0."""
    v = default_vehicle()
    alpha = np.deg2rad(5.0)
    xdot = rocket_dynamics(_state(v), np.array([v.T_max, alpha]), v)
    assert xdot[IOMEGA] < 0


def test_vacuum_flag_drops_gravity():
    v = default_vehicle()
    xdot = rocket_dynamics(_state(v), np.array([0.0, 0.0]), v, gravity=False)
    assert xdot[IVY] == pytest.approx(0.0)


def test_casadi_matches_numpy():
    casadi = pytest.importorskip("casadi")
    from rocket_control.core.dynamics import rocket_dynamics_ca

    v = default_vehicle()
    x = _state(v, theta=0.1, omega=-0.05, vx=3.0, vy=-8.0)
    u = np.array([12_000.0, np.deg2rad(-4.0)])
    x_sym = casadi.SX.sym("x", 7)
    u_sym = casadi.SX.sym("u", 2)
    f = casadi.Function("f", [x_sym, u_sym], [rocket_dynamics_ca(x_sym, u_sym, v)])
    np_dot = rocket_dynamics(x, u, v)
    ca_dot = np.array(f(x, u)).astype(float).ravel()
    np.testing.assert_allclose(ca_dot, np_dot, rtol=1e-12, atol=1e-12)
