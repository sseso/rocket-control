"""Planar rocket plant matching the README:

    xdot = [ v_x,
             v_y,
             (T/m) sin(theta + alpha),
             (T/m) cos(theta + alpha) - g,
             omega,
             -(d_com(m) T / I_z(m)) sin(alpha),
             -T / v_e ]

Set 'gravity=False' for the vacuum attitude demo (no external force).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .mass_properties import com_and_inertia, com_and_inertia_ca
from .types import IALPHA, IM, IOMEGA, IT, ITHETA, IVX, IVY, IX, IY, N_STATES, as_control, as_state
from .vehicle import VehicleSpec


def rocket_dynamics(
    state: np.ndarray,
    control: np.ndarray,
    vehicle: VehicleSpec,
    *,
    gravity: bool = True,
) -> np.ndarray:
    """NumPy plant. 'state' and 'control' are SI (radians)."""
    x = as_state(state)
    u = as_control(control)
    m = x[IM]
    T = u[IT]
    alpha = u[IALPHA]
    theta = x[ITHETA]

    d_com, i_z = com_and_inertia(m, vehicle)
    g = vehicle.g if gravity else 0.0
    thrust_angle = theta + alpha

    xdot = np.empty(N_STATES, dtype=float)
    xdot[IX] = x[IVX]
    xdot[IY] = x[IVY]
    xdot[IVX] = (T / m) * np.sin(thrust_angle)
    xdot[IVY] = (T / m) * np.cos(thrust_angle) - g
    xdot[ITHETA] = x[IOMEGA]
    xdot[IOMEGA] = -(d_com * T / i_z) * np.sin(alpha)
    xdot[IM] = -T / vehicle.v_e
    return xdot


def rocket_dynamics_ca(
    state: Any,
    control: Any,
    vehicle: VehicleSpec,
    *,
    gravity: bool = True,
) -> Any:
    """CasADi plant with the same algebra as :func:'rocket_dynamics'."""
    import casadi as ca

    m = state[IM]
    T = control[IT]
    alpha = control[IALPHA]
    theta = state[ITHETA]
    d_com, i_z = com_and_inertia_ca(m, vehicle)
    g = vehicle.g if gravity else 0.0
    thrust_angle = theta + alpha
    return ca.vertcat(
        state[IVX],
        state[IVY],
        (T / m) * ca.sin(thrust_angle),
        (T / m) * ca.cos(thrust_angle) - g,
        state[IOMEGA],
        -(d_com * T / i_z) * ca.sin(alpha),
        -T / vehicle.v_e,
    )
