"""Heuristic pre-flight check.

The delta v budget is a rough gravity-loss estimate. A True result does not
prove a feasible landing exists; a False result is only a cheap reject.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from rocket_control.core import IM, IVX, IVY, ITHETA, IOMEGA, IX, IY
from rocket_control.core.mass_properties import com_and_inertia

from .scenarios import LandingProblem


@dataclass(frozen=True)
class HeuristicFeasibility:
    ok: bool
    reasons: tuple[str, ...]


def heuristic_feasible(initial_state: Sequence[float], problem: LandingProblem) -> HeuristicFeasibility:
    cfg = problem.feasibility
    vehicle = problem.vehicle
    s = np.asarray(initial_state, dtype=float)
    x, y, vx, vy, theta, omega, m = (
        s[IX], s[IY], s[IVX], s[IVY], s[ITHETA], s[IOMEGA], s[IM],
    )
    reasons: List[str] = []
    fuel = m - vehicle.m_dry
    if fuel < 1e-6:
        reasons.append("no usable fuel")

    d_com, _ = com_and_inertia(m, vehicle)
    nozzle_y = y - d_com
    if nozzle_y < cfg.nozzle_ground_threshold:
        reasons.append("nozzle already at or below ground")

    hover = m * vehicle.g
    if vehicle.T_max < cfg.hover_thrust_safety_factor * hover:
        reasons.append("thrust too low to hover")

    dv_available = vehicle.v_e * np.log(m / vehicle.m_dry)
    speed = np.sqrt(vx**2 + vy**2)
    dv_required = speed + cfg.dv_gravity_loss_factor * vehicle.g * (
        abs(vy) / vehicle.g + cfg.dv_time_margin
    )
    if dv_available < cfg.dv_available_margin * dv_required:
        reasons.append(
            f"delta-v budget too low: {dv_available:.1f} <~ {dv_required:.1f} m/s"
        )

    if abs(theta) > cfg.initial_theta_max or abs(omega) > cfg.initial_omega_max:
        reasons.append("initial attitude or rate too extreme")
    if abs(x) > cfg.initial_x_max:
        reasons.append("too far downrange")

    return HeuristicFeasibility(ok=not reasons, reasons=tuple(reasons))
