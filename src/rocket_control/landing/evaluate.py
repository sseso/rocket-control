"""Post-solve landing quality, independent of the NLP solver."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from rocket_control.core import IM, IVX, IVY, IX, IY
from rocket_control.core.mass_properties import com_and_inertia, com_and_inertia_array

from .scenarios import LandingProblem
from .types import Trajectory


@dataclass(frozen=True)
class LandingMetrics:
    ok: bool
    error_x: float
    error_y: float
    final_pos_err: float
    final_vel_err: float
    min_nozzle: float
    skidding_detected: bool
    issues: tuple[str, ...]


def evaluate_landing(traj: Trajectory, problem: LandingProblem) -> LandingMetrics:
    states = traj.states
    vehicle = problem.vehicle
    tol = problem.tolerances
    coms, _ = com_and_inertia_array(states[:, IM], vehicle)
    y_nozzle = states[:, IY] - coms
    com_term, _ = com_and_inertia(states[-1, IM], vehicle)
    error_x = float(states[-1, IX])
    error_y = float(states[-1, IY] - com_term)
    final_pos_err = float(np.hypot(error_x, error_y))
    final_vel_err = float(np.linalg.norm(states[-1, [IVX, IVY]]))
    min_nozzle = float(np.min(y_nozzle))
    skidding = bool(
        np.any((y_nozzle[:-1] < 0.1) & (np.abs(states[:-1, IX]) > tol.skidding_x_threshold))
    )
    issues: List[str] = []
    if abs(error_x) > tol.landing_pos_tol:
        issues.append(f"X-position error too high: {error_x:.3f} m")
    if abs(error_y) > tol.landing_pos_tol:
        issues.append(f"Y-position error too high: {error_y:.3f} m")
    if final_pos_err >= tol.landing_pos_tol:
        issues.append(f"pos error = {final_pos_err:.3f} m")
    if final_vel_err >= tol.landing_vel_tol:
        issues.append(f"vel error = {final_vel_err:.3f} m/s")
    if min_nozzle < tol.min_nozzle_tol:
        issues.append(f"ground clip = {abs(min_nozzle):.3f} m")
    if skidding:
        issues.append(
            f"unrealistic skidding detected while |x| > {tol.skidding_x_threshold} m"
        )
    ok = (
        final_pos_err < tol.landing_pos_tol
        and final_vel_err < tol.landing_vel_tol
        and min_nozzle >= tol.min_nozzle_tol
        and not skidding
    )
    return LandingMetrics(
        ok=ok,
        error_x=error_x,
        error_y=error_y,
        final_pos_err=final_pos_err,
        final_vel_err=final_vel_err,
        min_nozzle=min_nozzle,
        skidding_detected=skidding,
        issues=tuple(issues),
    )


def format_landing_report(metrics: LandingMetrics) -> str:
    if metrics.ok:
        return (
            "Landing looks good within tolerances!\n"
            f"  X-offset:      {metrics.error_x:6.3f} m\n"
            f"  Y-offset:      {metrics.error_y:6.3f} m  (+ = floating, - = sunk)\n"
            f"  Velocity err:  {metrics.final_vel_err:6.3f} m/s\n"
            f"  Min nozzle:    {metrics.min_nozzle:6.3f} m"
        )
    lines = ["Solution found, but landing quality is NOT acceptable:"]
    lines.extend(f"  - {issue}" for issue in metrics.issues)
    return "\n".join(lines)
