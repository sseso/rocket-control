"""Empirical landing success map over a (x, nozzle-altitude) grid.

This is not a reachable-set computation. Each cell is: heuristic check,
then IPOPT, then the same landing tolerances as a single solve.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

from rocket_control.core.mass_properties import com_and_inertia
from rocket_control.landing.evaluate import evaluate_landing
from rocket_control.landing.feasibility import heuristic_feasible
from rocket_control.landing.nlp import solve_optimal_landing
from rocket_control.landing.scenarios import GridSpec, LandingProblem, Scenario


@dataclass(frozen=True)
class GridPointResult:
    x: float
    nozzle_altitude: float
    heuristic_ok: bool
    solved: bool
    landing_ok: bool


def grid_axes(spec: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    xs = np.arange(spec.x_min, spec.x_max + spec.x_step / 2.0, spec.x_step)
    hs = np.arange(spec.h_min, spec.h_max + spec.h_step / 2.0, spec.h_step)
    return xs, hs


def run_success_grid(
    problem: LandingProblem,
    *,
    progress: Callable[[str], None] | None = None,
    verbose_nlp: bool = False,
) -> list[GridPointResult]:
    spec = problem.grid
    xs, hs = grid_axes(spec)
    total = len(xs) * len(hs)
    current = 0
    out: list[GridPointResult] = []
    m0 = problem.vehicle.m0
    d_com, _ = com_and_inertia(m0, problem.vehicle)

    for x in xs:
        for h in hs:
            current += 1
            if progress:
                progress(f"[{current}/{total}] x={x:.1f} m, h={h:.1f} m")
            sc = Scenario(
                nozzle_altitude=float(h),
                x0=float(x),
                vx0=spec.vx0,
                vy0=spec.vy0,
                theta0=spec.theta0,
                omega0=spec.omega0,
            )
            s0 = np.array(
                [sc.x0, sc.nozzle_altitude + d_com, sc.vx0, sc.vy0, sc.theta0, sc.omega0, m0],
                dtype=float,
            )
            feas = heuristic_feasible(s0, problem)
            if not feas.ok:
                out.append(GridPointResult(float(x), float(h), False, False, False))
                continue
            traj = solve_optimal_landing(s0, problem, verbose=verbose_nlp)
            if not traj.success:
                out.append(GridPointResult(float(x), float(h), True, False, False))
                continue
            metrics = evaluate_landing(traj, problem)
            out.append(GridPointResult(float(x), float(h), True, True, metrics.ok))
    return out


def save_grid_plot(
    results: Sequence[GridPointResult],
    problem: LandingProblem,
    output_path: str | Path,
    *,
    show: bool = False,
) -> None:
    import matplotlib.pyplot as plt

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    spec = problem.grid
    fig, ax = plt.subplots(figsize=(12, 8))
    ok = [(r.x, r.nozzle_altitude) for r in results if r.landing_ok]
    bad = [(r.x, r.nozzle_altitude) for r in results if not r.landing_ok]
    if ok:
        sx, sh = zip(*ok)
        ax.plot(sx, sh, "go", markersize=8, label="Success")
    if bad:
        fx, fh = zip(*bad)
        ax.plot(fx, fh, "rx", markersize=8, label="Failure")
    ax.set_xlabel("Initial x (m)")
    ax.set_ylabel("Initial nozzle altitude (m)")
    ax.set_title("Empirical landing success map\n(IPOPT + tolerances; not a reachable set)")
    ax.grid(True)
    ax.legend(loc="upper left", fontsize=10)
    v = problem.vehicle
    d_com, _ = com_and_inertia(v.m0, v)
    textstr = (
        "Fixed initial conditions (same for all points):\n\n"
        rf"  $v_{{x0}}$ = {spec.vx0:>+6.1f} m/s" + "\n"
        rf"  $v_{{y0}}$ = {spec.vy0:>+6.1f} m/s" + "\n"
        rf"  $\theta_0$ = {spec.theta0:>+6.3f} rad" + "\n"
        rf"  $\omega_0$ = {spec.omega0:>+6.3f} rad/s" + "\n"
        f"  Dry mass  = {v.m_dry:>6.0f} kg\n"
        f"  Fuel mass = {v.m_fuel:>6.0f} kg\n"
        f"  CoM       = {d_com:>6.2f} m from nozzle\n"
        "\n"
        rf"Grid: $x_0 \in$ [{spec.x_min}, {spec.x_max}] m (step {spec.x_step} m)" + "\n"
        rf"      $h_0 \in$ [{spec.h_min}, {spec.h_max}] m (step {spec.h_step} m)"
    )
    ax.text(
        1.05, 0.98, textstr, transform=ax.transAxes, fontsize=10, family="monospace",
        va="top", ha="left",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="gray"),
    )
    plt.subplots_adjust(right=0.72)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    npz = output_path.with_suffix(".npz")
    np.savez(
        npz,
        x=np.array([r.x for r in results]),
        h=np.array([r.nozzle_altitude for r in results]),
        heuristic_ok=np.array([r.heuristic_ok for r in results]),
        solved=np.array([r.solved for r in results]),
        landing_ok=np.array([r.landing_ok for r in results]),
    )
    if show:
        plt.show()
    else:
        plt.close(fig)
