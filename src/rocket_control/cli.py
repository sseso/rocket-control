"""Command-line entry: landing, attitude, and (later) grid experiments."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from rocket_control.core import InfeasibleError, default_vehicle


def _add_common(p: argparse.ArgumentParser) -> None:
    p.add_argument("--show", action="store_true", help="display matplotlib windows")
    p.add_argument("--verbose", action="store_true")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rocket-control",
        description="2D rocket landing (direct collocation) and attitude demo.",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    land = sub.add_parser("landing", help="solve the time-optimal 2D landing NLP")
    land.add_argument("-o", "--output", default="results/landing.mp4")
    land.add_argument("--x", type=float, default=None, help="initial downrange x [m]")
    land.add_argument("--alt", type=float, default=None, help="initial nozzle altitude [m]")
    land.add_argument("--vx", type=float, default=None)
    land.add_argument("--vy", type=float, default=None)
    land.add_argument("--theta-deg", type=float, default=None, help="initial pitch [deg]")
    land.add_argument("--omega-deg", type=float, default=None, help="initial pitch rate [deg/s]")
    land.add_argument("--no-anim", action="store_true")
    land.add_argument("--no-plots", action="store_true")
    _add_common(land)

    att = sub.add_parser("attitude", help="closed-loop vacuum attitude manoeuvre (not an NLP)")
    att.add_argument("-o", "--output", default=None)
    att.add_argument("--mode", choices=["rotation", "translation", "dual"], default="rotation")
    att.add_argument("--theta0-deg", type=float, default=20.0)
    att.add_argument("--target-deg", type=float, default=0.0)
    att.add_argument("--omega0-deg", type=float, default=0.0)
    att.add_argument("--dry-mass", type=float, default=None)
    att.add_argument("--fuel-mass", type=float, default=None)
    att.add_argument("--thrust", type=float, default=None)
    att.add_argument("--isp", type=float, default=None)
    att.add_argument("--interactive", action="store_true")
    att.add_argument("--no-anim", action="store_true")
    att.add_argument("--no-plots", action="store_true")
    _add_common(att)

    grid = sub.add_parser("grid", help="empirical landing success map over a (x, h) grid")
    grid.add_argument("-o", "--output", default="results/success_grid.png")
    _add_common(grid)
    return parser


def _cmd_landing(args: argparse.Namespace) -> int:
    from dataclasses import replace

    from rocket_control.landing import (
        default_landing_problem,
        evaluate_landing,
        heuristic_feasible,
        solve_optimal_landing,
    )
    from rocket_control.landing.evaluate import format_landing_report
    from rocket_control.landing.scenarios import Scenario

    problem = default_landing_problem()
    sc = problem.scenario
    updates = {}
    if args.x is not None:
        updates["x0"] = args.x
    if args.alt is not None:
        updates["nozzle_altitude"] = args.alt
    if args.vx is not None:
        updates["vx0"] = args.vx
    if args.vy is not None:
        updates["vy0"] = args.vy
    if args.theta_deg is not None:
        updates["theta0"] = np.deg2rad(args.theta_deg)
    if args.omega_deg is not None:
        updates["omega0"] = np.deg2rad(args.omega_deg)
    if updates:
        sc = replace(sc, **updates)
        problem = replace(problem, scenario=sc)

    s0 = problem.initial_state()
    print("Time-optimal rocket landing solver")
    print(f"  initial state (SI, rad): {s0}")
    feas = heuristic_feasible(s0, problem)
    if not feas.ok:
        print("Heuristic feasibility check failed (not a reachable-set proof):")
        for r in feas.reasons:
            print(f"  - {r}")
        return 1
    print("Heuristic check passed. Solving NLP...")
    if np.any(np.abs(s0[[0, 2, 3]]) < problem.mesh.eps_state):
        print(f"  note: |x|,|vx|,|vy| < {problem.mesh.eps_state:g} will be regularised for IPOPT.")
    traj = solve_optimal_landing(s0, problem, verbose=args.verbose)
    if not traj.success:
        print("IPOPT did not report a converged solution.")
        return 2
    print(f"  tf = {traj.tf:.3f} s")
    metrics = evaluate_landing(traj, problem)
    print(format_landing_report(metrics))
    if not args.no_plots:
        from rocket_control.viz import plot_landing_diagnostics

        plot_landing_diagnostics(traj, problem, show=args.show)
    if metrics.ok and not args.no_anim:
        from rocket_control.viz import animate_landing

        print(f"Writing animation to {args.output}")
        animate_landing(traj, problem, args.output, show=args.show)
    elif not metrics.ok:
        print("Animation skipped (landing quality outside tolerances).")
    return 0


def _cmd_attitude(args: argparse.Namespace) -> int:
    from dataclasses import replace

    from rocket_control.attitude import AttitudeSettings, plan_manoeuvre, run_simulation
    from rocket_control.attitude.angles import wrap_angle

    vehicle = default_vehicle()
    if args.interactive:
        theta0 = wrap_angle(np.deg2rad(float(input("Initial angle (deg): "))))
        target = wrap_angle(np.deg2rad(float(input("Target angle (deg): "))))
        omega0 = np.deg2rad(float(input("Initial angular velocity (deg/s): ")))
        dry = float(input("Dry mass (kg): "))
        fuel = float(input("Fuel mass (kg): "))
        thrust = float(input("Thrust Force (N): "))
        isp = float(input("Specific Impulse (s): "))
        vehicle = replace(vehicle, m_dry=dry, m_fuel=fuel, I_sp=isp, T_max=max(thrust, vehicle.T_min))
    else:
        theta0 = wrap_angle(np.deg2rad(args.theta0_deg))
        target = wrap_angle(np.deg2rad(args.target_deg))
        omega0 = np.deg2rad(args.omega0_deg)
        thrust = args.thrust if args.thrust is not None else vehicle.T_max
        vehicle = replace(
            vehicle,
            m_dry=args.dry_mass if args.dry_mass is not None else vehicle.m_dry,
            m_fuel=args.fuel_mass if args.fuel_mass is not None else vehicle.m_fuel,
            I_sp=args.isp if args.isp is not None else vehicle.I_sp,
            T_max=max(thrust, vehicle.T_min),
        )

    settings = AttitudeSettings()
    include_translation = args.mode in ("translation", "dual")
    try:
        plan = plan_manoeuvre(theta0, target, omega0, thrust, vehicle, settings)
    except InfeasibleError as exc:
        print(f"Infeasible manoeuvre: {exc}")
        return 1
    print(f"Closed-loop attitude demo (not optimal control). mode={args.mode}")
    result = run_simulation(
        theta0, omega0, thrust, plan, vehicle, settings,
        include_translation=include_translation,
    )
    print(f"Settled at t = {result.final_time:.2f} s")
    if not args.no_plots:
        from rocket_control.viz import plot_attitude_diagnostics

        plot_attitude_diagnostics(
            result, target, include_translation=include_translation, dt=settings.dt, show=args.show,
        )
    if not args.no_anim:
        from rocket_control.viz import animate_attitude

        out = args.output or str(Path("results") / f"attitude_{args.mode}.mp4")
        print(f"Writing animation to {out}")
        animate_attitude(
            result, theta0, target, omega0, vehicle, settings, mode=args.mode, output_path=out, show=args.show,
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.cmd == "landing":
        return _cmd_landing(args)
    if args.cmd == "attitude":
        return _cmd_attitude(args)
    if args.cmd == "grid":
        print("The grid subcommand is added in the experiments pass.")
        return 1
    parser.error(f"unknown command {args.cmd}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
