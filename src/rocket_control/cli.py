"""Command-line entry: landing, attitude, and (later) grid experiments."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from rocket_control.core import InfeasibleError, default_vehicle


def _add_verbose(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--verbose",
        action="store_true",
        help="print IPOPT / solver internals (quiet by default)",
    )


def _add_plots(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--plots",
        nargs="?",
        const="",
        default=None,
        metavar="PNG",
        help="diagnostic time-series figure: omit PNG to open a window, or pass a path to save",
    )


def _sub(parser: argparse.ArgumentParser, name: str, help_text: str) -> argparse.ArgumentParser:
    return parser.add_parser(
        name,
        help=help_text,
        description=help_text,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rocket-control",
        description="2D rocket landing (direct collocation) and vacuum attitude demo.",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    land = _sub(sub, "landing", "Time-optimal 2D landing NLP (trapezoidal collocation + IPOPT).")
    land.add_argument(
        "-o", "--output",
        default="results/landing.mp4",
        metavar="MP4",
        help="animation path (default: results/landing.mp4); ignored with --no-anim",
    )
    land.add_argument(
        "--x",
        type=float,
        default=None,
        metavar="M",
        help="initial horizontal offset from the landing target [m] (state x; default 30)",
    )
    land.add_argument(
        "--alt",
        type=float,
        default=None,
        metavar="M",
        help="initial nozzle height above ground [m] (default 160.42); state y = alt + d_com",
    )
    land.add_argument(
        "--vx",
        type=float,
        default=None,
        metavar="M/S",
        help="initial horizontal velocity [m/s] (default -8)",
    )
    land.add_argument(
        "--vy",
        type=float,
        default=None,
        metavar="M/S",
        help="initial vertical velocity [m/s], negative is downward (default -30)",
    )
    land.add_argument(
        "--theta-deg",
        type=float,
        default=None,
        metavar="DEG",
        help="initial pitch from vertical [deg] (default 0); converted to rad internally",
    )
    land.add_argument(
        "--omega-deg",
        type=float,
        default=None,
        metavar="DEG/S",
        help="initial pitch rate [deg/s] (default 0); converted to rad/s internally",
    )
    land.add_argument(
        "--no-anim",
        action="store_true",
        help="skip writing the mp4 (still solves and prints the landing report)",
    )
    _add_plots(land)
    _add_verbose(land)

    att = _sub(
        sub,
        "attitude",
        "Closed-loop vacuum attitude slew (bang-bang gimbal; not an NLP).",
    )
    att.add_argument(
        "-o", "--output",
        default=None,
        metavar="MP4",
        help="animation path (default: results/attitude_<mode>.mp4); ignored with --no-anim",
    )
    att.add_argument(
        "--mode",
        choices=["rotation", "translation", "dual"],
        default="rotation",
        help="rotation: body-fixed view; translation: free motion in vacuum; dual: both",
    )
    att.add_argument(
        "--theta0-deg",
        type=float,
        default=20.0,
        metavar="DEG",
        help="initial pitch [deg] (default 20)",
    )
    att.add_argument(
        "--target-deg",
        type=float,
        default=0.0,
        metavar="DEG",
        help="target pitch [deg] (default 0)",
    )
    att.add_argument(
        "--omega0-deg",
        type=float,
        default=0.0,
        metavar="DEG/S",
        help="initial pitch rate [deg/s] (default 0)",
    )
    att.add_argument(
        "--dry-mass",
        type=float,
        default=None,
        metavar="KG",
        help="dry mass [kg] (default: shared VehicleSpec 1250)",
    )
    att.add_argument(
        "--fuel-mass",
        type=float,
        default=None,
        metavar="KG",
        help="fuel mass [kg] (default: shared VehicleSpec 500)",
    )
    att.add_argument(
        "--thrust",
        type=float,
        default=None,
        metavar="N",
        help="constant engine thrust [N] (default: VehicleSpec.T_max 50000)",
    )
    att.add_argument(
        "--isp",
        type=float,
        default=None,
        metavar="S",
        help="specific impulse [s] (default: 500)",
    )
    att.add_argument(
        "--no-anim",
        action="store_true",
        help="skip writing the mp4",
    )
    _add_plots(att)
    _add_verbose(att)

    grid = _sub(
        sub,
        "grid",
        "Empirical landing success map over a (x, nozzle-altitude) grid.",
    )
    grid.add_argument(
        "-o", "--output",
        default="results/success_grid.png",
        metavar="PNG",
        help="success-map figure (also writes a .npz)",
    )
    grid.add_argument(
        "--show",
        action="store_true",
        help="open the saved grid figure in a window after writing it",
    )
    _add_verbose(grid)
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
    if args.plots is not None:
        from rocket_control.viz import plot_landing_diagnostics

        save_path = args.plots or None
        plot_landing_diagnostics(traj, problem, show=save_path is None, save_path=save_path)
        if save_path:
            print(f"Wrote plots to {save_path}")
    if metrics.ok and not args.no_anim:
        from rocket_control.viz import animate_landing

        print(f"Writing animation to {args.output}")
        animate_landing(traj, problem, args.output, show=False)
    elif not metrics.ok:
        print("Animation skipped (landing quality outside tolerances).")
    return 0


def _cmd_attitude(args: argparse.Namespace) -> int:
    from dataclasses import replace

    from rocket_control.attitude import AttitudeSettings, plan_manoeuvre, run_simulation
    from rocket_control.attitude.angles import wrap_angle

    vehicle = default_vehicle()
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
    if args.plots is not None:
        from rocket_control.viz import plot_attitude_diagnostics

        save_path = args.plots or None
        plot_attitude_diagnostics(
            result,
            target,
            include_translation=include_translation,
            dt=settings.dt,
            show=save_path is None,
            save_path=save_path,
        )
        if save_path:
            print(f"Wrote plots to {save_path}")
    if not args.no_anim:
        from rocket_control.viz import animate_attitude

        out = args.output or str(Path("results") / f"attitude_{args.mode}.mp4")
        print(f"Writing animation to {out}")
        animate_attitude(
            result, theta0, target, omega0, vehicle, settings, mode=args.mode, output_path=out, show=False,
        )
    return 0


def _cmd_grid(args: argparse.Namespace) -> int:
    from rocket_control.landing import default_landing_problem
    from rocket_control.landing.grid import run_success_grid, save_grid_plot

    problem = default_landing_problem()
    print("Empirical landing success map (not a reachable set).")
    results = run_success_grid(
        problem,
        progress=print,
        verbose_nlp=args.verbose,
    )
    n_ok = sum(1 for r in results if r.landing_ok)
    print(f"Successes: {n_ok}/{len(results)}")
    save_grid_plot(results, problem, args.output, show=args.show)
    print(f"Wrote {args.output} and {Path(args.output).with_suffix('.npz')}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.cmd == "landing":
        return _cmd_landing(args)
    if args.cmd == "attitude":
        return _cmd_attitude(args)
    if args.cmd == "grid":
        return _cmd_grid(args)
    parser.error(f"unknown command {args.cmd}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
