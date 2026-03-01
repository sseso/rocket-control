"""
scripts/attitude_sim.py
-------------------
Attitude control (rotation) simulation entry point.

Run:
    python attitude_sim.py --name my_run --mode dual
    modes: dual, rotation, translation
"""

import os
import argparse

from rocket_control.attitude.config    import AttitudeConfig
from rocket_control.attitude.physics   import wrap_angle, check_controllability
from rocket_control.attitude.simulator import run_simulation
from rocket_control.attitude.plots     import plot_diagnostics
from rocket_control.attitude.animation import animate_attitude

os.makedirs("results", exist_ok=True)


def get_args():
    p = argparse.ArgumentParser(description="Attitude control simulation")
    p.add_argument("--name",  default=None,
                   help="Output filename stem (default: attitude_<mode>)")
    p.add_argument("--mode",  default=None,
                   choices=["rotation", "translation", "dual"],
                   help="Animation mode (overrides interactive prompt)")
    return p.parse_args()


def main():
    args   = args = get_args()
    config = AttitudeConfig()

    # ---- User inputs ----
    theta_0      = wrap_angle(float(input("Initial angle (deg): ")))
    theta_target = wrap_angle(float(input("Target angle (deg): ")))
    omega_0      = float(input("Initial angular velocity (deg/s): "))
    dry_mass     = float(input("Dry mass (kg): "))
    fuel_mass    = float(input("Fuel mass (kg): "))
    thrust_force = float(input("Thrust Force (N): "))
    v_e          = float(input("Exhaust velocity (m/s): "))

    # ---- Mode selection ----
    if args.mode:
        mode = args.mode
        include_translation = mode in ("translation", "dual")
        render_both         = mode == "dual"
        print(f"--> Mode: {mode}")
    else:
        render_both = False
        resp = input("Render both views side-by-side? (y/n): ").strip().lower()
        if resp in ('y', 'yes', '1', 'true'):
            render_both = True
            include_translation = True
            mode = "dual"
        else:
            resp = input("Include translation (full motion)? (y/n): ").strip().lower()
            include_translation = resp in ('y', 'yes', '1', 'true')
            mode = "translation" if include_translation else "rotation"
        print(f"--> Mode: {mode}")

    # ---- Output path ----
    stem = args.name if args.name else f"attitude_{mode}"
    output_path = f"results/{stem}.mp4"

    # ---- Controllability check ----
    effective_target, use_wrap = check_controllability(
        theta_0, theta_target, omega_0,
        dry_mass, fuel_mass, thrust_force, v_e, config
    )

    # ---- Simulation ----
    result = run_simulation(
        theta_0, theta_target, omega_0,
        dry_mass, fuel_mass, thrust_force, v_e,
        effective_target, use_wrap,
        include_translation, config
    )

    # ---- Diagnostic plots ----
    plot_diagnostics(result, theta_target, include_translation, config)

    # ---- Animation ----
    animate_attitude(
        result, theta_0, theta_target, omega_0,
        dry_mass, fuel_mass, config,
        mode=mode, output_path=output_path
    )


if __name__ == "__main__":
    main()
