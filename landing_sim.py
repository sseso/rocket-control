"""
landing_sim.py
------
Full 2D CasADi rocket landing simulation.

Edit config.py parameters to control simulation details.

Run:
    python landing_sim.py video_file_name
"""

import os
import numpy as np

from rocket_control.landing.config       import RocketConfig
from rocket_control.landing.physics      import calculate_com_and_I, approx_controllable
from rocket_control.landing.solver       import solve_optimal_landing
from rocket_control.landing.landing_check import evaluate_landing, print_landing_report
from rocket_control.landing.animation    import plot_diagnostics, animate_results

os.makedirs("results", exist_ok=True)


def main(output_name="landing"):
    config = RocketConfig()

    print("--- Time-Optimal Rocket Landing Solver ---")

    h_0  = config.initial_nozzle_altitude
    x_0  = config.initial_x_0
    vx_0 = config.initial_vx_0
    vy_0 = config.initial_vy_0

    initial_m   = config.m_dry + config.m_fuel
    initial_com, _ = calculate_com_and_I(initial_m, config)

    # Solver degeneracy guard
    if x_0 == 0.0:
        x_0 = 0.0001

    s0 = np.array([
        x_0,
        h_0 + initial_com,   # y = CoM altitude
        vx_0,
        vy_0,
        0.0,                  # theta
        0.0,                  # omega
        initial_m,
    ])

    print(f"\nInitial state:\n  {s0}")
    print(f"  Fuel mass   : {config.m_fuel:6.1f} kg")
    print(f"  Dry mass    : {config.m_dry:6.1f} kg")
    print(f"  Initial CoM : {initial_com:6.2f} m from nozzle")
    print(f"  Nozzle alt  : {h_0:6.1f} m")

    # 1. Quick physical feasibility check
    print("\nChecking approximate controllability...")
    if not approx_controllable(s0, config):
        print("--> Not controllable: insufficient fuel, altitude or thrust-to-weight ratio.")
        print("  Simulation / optimization will NOT be run.")
        print("  (Tip: try increasing m_fuel or reducing |vy_0| / altitude)")
        return

    print("--> Approximate check passed. Starting optimization...")

    # 2. Run NLP solver
    states, controls, tf, success = solve_optimal_landing(s0, config)

    if not success:
        print("\nOptimization did NOT converge to a solution.")
        print("  Most likely causes: infeasible constraints, poor initial guess, numerical issues")
        print("  Animation and detailed plots will NOT be shown.")
        return

    print("\nOptimization finished. Evaluating solution quality...")

    # 3. Diagnostic plots
    plot_diagnostics(states, controls, tf, config)

    # 4. Landing quality check
    LANDING_GOOD, metrics = evaluate_landing(states, controls, tf, config)
    print_landing_report(LANDING_GOOD, metrics, config)

    if LANDING_GOOD:
        print("Generating animation...\n")
        animate_results(states, controls, tf, config,
                    output_path=f"results/{output_name}.mp4")
    else:
        print("\nAnimation will NOT be generated.")


if __name__ == "__main__":
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "landing"
    main(output_name=name)