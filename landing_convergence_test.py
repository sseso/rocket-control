"""
landing_convergence_test.py
-------
Convergence & Reachable Set grid test.

Edit config_test.py parameters to control simulation details.

Run:
    python landing_convergence_test.py output_file_name
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

try:
    from IPython.display import clear_output
    IN_NOTEBOOK = True
except ImportError:
    IN_NOTEBOOK = False

from rocket_control.landing.config_test   import RocketConfigTest
from rocket_control.landing.physics       import calculate_com_and_I, approx_controllable
from rocket_control.landing.solver        import solve_optimal_landing
from rocket_control.landing.landing_check import evaluate_landing, print_landing_report

os.makedirs("results", exist_ok=True)


def main(output_name="test_results"):
    config = RocketConfigTest()

    print("--- Time-Optimal Rocket Landing Solver (Grid Test) ---")

    vx_0 = config.test_vx_0
    vy_0 = config.test_vy_0

    initial_m      = config.m_dry + config.m_fuel
    initial_com, _ = calculate_com_and_I(initial_m, config)

    print(f"\nFixed initial conditions for all grid points:")
    print(f"  Vx: {vx_0} m/s")
    print(f"  Vy: {vy_0} m/s")
    print(f"  Theta: {config.test_theta_0} rad")
    print(f"  Omega: {config.test_omega_0} rad/s")
    print(f"  Fuel mass   : {config.m_fuel:6.1f} kg")
    print(f"  Dry mass    : {config.m_dry:6.1f} kg")
    print(f"  Initial CoM : {initial_com:6.2f} m from nozzle")

    x_grid = np.arange(config.x_grid_min,
                        config.x_grid_max + config.x_grid_step / 2,
                        config.x_grid_step)
    h_grid = np.arange(config.h_grid_min,
                        config.h_grid_max + config.h_grid_step / 2,
                        config.h_grid_step)

    total = len(x_grid) * len(h_grid)
    current = 0
    results = []

    print(f"\nTotal simulations to run: {total}\n")

    for x_0_orig in x_grid:
        for h_0 in h_grid:
            current += 1
            pct = (current / total) * 100
            prog = f"Progress: [{current}/{total}] {pct:5.1f}%"

            if IN_NOTEBOOK:
                clear_output(wait=True)
                print(prog)
            else:
                sys.stdout.write(f"\r{prog}")
                sys.stdout.flush()

            print(f"\nTesting initial position: x={x_0_orig:.1f}, nozzle_alt={h_0:.1f} m")
            print("")

            # Degeneracy guards
            x_0    = 0.0001 if abs(x_0_orig) < 1e-6 else x_0_orig
            vx_eff = 0.0001 if abs(vx_0)     < 1e-6 else vx_0
            vy_eff = 0.0001 if abs(vy_0)     < 1e-6 else vy_0

            s0 = np.array([
                x_0,
                h_0 + initial_com,
                vx_eff,
                vy_eff,
                config.test_theta_0,
                config.test_omega_0,
                initial_m,
            ])

            print("Checking approximate controllability...")
            if not approx_controllable(s0, config):
                print("--> Not controllable.")
                results.append((x_0_orig, h_0, False))
                continue

            print("--> Approximate check passed. Starting optimization...")
            states, controls, tf, success = solve_optimal_landing(s0, config)

            if not success:
                print("\nOptimization did NOT converge.")
                results.append((x_0_orig, h_0, False))
                continue

            print("\nOptimization finished. Evaluating solution quality...")
            LANDING_GOOD, metrics = evaluate_landing(states, controls, tf, config)
            print_landing_report(LANDING_GOOD, metrics, config)
            results.append((x_0_orig, h_0, LANDING_GOOD))

    print("\n\nAll simulations completed.")
    print("Generating success grid plot...")

    # ---- Success grid plot ----
    fig, ax = plt.subplots(figsize=(12, 8))

    success_points = [(x, h) for x, h, ok in results if ok]
    failure_points = [(x, h) for x, h, ok in results if not ok]

    if success_points:
        sx, sh = zip(*success_points)
        ax.plot(sx, sh, 'go', markersize=8, label='Success')
    if failure_points:
        fx, fh = zip(*failure_points)
        ax.plot(fx, fh, 'rx', markersize=8, label='Failure')

    ax.set_xlabel('Initial x (m)')
    ax.set_ylabel('Initial Nozzle Altitude (m)')
    ax.set_title('Landing Success Grid\n(fixed velocity & attitude conditions)')
    ax.grid(True)
    ax.legend(loc='upper left', fontsize=10)

    textstr = (
        "Fixed initial conditions (same for all points):\n\n"
        rf"  Horizontal velocity   $Vx_0$" + f" = {vx_0:>+6.1f} m/s\n"
        fr"  Vertical velocity     $Vy_0$" + f" = {vy_0:>+6.1f} m/s\n"
        fr"  Pitch angle            $\theta_0$" + f" = {config.test_theta_0:>+6.1f} rad\n"
        fr"  Pitch rate             $\omega_0$" + f" = {config.test_omega_0:>+6.1f} rad/s\n"
        f"  Dry mass                  = {config.m_dry:>6.0f} kg\n"
        f"  Fuel mass                 = {config.m_fuel:>6.0f} kg\n"
        f"  Total initial mass        = {initial_m:>6.0f} kg\n"
        f"  Initial CoM from nozzle   =  {initial_com:>5.2f} m\n"
        "\n"
        rf"Grid: $x_0$ $\in$ [{config.x_grid_min}, {config.x_grid_max}] m  (step {config.x_grid_step} m)" + "\n"
        rf"      $h_0$ $\in$ [{config.h_grid_min}, {config.h_grid_max}]  m  (step {config.h_grid_step} m) (nozzle)"
    )

    props = dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='gray')
    ax.text(1.05, 0.98, textstr,
            transform=ax.transAxes,
            fontsize=10, family='monospace',
            verticalalignment='top', horizontalalignment='left',
            bbox=props)

    plt.subplots_adjust(right=0.72)
    plt.savefig(f"results/{output_name}.png", dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Grid plot saved to results/{output_name}.png")


if __name__ == "__main__":
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "test_results"
    main(output_name=name)
