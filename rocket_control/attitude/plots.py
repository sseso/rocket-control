"""
attitude/plots.py
-----------------
Diagnostic time-series plots for the attitude simulation.
"""

import matplotlib.pyplot as plt


def plot_diagnostics(result, theta_target, include_translation, config):
    """
    Multi-panel diagnostic plot.

    Parameters
    ----------
    result             : SimResult
    theta_target       : float - target angle (deg), for reference line
    include_translation: bool
    config             : AttitudeConfig
    """
    times = result.times
    t_end = result.final_time

    if include_translation:
        nrows  = 12
        fig_h  = 26
        data   = [result.angles, result.omegas, result.gimbals, result.masses,
                  result.thrusts, result.torques, result.coms, result.Is,
                  result.x_coms, result.y_coms, result.vxs, result.vys]
        labels = ['Angle (deg)', 'Ω (deg/s)', 'Gimbal (deg)', 'Mass (kg)',
                  'Thrust (N)', 'Torque (N·m)', 'CoM from nozzle (m)',
                  'Moment of Inertia (kg·m²)', 'X pos (m)', 'Y pos (m)',
                  'Vx (m/s)', 'Vy (m/s)']
        colors = ['cyan', 'lime', 'orange', 'red', 'blue', 'purple',
                  'magenta', 'gold', 'teal', 'navy', 'brown', 'olive']
        title  = "Post-Flight Analysis (Full 6DOF Simulation)"
    else:
        nrows  = 8
        fig_h  = 18
        data   = [result.angles, result.omegas, result.gimbals, result.masses,
                  result.thrusts, result.torques, result.coms, result.Is]
        labels = ['Angle (deg)', 'Ω (deg/s)', 'Gimbal (deg)', 'Mass (kg)',
                  'Thrust (N)', 'Torque (N·m)', 'CoM from nozzle (m)',
                  'Moment of Inertia (kg·m²)']
        colors = ['cyan', 'lime', 'orange', 'red', 'blue', 'purple', 'magenta', 'gold']
        title  = "Post-Flight Analysis"

    fig, axs = plt.subplots(nrows, 1, figsize=(10, fig_h), sharex=True)
    plt.subplots_adjust(hspace=0.28)

    for i, ax in enumerate(axs):
        ax.plot(times, data[i], color=colors[i])
        ax.set_ylabel(labels[i])
        ax.set_facecolor("#ffffff")
        ax.grid(True, alpha=0.18)
        ax.set_xlim(-0.4, times[-1] + 0.6)
        ax.axvline(t_end - config.dt, color="red", linestyle="--", alpha=0.5, lw=1.2)

    axs[0].axhline(theta_target, color='black', linestyle='--', alpha=0.6, label="target")
    axs[0].legend(fontsize=9)

    axs[7 if not include_translation else 7].axhline(
        result.Is[0], color='darkgreen', ls='--', alpha=0.5,
        label=f"initial I = {result.Is[0]:.0f}")
    if len(result.Is) > 1:
        axs[7].axhline(
            result.Is[-1], color='darkred', ls='--', alpha=0.5,
            label=f"final I = {result.Is[-1]:.0f}")
    axs[7].legend(fontsize=9, loc='upper right')

    axs[-1].set_xlabel("Time (s)")
    plt.suptitle(title, fontsize=16, y=0.995)
    plt.show()
