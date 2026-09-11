"""Time-series diagnostic plots."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from rocket_control.attitude.simulate import AttitudeResult
from rocket_control.core.mass_properties import com_and_inertia_array
from rocket_control.landing.scenarios import LandingProblem
from rocket_control.landing.types import Trajectory


def plot_landing_diagnostics(
    traj: Trajectory,
    problem: LandingProblem,
    *,
    show: bool = True,
    save_path: str | Path | None = None,
) -> None:
    import matplotlib.pyplot as plt

    states, controls, tf = traj.states, traj.controls, traj.tf
    times = traj.times
    masses = states[:, 6]
    coms, i_zs = com_and_inertia_array(masses, problem.vehicle)
    torques = coms * controls[:, 0] * np.sin(controls[:, 1])

    plot_data = [
        np.rad2deg(states[:, 4]),
        np.rad2deg(states[:, 5]),
        np.rad2deg(controls[:, 1]),
        masses,
        controls[:, 0],
        torques,
        coms,
        i_zs,
        states[:, 0],
        states[:, 1] - coms,
        states[:, 2],
        states[:, 3],
    ]
    labels = [
        r"$\theta$ (deg)", r"$\omega$ (deg/s)", "Gimbal (deg)", "Mass (kg)",
        "Thrust (N)", r"Torque (N$\cdot$m)", "CoM from nozzle (m)", r"MoI (kg$\cdot \text{m}^2$)",
        "X position (m)", "Nozzle height (m)", "Vx (m/s)", "Vy (m/s)",
    ]
    colors = [
        "cyan", "lime", "orange", "red", "blue", "purple",
        "magenta", "gold", "teal", "navy", "brown", "olive",
    ]
    fig, axs = plt.subplots(6, 2, figsize=(14, 9), sharex=True, constrained_layout=True)
    for i, ax in enumerate(axs.flat):
        ax.plot(times, plot_data[i], color=colors[i], lw=1.4)
        ax.set_ylabel(labels[i], fontsize=10)
        ax.grid(True, alpha=0.18, ls="--")
    nozzle_ax = axs.flat[9]
    nozzle_ax.axhspan(-5, 0, facecolor="gray", alpha=0.15)
    nozzle_ax.axhline(0, color="darkred", lw=1.2, ls="--", alpha=0.7)
    for ax in axs[-1, :]:
        ax.set_xlabel("Time (s)", fontsize=11)
    fig.suptitle("Post-Flight Analysis", fontsize=16)
    if save_path:
        fig.savefig(save_path, dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_attitude_diagnostics(
    result: AttitudeResult,
    theta_target: float,
    *,
    include_translation: bool,
    dt: float,
    show: bool = True,
    save_path: str | Path | None = None,
) -> None:
    import matplotlib.pyplot as plt

    times = result.times
    t_end = result.final_time
    if include_translation:
        data = [
            np.rad2deg(result.theta), np.rad2deg(result.omega), np.rad2deg(result.gimbal),
            result.mass, result.thrust, result.torque, result.com, result.inertia,
            result.x, result.y, result.vx, result.vy,
        ]
        labels = [
            "Angle (deg)", r"$\Omega$ (deg/s)", "Gimbal (deg)", "Mass (kg)",
            "Thrust (N)", r"Torque (N$\cdot$m)", "CoM from nozzle (m)", r"MoI (kg$\cdot \text{m}^2$)",
            "X pos (m)", "Y pos (m)", "Vx (m/s)", "Vy (m/s)",
        ]
        colors = [
            "cyan", "lime", "orange", "red", "blue", "purple",
            "magenta", "gold", "teal", "navy", "brown", "olive",
        ]
        title = "Post-Flight Analysis (vacuum translation enabled)"
        fig_h = 26
    else:
        data = [
            np.rad2deg(result.theta), np.rad2deg(result.omega), np.rad2deg(result.gimbal),
            result.mass, result.thrust, result.torque, result.com, result.inertia,
        ]
        labels = [
            "Angle (deg)", r"$\Omega$ (deg/s)", "Gimbal (deg)", "Mass (kg)",
            "Thrust (N)", r"Torque (N$\cdot$m)", "CoM from nozzle (m)", r"MoI (kg$\cdot \text{m}^2$)",
        ]
        colors = ["cyan", "lime", "orange", "red", "blue", "purple", "magenta", "gold"]
        title = "Post-Flight Analysis (rotation)"
        fig_h = 18

    fig, axs = plt.subplots(len(data), 1, figsize=(10, fig_h), sharex=True)
    plt.subplots_adjust(hspace=0.28)
    for i, ax in enumerate(axs):
        ax.plot(times, data[i], color=colors[i])
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.18)
        ax.set_xlim(-0.4, times[-1] + 0.6)
        ax.axvline(t_end - dt, color="red", linestyle="--", alpha=0.5, lw=1.2)
    axs[0].axhline(np.rad2deg(theta_target), color="black", linestyle="--", alpha=0.6, label="target")
    axs[0].legend(fontsize=9)
    axs[7].axhline(result.inertia[0], color="darkgreen", ls="--", alpha=0.5,
                   label=f"initial I = {result.inertia[0]:.0f}")
    axs[7].axhline(result.inertia[-1], color="darkred", ls="--", alpha=0.5,
                   label=f"final I = {result.inertia[-1]:.0f}")
    axs[7].legend(fontsize=9, loc="upper right")
    axs[-1].set_xlabel("Time (s)")
    plt.suptitle(title, fontsize=16, y=0.995)
    if save_path:
        fig.savefig(save_path, dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)
