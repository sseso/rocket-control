"""Euler integration of the shared plant with the bang-bang gimbal law."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from rocket_control.core import IM, IOMEGA, ITHETA, IVX, IVY, IX, IY, VehicleSpec
from rocket_control.core.dynamics import rocket_dynamics
from rocket_control.core.mass_properties import com_and_inertia

from .angles import wrap_angle
from .config import AttitudeSettings
from .controller import gimbal_command, max_pitch_accel, slew_gimbal
from .feasibility import ManoeuvrePlan


@dataclass
class AttitudeResult:
    """Time series in SI (radians)."""

    times: NDArray[np.float64]
    theta: NDArray[np.float64]
    theta_unwrapped: NDArray[np.float64]
    omega: NDArray[np.float64]
    gimbal: NDArray[np.float64]
    mass: NDArray[np.float64]
    thrust: NDArray[np.float64]
    torque: NDArray[np.float64]
    com: NDArray[np.float64]
    inertia: NDArray[np.float64]
    x: NDArray[np.float64]
    y: NDArray[np.float64]
    vx: NDArray[np.float64]
    vy: NDArray[np.float64]
    final_time: float


def run_simulation(
    theta0: float,
    omega0: float,
    thrust: float,
    plan: ManoeuvrePlan,
    vehicle: VehicleSpec,
    settings: AttitudeSettings,
    *,
    include_translation: bool,
) -> AttitudeResult:
    dt = settings.dt
    state = np.array(
        [0.0, 0.0, 0.0, 0.0, theta0, omega0, vehicle.m0],
        dtype=float,
    )
    gimbal = 0.0
    t = 0.0
    settled = False
    running = True

    logs: dict[str, list[float]] = {k: [] for k in (
        "t", "th", "th_u", "om", "g", "m", "T", "tau", "com", "I", "x", "y", "vx", "vy"
    )}

    def _record(st: np.ndarray, T: float, alpha: float, tau: float, d_com: float, i_z: float) -> None:
        logs["t"].append(t)
        logs["th"].append(wrap_angle(st[ITHETA]))
        logs["th_u"].append(st[ITHETA])
        logs["om"].append(st[IOMEGA])
        logs["g"].append(alpha)
        logs["m"].append(st[IM])
        logs["T"].append(T)
        logs["tau"].append(tau)
        logs["com"].append(d_com)
        logs["I"].append(i_z)
        logs["x"].append(st[IX])
        logs["y"].append(st[IY])
        logs["vx"].append(st[IVX])
        logs["vy"].append(st[IVY])

    while running and t < settings.max_time:
        d_com, i_z = com_and_inertia(state[IM], vehicle)
        max_a = max_pitch_accel(thrust, d_com, i_z, settings.gimbal_limit)
        cmd, engine_on = gimbal_command(
            state[ITHETA], state[IOMEGA], plan.effective_target, max_a, settings,
            use_wrap=plan.use_wrap,
        )
        if not engine_on:
            state[IOMEGA] = 0.0
            settled = True
        gimbal = slew_gimbal(gimbal, cmd, settings)
        T = thrust if engine_on else 0.0
        control = np.array([T, gimbal], dtype=float)

        xdot = rocket_dynamics(state, control, vehicle, gravity=False)
        if not include_translation:
            xdot[IX] = xdot[IY] = xdot[IVX] = xdot[IVY] = 0.0
        if state[IM] <= vehicle.m_dry + 1e-9:
            xdot[IM] = 0.0
            xdot[IVX] = 0.0
            xdot[IVY] = 0.0
            xdot[IOMEGA] = 0.0

        tau = -(d_com * T / i_z) * np.sin(gimbal) * i_z  # = -d_com T sin(alpha)
        _record(state, T, gimbal, tau, d_com, i_z)

        state = state + dt * xdot
        state[IM] = max(state[IM], vehicle.m_dry)
        t += dt
        if (settled and abs(gimbal) < np.deg2rad(0.01)) or state[IM] <= vehicle.m_dry:
            running = False

    final_time = t
    hold_steps = int(settings.post_hold / dt)
    d_com, i_z = com_and_inertia(state[IM], vehicle)
    for _ in range(hold_steps):
        t += dt
        logs["t"].append(t)
        logs["th"].append(wrap_angle(state[ITHETA]))
        logs["th_u"].append(state[ITHETA])
        logs["om"].append(0.0)
        logs["g"].append(0.0)
        logs["m"].append(state[IM])
        logs["T"].append(0.0)
        logs["tau"].append(0.0)
        logs["com"].append(d_com)
        logs["I"].append(i_z)
        logs["x"].append(state[IX])
        logs["y"].append(state[IY])
        logs["vx"].append(state[IVX])
        logs["vy"].append(state[IVY])

    def _a(key: str) -> np.ndarray:
        return np.asarray(logs[key], dtype=float)

    return AttitudeResult(
        times=_a("t"),
        theta=_a("th"),
        theta_unwrapped=_a("th_u"),
        omega=_a("om"),
        gimbal=_a("g"),
        mass=_a("m"),
        thrust=_a("T"),
        torque=_a("tau"),
        com=_a("com"),
        inertia=_a("I"),
        x=_a("x"),
        y=_a("y"),
        vx=_a("vx"),
        vy=_a("vy"),
        final_time=final_time,
    )
