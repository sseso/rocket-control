"""Slew-limited bang-bang gimbal law."""

from __future__ import annotations

import numpy as np

from .angles import wrap_angle
from .config import AttitudeSettings


def max_pitch_accel(thrust: float, lever_arm: float, inertia: float, gimbal_limit: float) -> float:
    """Maximum |omega-dot| at full gimbal (rad/s^2)."""
    if inertia <= 0:
        return 0.0
    return abs(thrust * np.sin(gimbal_limit) * lever_arm / inertia)


def gimbal_command(
    theta: float,
    omega: float,
    target: float,
    max_alpha: float,
    settings: AttitudeSettings,
    *,
    use_wrap: bool,
) -> tuple[float, bool]:
    """Return (target gimbal [rad], engine_on)."""
    error = wrap_angle(target - theta) if use_wrap else (target - theta)
    if abs(error) < settings.settle_pos_tol and abs(omega) < settings.settle_vel_tol:
        return 0.0, False

    target_omega = np.sign(error) * np.sqrt(2.0 * max_alpha * abs(error)) if max_alpha > 0 else 0.0
    switch_error = target_omega - omega
    if abs(switch_error) > settings.boundary:
        cmd = -np.sign(switch_error) * settings.gimbal_limit
    else:
        cmd = -(switch_error / settings.boundary) * settings.gimbal_limit
    return float(cmd), True


def slew_gimbal(current: float, target: float, settings: AttitudeSettings) -> float:
    err = target - current
    if abs(err) <= 0.01 * np.pi / 180.0:
        return current
    step = np.sign(err) * settings.max_gimbal_speed * settings.dt
    return float(current + np.clip(step, -abs(err), abs(err)))
