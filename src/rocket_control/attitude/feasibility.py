"""Fuel / path-length check for the bang-bang attitude manoeuvre."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rocket_control.core import InfeasibleError, VehicleSpec
from rocket_control.core.mass_properties import com_and_inertia

from .angles import wrap_angle
from .config import AttitudeSettings
from .controller import max_pitch_accel


@dataclass(frozen=True)
class ManoeuvrePlan:
    effective_target: float
    use_wrap: bool


def plan_manoeuvre(
    theta0: float,
    theta_target: float,
    omega0: float,
    thrust: float,
    vehicle: VehicleSpec,
    settings: AttitudeSettings,
) -> ManoeuvrePlan:
    d_com, i_z = com_and_inertia(vehicle.m0, vehicle)
    max_alpha = max_pitch_accel(thrust, d_com, i_z, settings.gimbal_limit)
    burn_rate = thrust / vehicle.v_e if vehicle.v_e > 0 else 0.0

    if max_alpha <= 0:
        raise InfeasibleError("zero angular acceleration: check thrust, gimbal, and inertia")

    stop_time = abs(omega0) / max_alpha
    if burn_rate * stop_time > vehicle.m_fuel:
        raise InfeasibleError("not enough fuel to stop the initial rotation")

    short_error = wrap_angle(theta_target - theta0)
    stopping_dist = (omega0**2) / (2.0 * max_alpha)
    use_wrap = True
    effective_target = theta_target
    if np.sign(omega0) != np.sign(short_error) and stopping_dist > abs(short_error):
        use_wrap = False
        effective_target = theta0 + (short_error - 2.0 * np.pi * np.sign(short_error))

    effective_error = abs(effective_target - theta0)
    maneuver_time = 2.0 * np.sqrt(effective_error / max_alpha)
    fuel_maneuver = burn_rate * maneuver_time
    fuel_stop = 0.0
    if np.sign(omega0) != np.sign(effective_target - theta0):
        fuel_stop = burn_rate * abs(omega0) / max_alpha
    if fuel_maneuver + fuel_stop > vehicle.m_fuel:
        raise InfeasibleError("not enough fuel to reach the target attitude")

    return ManoeuvrePlan(effective_target=float(effective_target), use_wrap=use_wrap)
