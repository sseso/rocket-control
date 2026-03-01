"""
attitude/simulator.py
---------------------
Time-stepping simulation loop for the attitude control case.
Returns a SimResult with all recorded trajectories.
"""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from .physics import calculate_com_and_I, wrap_angle


@dataclass
class SimResult:
    """All time-series data produced by run_simulation."""
    times:            List[float] = field(default_factory=list)
    angles:           List[float] = field(default_factory=list)   # wrapped (deg)
    unwrapped_angles: List[float] = field(default_factory=list)   # raw (deg)
    omegas:           List[float] = field(default_factory=list)
    gimbals:          List[float] = field(default_factory=list)
    masses:           List[float] = field(default_factory=list)
    thrusts:          List[float] = field(default_factory=list)
    torques:          List[float] = field(default_factory=list)
    coms:             List[float] = field(default_factory=list)
    Is:               List[float] = field(default_factory=list)
    x_coms:           List[float] = field(default_factory=list)
    y_coms:           List[float] = field(default_factory=list)
    vxs:              List[float] = field(default_factory=list)
    vys:              List[float] = field(default_factory=list)
    final_time:       float = 0.0  # t at which the manoeuvre settled


def run_simulation(theta_0, theta_target, omega_0,
                   dry_mass, fuel_mass, thrust_force, v_e,
                   effective_target, use_wrap,
                   include_translation, config):
    """
    Run the attitude control simulation.

    Parameters
    ----------
    theta_0, theta_target : float  - initial and target angles (deg)
    omega_0               : float  - initial angular rate (deg/s)
    dry_mass, fuel_mass   : float  - masses (kg)
    thrust_force          : float  - engine thrust (N)
    v_e                   : float  - exhaust velocity (m/s)
    effective_target      : float  - from check_controllability
    use_wrap              : bool   - from check_controllability
    include_translation   : bool   - whether to integrate translational DoF
    config                : AttitudeConfig

    Returns
    -------
    SimResult
    """
    res = SimResult()

    curr_theta  = theta_0
    curr_omega  = omega_0
    curr_gimbal = 0.0
    curr_mass   = dry_mass + fuel_mass
    x_com = y_com = vx = vy = 0.0

    t       = 0.0
    running = True
    settled = False
    dt      = config.dt

    while running and t < config.max_time:
        fuel_remaining = max(curr_mass - dry_mass, 0.0)
        com, I = calculate_com_and_I(curr_mass, fuel_remaining, config)
        lever_arm = com

        max_alpha = np.degrees(
            (thrust_force * np.sin(np.radians(config.gimbal_limit)) * lever_arm) / I
        )

        # ---- Control law ----
        if use_wrap:
            error = wrap_angle(effective_target - curr_theta)
        else:
            error = effective_target - curr_theta

        target_omega      = np.sign(error) * np.sqrt(2 * max_alpha * abs(error))
        switch_error      = target_omega - curr_omega
        boundary          = config.boundary

        if abs(switch_error) > boundary:
            target_gimbal_cmd = -np.sign(switch_error) * config.gimbal_limit
        else:
            target_gimbal_cmd = -(switch_error / boundary) * config.gimbal_limit

        active_thrust = True
        if abs(error) < config.settle_pos_tol and abs(curr_omega) < config.settle_vel_tol:
            target_gimbal_cmd = 0.0
            curr_omega        = 0.0
            active_thrust     = False
            settled           = True

        # Gimbal slew rate limit
        gimbal_err = target_gimbal_cmd - curr_gimbal
        if abs(gimbal_err) > 0.01:
            step        = np.sign(gimbal_err) * config.max_gimbal_speed * dt
            curr_gimbal += np.clip(step, -abs(gimbal_err), abs(gimbal_err))

        # ---- Physics ----
        current_thrust = thrust_force if active_thrust else 0.0

        if include_translation:
            thrust_rad = np.radians(curr_theta + curr_gimbal)
            ax_lin = (current_thrust * np.sin(thrust_rad)) / curr_mass if curr_mass > dry_mass + 1e-6 else 0.0
            ay_lin = (current_thrust * np.cos(thrust_rad)) / curr_mass if curr_mass > dry_mass + 1e-6 else 0.0
            vx    += ax_lin * dt
            vy    += ay_lin * dt
            x_com += vx * dt
            y_com += vy * dt

        torque     = -thrust_force * np.sin(np.radians(curr_gimbal)) * lever_arm
        alpha      = np.degrees(torque / I)
        curr_omega += alpha * dt
        curr_theta += curr_omega * dt

        if active_thrust:
            curr_mass = max(curr_mass - (current_thrust / v_e) * dt, dry_mass)

        # ---- Record ----
        res.times.append(t)
        res.angles.append(wrap_angle(curr_theta))
        res.unwrapped_angles.append(curr_theta)
        res.omegas.append(curr_omega)
        res.gimbals.append(curr_gimbal)
        res.masses.append(curr_mass)
        res.thrusts.append(current_thrust)
        res.torques.append(torque)
        res.coms.append(com)
        res.Is.append(I)
        res.x_coms.append(x_com)
        res.y_coms.append(y_com)
        res.vxs.append(vx)
        res.vys.append(vy)

        t += dt
        if (settled and abs(curr_gimbal) < 0.01) or curr_mass <= dry_mass:
            running = False

    res.final_time = t

    # ---- Post-settle hold ----
    hold_steps = int(config.post_hold / dt)
    for _ in range(hold_steps):
        res.times.append(res.times[-1] + dt)
        res.angles.append(wrap_angle(curr_theta))
        res.unwrapped_angles.append(curr_theta)
        res.omegas.append(0.0)
        res.gimbals.append(0.0)
        res.masses.append(res.masses[-1])
        res.thrusts.append(0.0)
        res.torques.append(0.0)
        res.coms.append(res.coms[-1])
        res.Is.append(res.Is[-1])
        res.x_coms.append(res.x_coms[-1])
        res.y_coms.append(res.y_coms[-1])
        res.vxs.append(res.vxs[-1])
        res.vys.append(res.vys[-1])

    print(f"Final stopping time: {res.final_time:.2f} s")
    return res
