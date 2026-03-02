"""
attitude/physics.py
-------------------
Pure-physics helpers for the attitude control simulation.
No matplotlib, no CasADi — only NumPy.
"""

import sys
import numpy as np


def wrap_angle(a):
    """Wrap angle to (-180, 180] degrees."""
    return (a + 180) % 360 - 180


def calculate_com_and_I(curr_mass, fuel_remaining, config):
    """
    Compute center-of-mass location (from nozzle) and axial moment of inertia.

    Parameters
    ----------
    curr_mass      : float  - total current mass (kg)
    fuel_remaining : float  - remaining fuel mass (kg)
    config         : AttitudeConfig

    Returns
    -------
    com_from_nozzle : float  (m)
    I_z             : float  (kg*m^2)
    """
    dry_mass = curr_mass - fuel_remaining

    com_from_nozzle = (
        dry_mass       * config.dry_cm_from_nozzle +
        fuel_remaining * config.fuel_cm_from_nozzle
    ) / curr_mass

    I_dry_cm  = (1 / 12) * dry_mass * config.rocket_height ** 2
    dry_dist  = config.dry_cm_from_nozzle - com_from_nozzle
    I_dry     = I_dry_cm + dry_mass * dry_dist ** 2

    I_fuel_cm = (1 / 12) * fuel_remaining * config.fuel_tank_height ** 2
    fuel_dist = config.fuel_cm_from_nozzle - com_from_nozzle
    I_fuel    = I_fuel_cm + fuel_remaining * fuel_dist ** 2

    return com_from_nozzle, I_dry + I_fuel


def check_controllability(theta_0, theta_target, omega_0,
                           dry_mass, fuel_mass, thrust_force, v_e, config):
    """
    Verify the manoeuvre is physically achievable and choose short vs long path.

    Returns
    -------
    effective_target : float  - the actual target angle to drive toward (deg)
    use_wrap         : bool   - whether to use wrap_angle in the error signal

    Raises SystemExit if uncontrollable.
    """
    initial_mass = dry_mass + fuel_mass
    initial_com, initial_I = calculate_com_and_I(initial_mass, fuel_mass, config)

    max_alpha = np.degrees(
        (thrust_force * np.sin(np.radians(config.gimbal_limit)) * initial_com) / initial_I
    )

    # ---- Can we even stop the initial rotation? ----
    stopping_time = abs(omega_0) / max_alpha if max_alpha > 0 else 0
    fuel_burn_rate = thrust_force / v_e
    fuel_needed = fuel_burn_rate * stopping_time

    if fuel_needed > fuel_mass:
        sys.exit("ABORT: Not enough fuel to stop initial rotation.")

    # ---- Can we reach the target? ----
    short_error   = wrap_angle(theta_target - theta_0)
    stopping_dist = (omega_0 ** 2) / (2 * max_alpha) if max_alpha > 0 else 9_999_999

    use_wrap = True
    effective_target = theta_target

    if np.sign(omega_0) != np.sign(short_error) and stopping_dist > abs(short_error):
        # Long path is actually cheaper
        use_wrap = False
        effective_target = theta_0 + (short_error - 360 * np.sign(short_error))
        print(f"Using long path for optimality (effective target: {effective_target:.2f}°)")
    else:
        print("Using short path.")

    effective_error = abs(effective_target - theta_0)
    maneuver_time   = 2 * np.sqrt(effective_error / max_alpha) if max_alpha > 0 else 0
    fuel_maneuver   = fuel_burn_rate * maneuver_time
    fuel_stop       = (
        fuel_burn_rate * abs(omega_0) / max_alpha
        if np.sign(omega_0) != np.sign(effective_target - theta_0) and max_alpha > 0
        else 0
    )
    fuel_needed = fuel_maneuver + fuel_stop

    if fuel_needed > fuel_mass:
        sys.exit("ABORT: Not enough fuel to reach the target.")

    print("System is controllable. Proceeding to simulation...")
    return effective_target, use_wrap
