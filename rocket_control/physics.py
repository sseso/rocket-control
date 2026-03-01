"""
physics.py
----------
Pure-Python / NumPy helpers for rocket mass properties and a rough
pre-flight controllability check. All magic numbers come from a RocketConfig.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Mass properties
# ---------------------------------------------------------------------------

def calculate_com_and_I(curr_mass, config):
    """
    Compute center-of-mass location (from nozzle) and axial moment of inertia
    for the given total mass, using geometry from config.

    Returns
    -------
    com_from_nozzle : float  (m)
    I_z             : float  (kg·m²)
    """
    fuel_remaining = max(curr_mass - config.m_dry, 0.0)
    com_from_nozzle = (
        config.m_dry  * config.dry_cm_from_nozzle +
        fuel_remaining * config.fuel_cm_from_nozzle
    ) / curr_mass

    I_dry_cm   = (1 / 12) * config.m_dry * config.rocket_height ** 2
    dry_dist   = config.dry_cm_from_nozzle - com_from_nozzle
    I_dry      = I_dry_cm + config.m_dry * dry_dist ** 2

    if fuel_remaining > 0:
        I_fuel_cm = (1 / 12) * fuel_remaining * config.fuel_tank_height ** 2
        fuel_dist = config.fuel_cm_from_nozzle - com_from_nozzle
        I_fuel    = I_fuel_cm + fuel_remaining * fuel_dist ** 2
    else:
        I_fuel = 0.0

    return com_from_nozzle, I_dry + I_fuel


# ---------------------------------------------------------------------------
# Pre-flight feasibility check
# ---------------------------------------------------------------------------

def approx_controllable(initial_state, config):
    """
    Very rough sanity check: is the landing physically plausible?

    Parameters
    ----------
    initial_state : array-like, shape (7,)
        [x, y, vx, vy, theta, omega, m]
    config : RocketConfig (or subclass)

    Returns
    -------
    bool
    """
    x, y, vx, vy, theta, omega, m = initial_state
    fuel_mass = m - config.m_dry

    if fuel_mass < 1e-6:
        print("No usable fuel --> cannot control attitude, position or rate.")
        return False

    com, _ = calculate_com_and_I(m, config)
    nozzle_y = y - com
    if nozzle_y < config.nozzle_ground_threshold:
        return False

    hover_thrust_needed = m * config.g
    if config.T_max < config.hover_thrust_safety_factor * hover_thrust_needed:
        print("Thrust too low even to hover.")
        return False

    dv_available     = config.v_e * np.log(m / config.m_dry)
    speed            = np.sqrt(vx ** 2 + vy ** 2)
    dv_required_rough = speed + config.dv_gravity_loss_factor * config.g * (
        abs(vy) / config.g + config.dv_time_margin
    )

    if dv_available < config.dv_available_margin * dv_required_rough:
        print(f"Delta-v budget too low: {dv_available:.1f} <~ {dv_required_rough:.1f} m/s")
        return False

    if (abs(theta) > np.deg2rad(config.initial_theta_max_deg) or
            abs(omega) > np.deg2rad(config.initial_omega_max_deg)):
        print("Initial attitude or rate too extreme for recovery with limited control.")
        return False

    if abs(x) > config.initial_x_max:
        print("Too far downrange for realistic recovery.")
        return False

    return True
