"""Mass-dependent CoM and axial moment of inertia.

Matches the README formulation:

    d_com(m) = (m_dry * d_dry + max(m - m_dry, 0) * d_fuel) / m

    I_z(m)   = I_dry,cm + m_dry (d_dry - d_com)^2
             + I_fuel,cm(m) + m_fuel_rem (d_fuel - d_com)^2

with rod inertias I_cm = m L^2 / 12. NumPy and CasADi share this algebra.
"""

from __future__ import annotations

from typing import Any, Tuple

import numpy as np

from .vehicle import VehicleSpec


def com_and_inertia(mass: float, vehicle: VehicleSpec) -> Tuple[float, float]:
    """Return (d_com from nozzle [m], I_z [kg m^2]) for a scalar mass."""
    m = float(mass)
    if m <= 0:
        raise ValueError("mass must be positive")
    fuel = max(m - vehicle.m_dry, 0.0)
    d_com = (
        vehicle.m_dry * vehicle.dry_cm_from_nozzle
        + fuel * vehicle.fuel_cm_from_nozzle
    ) / m

    i_dry_cm = (1.0 / 12.0) * vehicle.m_dry * vehicle.rocket_height**2
    i_dry = i_dry_cm + vehicle.m_dry * (vehicle.dry_cm_from_nozzle - d_com) ** 2

    if fuel > 0:
        i_fuel_cm = (1.0 / 12.0) * fuel * vehicle.fuel_tank_height**2
        i_fuel = i_fuel_cm + fuel * (vehicle.fuel_cm_from_nozzle - d_com) ** 2
    else:
        i_fuel = 0.0

    return d_com, i_dry + i_fuel


def com_and_inertia_ca(mass: Any, vehicle: VehicleSpec) -> Tuple[Any, Any]:
    """CasADi version of :func:'com_and_inertia'.

    Fuel mass is 'm - m_dry' without 'fmax' so the NLP Hessian stays smooth.
    """
    fuel = mass - vehicle.m_dry
    d_com = (
        vehicle.m_dry * vehicle.dry_cm_from_nozzle
        + fuel * vehicle.fuel_cm_from_nozzle
    ) / mass

    i_dry_cm = (1.0 / 12.0) * vehicle.m_dry * vehicle.rocket_height**2
    i_dry = i_dry_cm + vehicle.m_dry * (vehicle.dry_cm_from_nozzle - d_com) ** 2
    i_fuel_cm = (1.0 / 12.0) * fuel * vehicle.fuel_tank_height**2
    i_fuel = i_fuel_cm + fuel * (vehicle.fuel_cm_from_nozzle - d_com) ** 2
    return d_com, i_dry + i_fuel


def com_and_inertia_array(masses: np.ndarray, vehicle: VehicleSpec) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorized CoM / I_z for a 1-D mass array."""
    coms = np.empty_like(masses, dtype=float)
    i_zs = np.empty_like(masses, dtype=float)
    for i, m in enumerate(np.asarray(masses, dtype=float).ravel()):
        coms.ravel()[i], i_zs.ravel()[i] = com_and_inertia(m, vehicle)
    return coms, i_zs
