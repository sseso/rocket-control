"""Shared SI physics: vehicle, mass properties, and 2D rocket dynamics."""

from .constants import G0, G_EARTH
from .exceptions import InfeasibleError
from .mass_properties import com_and_inertia, com_and_inertia_ca
from .dynamics import rocket_dynamics, rocket_dynamics_ca
from .types import (
    IX,
    IY,
    IVX,
    IVY,
    ITHETA,
    IOMEGA,
    IM,
    IT,
    IALPHA,
    N_STATES,
    N_CONTROLS,
    State,
    Control,
)
from .vehicle import VehicleSpec, default_vehicle

__all__ = [
    "G0",
    "G_EARTH",
    "InfeasibleError",
    "VehicleSpec",
    "default_vehicle",
    "com_and_inertia",
    "com_and_inertia_ca",
    "rocket_dynamics",
    "rocket_dynamics_ca",
    "IX",
    "IY",
    "IVX",
    "IVY",
    "ITHETA",
    "IOMEGA",
    "IM",
    "IT",
    "IALPHA",
    "N_STATES",
    "N_CONTROLS",
    "State",
    "Control",
]
