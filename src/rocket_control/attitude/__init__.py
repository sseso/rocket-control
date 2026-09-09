"""Closed-loop bang-bang attitude demo on the shared vacuum plant."""

from .config import AttitudeSettings
from .feasibility import plan_manoeuvre
from .simulate import AttitudeResult, run_simulation

__all__ = [
    "AttitudeSettings",
    "AttitudeResult",
    "plan_manoeuvre",
    "run_simulation",
]
