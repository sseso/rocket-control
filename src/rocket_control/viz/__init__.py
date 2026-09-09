"""Matplotlib animation and diagnostic plots (optional at import of physics)."""

from .diagnostics import plot_attitude_diagnostics, plot_landing_diagnostics
from .landing_anim import animate_landing
from .attitude_anim import animate_attitude

__all__ = [
    "plot_landing_diagnostics",
    "plot_attitude_diagnostics",
    "animate_landing",
    "animate_attitude",
]
