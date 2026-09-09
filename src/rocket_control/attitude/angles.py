"""Principal-value wrapping in radians."""

from __future__ import annotations

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)
