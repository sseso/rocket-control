"""State and control layouts.

State x = [x, y, v_x, v_y, theta, omega, m]
  x, y     : CoM position (m); y is altitude of the CoM
  v_x, v_y : CoM velocity (m/s)
  theta    : pitch from vertical (rad), theta = 0 is upright
  omega    : pitch rate (rad/s), omega = d(theta)/dt
  m        : total mass (kg)

Control u = [T, alpha]
  T     : thrust (N)
  alpha : gimbal angle from the body axis (rad)
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
from numpy.typing import NDArray

IX, IY, IVX, IVY, ITHETA, IOMEGA, IM = range(7)
IT, IALPHA = 0, 1
N_STATES = 7
N_CONTROLS = 2

State = NDArray[np.float64]
Control = NDArray[np.float64]


def as_state(values: Sequence[float]) -> State:
    arr = np.asarray(values, dtype=float).reshape(N_STATES)
    return arr


def as_control(values: Sequence[float]) -> Control:
    arr = np.asarray(values, dtype=float).reshape(N_CONTROLS)
    return arr
