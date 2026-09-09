"""Landing NLP I/O types."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class Trajectory:
    """Discretized collocation solution (SI, radians)."""

    times: NDArray[np.float64]
    states: NDArray[np.float64]
    controls: NDArray[np.float64]
    tf: float
    success: bool
    initial_state: NDArray[np.float64]
