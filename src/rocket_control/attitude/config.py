"""Attitude closed-loop settings. All angles are radians."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AttitudeSettings:
    gimbal_limit: float = np.deg2rad(10.0)
    max_gimbal_speed: float = np.deg2rad(360.0)
    dt: float = 0.01
    max_time: float = 40.0
    post_hold: float = 1.5
    boundary: float = np.deg2rad(0.2)
    settle_pos_tol: float = np.deg2rad(0.2)
    settle_vel_tol: float = np.deg2rad(0.2)
    fps: int = 50
    step_size: int = 2
    hold_frames: int = 120
    arrow_length_rot: float = 20.0
    arrow_length_trans_fraction: float = 0.2
    trans_scale_factor: float = 1.75
    trans_min_margin: float = 30.0
