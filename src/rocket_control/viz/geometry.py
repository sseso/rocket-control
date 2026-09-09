"""Rocket drawing geometry. Independent of the plant."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from rocket_control.core.vehicle import VehicleSpec, default_vehicle


Point = tuple[float, float]


def rotate_point(x: float, y: float, pivot_x: float, pivot_y: float, angle_deg: float) -> Point:
    angle_rad = np.radians(angle_deg)
    nx, ny = x - pivot_x, y - pivot_y
    s, c = np.sin(-angle_rad), np.cos(-angle_rad)
    return nx * c - ny * s + pivot_x, nx * s + ny * c + pivot_y


def rotate_points(points: Sequence[Sequence[float]], pivot: Point, angle_deg: float) -> list[Point]:
    return [rotate_point(p[0], p[1], pivot[0], pivot[1], angle_deg) for p in points]


@dataclass(frozen=True)
class VisualGeometry:
    """Nozzle-origin polygons used by the landing animation."""

    body_raw: tuple[Point, ...] = (
        (-7.0, 10.0), (7.0, 10.0), (7.0, 80.0), (-7.0, 80.0),
    )
    nose_raw: tuple[Point, ...] = ((-7.0, 80.0), (7.0, 80.0), (0.0, 95.0))
    engine_attach: Point = (0.0, 10.0)
    engine_raw: tuple[Point, ...] = (
        (-8.4, 0.0), (8.4, 0.0), (4.5, 10.0), (-4.5, 10.0),
    )
    flame_raw: tuple[Point, ...] = ((-5.6, 0.0), (5.6, 0.0), (0.0, -35.0))
    rocket_height: float = 95.0


@dataclass(frozen=True)
class AttitudeVisual:
    """Geometric-centre polygons for the vacuum attitude views."""

    body_raw: tuple[Point, ...]
    nose_raw: tuple[Point, ...]
    engine_attach: Point
    engine_raw: tuple[Point, ...]
    flame_raw: tuple[Point, ...]
    rocket_height: float
    dry_cm_from_nozzle: float


def default_visual() -> VisualGeometry:
    return VisualGeometry()


def attitude_visual(vehicle: VehicleSpec | None = None) -> AttitudeVisual:
    """Scale the original 70 m drawing to the shared vehicle height."""
    v = vehicle or default_vehicle()
    scale = v.rocket_height / 70.0
    def _s(pts):
        return tuple((x * scale, y * scale) for x, y in pts)

    return AttitudeVisual(
        body_raw=_s(((-7, -35), (7, -35), (7, 35), (-7, 35))),
        nose_raw=_s(((-7, 35), (7, 35), (0, 50))),
        engine_attach=(0.0, -34.0 * scale),
        engine_raw=_s(((-8.4, -45), (8.4, -45), (4.5, -34), (-4.5, -34))),
        flame_raw=_s(((-5.6, -45), (5.6, -45), (0, -70))),
        rocket_height=v.rocket_height,
        dry_cm_from_nozzle=v.dry_cm_from_nozzle,
    )


@dataclass
class LandingVizSettings:
    fps: int = 30
    hold_time_start: float = 2.0
    hold_time_end: float = 2.0
    base_flame_length: float = 35.0
    flame_throttle_gamma: float = 0.5
    visual_margin_factor: float = 1.5
    visual_arrow_length_factor: float = 0.1
    label_width: int = field(default=len("FUEL BURNED:") + 1)
    value_width: int = 11
