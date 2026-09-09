"""Vehicle geometry, mass, and propulsion parameters."""

from __future__ import annotations

from dataclasses import dataclass

from .constants import G0, G_EARTH


@dataclass(frozen=True)
class VehicleSpec:
    """Rigid slender-rod rocket with a lower-half fuel tank."""

    m_dry: float
    m_fuel: float
    T_max: float
    T_min: float
    I_sp: float
    rocket_height: float
    fuel_tank_height: float
    g: float = G_EARTH
    g0: float = G0

    def __post_init__(self) -> None:
        if self.m_dry <= 0:
            raise ValueError("m_dry must be positive")
        if self.m_fuel < 0:
            raise ValueError("m_fuel must be non-negative")
        if self.T_max <= 0:
            raise ValueError("T_max must be positive")
        if not 0.0 <= self.T_min <= self.T_max:
            raise ValueError("T_min must satisfy 0 <= T_min <= T_max")
        if self.I_sp <= 0:
            raise ValueError("I_sp must be positive")
        if self.rocket_height <= 0 or self.fuel_tank_height <= 0:
            raise ValueError("geometry heights must be positive")

    @property
    def v_e(self) -> float:
        """Effective exhaust velocity (m/s)."""
        return self.I_sp * self.g0

    @property
    def dry_cm_from_nozzle(self) -> float:
        """Dry CoM location measured from the nozzle (uniform rod)."""
        return 0.5 * self.rocket_height

    @property
    def fuel_cm_from_nozzle(self) -> float:
        """Fuel CoM location: tank occupies the lower half of the body."""
        return 0.5 * self.fuel_tank_height

    @property
    def m0(self) -> float:
        return self.m_dry + self.m_fuel


def default_vehicle() -> VehicleSpec:
    """Default landing vehicle."""
    height = 95.0
    return VehicleSpec(
        m_dry=1250.0,
        m_fuel=500.0,
        T_max=50_000.0,
        T_min=4_000.0,
        I_sp=500.0,
        rocket_height=height,
        fuel_tank_height=0.5 * height,
    )
