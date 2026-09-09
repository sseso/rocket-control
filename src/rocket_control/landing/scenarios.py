"""Landing NLP configuration split into vehicle, weights, bounds, and IC."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from rocket_control.core import VehicleSpec, default_vehicle
from rocket_control.core.mass_properties import com_and_inertia


@dataclass(frozen=True)
class Discretization:
    N: int = 40
    tf_min: float = 1.0
    tf_max: float = 60.0
    initial_tf_guess: float = 15.0
    landing_nodes: int = 6
    max_iter: int = 500
    tol: float = 1e-6
    # Small IC regularisation so IPOPT is not stuck at a stationary gimbal gradient.
    eps_state: float = 1e-4


@dataclass(frozen=True)
class PathBounds:
    vx_min: float = -50.0
    vx_max: float = 50.0
    vy_min: float = -100.0
    vy_max: float = 50.0
    omega_min: float = -0.5
    omega_max: float = 0.5
    theta_max_loose: float = np.pi / 4
    tight_theta: float = np.deg2rad(2.0)
    tight_alpha: float = np.deg2rad(2.0)
    alpha_max: float = np.deg2rad(10.0)


@dataclass(frozen=True)
class NlpWeights:
    """Terms in J as documented in the README."""

    w_time: float = 10.0
    w_thrust: float = 1e-10
    w_gimbal: float = 0.05
    w_gimbal_rate: float = 0.8
    w_theta: float = 80.0
    w_alt_thrust: float = 6.25e-6
    w_landing: float = 4000.0
    w_v: float = 50.0
    w_ground: float = 1e8
    epsilon: float = 5.0
    clearance_zone_factor: float = 2.0


@dataclass(frozen=True)
class FeasibilityThresholds:
    nozzle_ground_threshold: float = -0.1
    hover_thrust_safety_factor: float = 0.6
    dv_available_margin: float = 0.6
    initial_theta_max: float = np.deg2rad(25.0)
    initial_omega_max: float = np.deg2rad(35.0)
    initial_x_max: float = 800.0
    dv_gravity_loss_factor: float = 1.5
    dv_time_margin: float = 5.0


@dataclass(frozen=True)
class LandingTolerances:
    landing_pos_tol: float = 0.10
    landing_vel_tol: float = 0.15
    min_nozzle_tol: float = -0.30
    skidding_x_threshold: float = 5.0


@dataclass(frozen=True)
class Scenario:
    """Initial conditions. 'theta0' / 'omega0' are radians."""

    nozzle_altitude: float = 160.42
    x0: float = 30.0
    vx0: float = -8.0
    vy0: float = -30.0
    theta0: float = 0.0
    omega0: float = 0.0


@dataclass(frozen=True)
class GridSpec:
    x_min: float = -10.0
    x_max: float = 10.0
    x_step: float = 5.0
    h_min: float = 0.0
    h_max: float = 100.0
    h_step: float = 25.0
    vx0: float = 0.0
    vy0: float = 0.0
    theta0: float = 0.0
    omega0: float = 0.0


@dataclass(frozen=True)
class LandingProblem:
    vehicle: VehicleSpec = field(default_factory=default_vehicle)
    mesh: Discretization = field(default_factory=Discretization)
    bounds: PathBounds = field(default_factory=PathBounds)
    weights: NlpWeights = field(default_factory=NlpWeights)
    feasibility: FeasibilityThresholds = field(default_factory=FeasibilityThresholds)
    tolerances: LandingTolerances = field(default_factory=LandingTolerances)
    scenario: Scenario = field(default_factory=Scenario)
    grid: GridSpec = field(default_factory=GridSpec)

    @property
    def clearance_zone(self) -> float:
        return self.vehicle.rocket_height * self.weights.clearance_zone_factor

    def initial_state(self, scenario: Scenario | None = None) -> np.ndarray:
        sc = scenario or self.scenario
        m0 = self.vehicle.m0
        d_com, _ = com_and_inertia(m0, self.vehicle)
        return np.array(
            [sc.x0, sc.nozzle_altitude + d_com, sc.vx0, sc.vy0, sc.theta0, sc.omega0, m0],
            dtype=float,
        )


def default_landing_problem() -> LandingProblem:
    return LandingProblem()


def default_scenario() -> Scenario:
    return Scenario()


def with_scenario(problem: LandingProblem, **kwargs: float) -> LandingProblem:
    return replace(problem, scenario=replace(problem.scenario, **kwargs))
