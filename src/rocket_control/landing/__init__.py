"""Time-optimal 2D landing via trapezoidal collocation + IPOPT."""

from .grid import GridPointResult, grid_axes, run_success_grid
from .evaluate import LandingMetrics, evaluate_landing
from .feasibility import HeuristicFeasibility, heuristic_feasible
from .nlp import solve_optimal_landing
from .scenarios import LandingProblem, default_landing_problem, default_scenario
from .types import Trajectory

__all__ = [
    "LandingMetrics",
    "LandingProblem",
    "Trajectory",
    "evaluate_landing",
    "heuristic_feasible",
    "HeuristicFeasibility",
    "solve_optimal_landing",
    "default_landing_problem",
    "default_scenario",
    "GridPointResult",
    "grid_axes",
    "run_success_grid",
]
