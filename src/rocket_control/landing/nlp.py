"""Trapezoidal direct collocation + IPOPT for the 2D landing NLP."""

from __future__ import annotations

from typing import Sequence

import numpy as np

from rocket_control.core import IM, IY, N_CONTROLS, N_STATES
from rocket_control.core.dynamics import rocket_dynamics_ca
from rocket_control.core.mass_properties import com_and_inertia, com_and_inertia_ca

from .costs import collocation_objective
from .scenarios import LandingProblem
from .types import Trajectory


def regularize_initial_state(state: Sequence[float], eps: float) -> np.ndarray:
    """Nudge exactly-zero x / vx / vy so the gimbal gradient is not identically 0."""
    s = np.array(state, dtype=float, copy=True)
    for idx in (0, 2, 3):  # x, vx, vy
        if abs(s[idx]) < eps:
            s[idx] = eps if s[idx] >= 0.0 else -eps
    return s


def solve_optimal_landing(
    initial_state: Sequence[float],
    problem: LandingProblem,
    *,
    verbose: bool = False,
) -> Trajectory:
    import casadi as ca

    s0_user = np.asarray(initial_state, dtype=float)
    s0 = regularize_initial_state(s0_user, problem.mesh.eps_state)
    vehicle = problem.vehicle
    mesh = problem.mesh
    bounds = problem.bounds
    N = mesh.N

    opti = ca.Opti()
    X = opti.variable(N_STATES, N + 1)
    U = opti.variable(N_CONTROLS, N + 1)
    tf = opti.variable()
    dt = tf / N

    xs, ys, vxs, vys, thetas, omegas, ms = (
        X[0, :], X[1, :], X[2, :], X[3, :], X[4, :], X[5, :], X[6, :]
    )
    Ts, alphas = U[0, :], U[1, :]

    for k in range(N):
        s_k, s_kp1 = X[:, k], X[:, k + 1]
        u_k, u_kp1 = U[:, k], U[:, k + 1]
        f_k = rocket_dynamics_ca(s_k, u_k, vehicle)
        f_kp1 = rocket_dynamics_ca(s_kp1, u_kp1, vehicle)
        opti.subject_to(s_kp1 == s_k + 0.5 * dt * (f_k + f_kp1))

    opti.subject_to(X[:, 0] == s0)
    opti.subject_to(X[0, -1] == 0)
    opti.subject_to(X[2, -1] == 0)
    opti.subject_to(X[3, -1] == 0)
    opti.subject_to(X[4, -1] == 0)
    opti.subject_to(X[5, -1] == 0)
    com_term, _ = com_and_inertia_ca(X[6, -1], vehicle)
    opti.subject_to(X[1, -1] == com_term)
    opti.subject_to(U[1, -1] == 0.0)

    opti.subject_to(opti.bounded(mesh.tf_min, tf, mesh.tf_max))
    opti.subject_to(opti.bounded(0.0, ys, ca.inf))
    opti.subject_to(opti.bounded(bounds.vx_min, vxs, bounds.vx_max))
    opti.subject_to(opti.bounded(bounds.vy_min, vys, bounds.vy_max))
    opti.subject_to(opti.bounded(bounds.omega_min, omegas, bounds.omega_max))
    opti.subject_to(opti.bounded(vehicle.m_dry, ms, s0[IM]))
    opti.subject_to(opti.bounded(vehicle.T_min, Ts, vehicle.T_max))

    for k in range(N + 1):
        if k >= N + 1 - mesh.landing_nodes:
            opti.subject_to(opti.bounded(-bounds.tight_theta, thetas[k], bounds.tight_theta))
            opti.subject_to(opti.bounded(-bounds.tight_alpha, alphas[k], bounds.tight_alpha))
        else:
            opti.subject_to(opti.bounded(-bounds.theta_max_loose, thetas[k], bounds.theta_max_loose))
            opti.subject_to(opti.bounded(-bounds.alpha_max, alphas[k], bounds.alpha_max))

    J = collocation_objective(
        {
            "dt": dt,
            "tf": tf,
            "ys": ys,
            "vxs": vxs,
            "vys": vys,
            "thetas": thetas,
            "ms": ms,
            "Ts": Ts,
            "alphas": alphas,
            "y0": s0[IY],
        },
        problem,
    )
    opti.minimize(J)

    opti.set_initial(tf, mesh.initial_tf_guess)
    target_com, _ = com_and_inertia(vehicle.m_dry, vehicle)
    target_state = np.array([0.0, target_com, 0.0, 0.0, 0.0, 0.0, vehicle.m_dry])
    for i in range(N_STATES):
        opti.set_initial(X[i, :], np.linspace(s0[i], target_state[i], N + 1))
    hover = float(np.clip(s0[IM] * vehicle.g, vehicle.T_min, vehicle.T_max))
    opti.set_initial(Ts, hover)
    opti.set_initial(alphas, 0.0)

    print_level = 5 if verbose else 0
    opti.solver(
        "ipopt",
        {"expand": True},
        {"max_iter": mesh.max_iter, "tol": mesh.tol, "print_level": print_level},
    )

    try:
        sol = opti.solve()
        success = True
    except RuntimeError:
        sol = opti.debug
        success = False

    states = np.array(sol.value(X)).T
    controls = np.array(sol.value(U)).T
    tf_sol = float(sol.value(tf))
    times = np.linspace(0.0, tf_sol, N + 1)
    return Trajectory(
        times=times,
        states=states,
        controls=controls,
        tf=tf_sol,
        success=success,
        initial_state=s0,
    )
