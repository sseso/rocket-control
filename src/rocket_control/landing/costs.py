"""NLP running cost matching the README (trapezoidal quadrature).

J_ground is a soft complement to the hard bound y >= 0.
"""

from __future__ import annotations

from typing import Any

from rocket_control.core.mass_properties import com_and_inertia_ca

from .scenarios import LandingProblem


def collocation_objective(opti_syms: dict[str, Any], problem: LandingProblem) -> Any:
    import casadi as ca

    dt = opti_syms["dt"]
    tf = opti_syms["tf"]
    ys = opti_syms["ys"]
    vxs = opti_syms["vxs"]
    vys = opti_syms["vys"]
    thetas = opti_syms["thetas"]
    ms = opti_syms["ms"]
    Ts = opti_syms["Ts"]
    alphas = opti_syms["alphas"]
    y0 = opti_syms["y0"]
    N = problem.mesh.N
    w = problem.weights

    com_heights = ca.horzcat(*[com_and_inertia_ca(ms[k], problem.vehicle)[0] for k in range(N + 1)])
    nozzle_heights = ys - com_heights
    ground_proximity = ca.fmax(0, 1.0 - nozzle_heights / problem.clearance_zone) ** 3
    ground_violation = ca.fmax(0, -nozzle_heights)
    h_nozzle_safe = ca.fmax(w.epsilon, nozzle_heights)
    y_norm = ys / y0

    J_time = w.w_time * tf
    J_thrust = w.w_thrust * dt * ca.sumsqr(Ts)
    J_gimbal = w.w_gimbal * dt * ca.sumsqr(alphas)
    J_gimbal_rate = w.w_gimbal_rate * dt * ca.sumsqr(ca.diff(alphas) / dt)
    J_theta = w.w_theta * dt * ca.sumsqr(thetas)
    J_inverse_h = w.w_v * dt * ca.sum2((vxs**2 + vys**2) / h_nozzle_safe)
    J_ground = w.w_ground * ca.sumsqr(ground_violation)
    J_landing_theta = w.w_landing * dt * ca.sumsqr(thetas * ground_proximity)
    J_landing_gimbal = w.w_landing * dt * ca.sumsqr(alphas * ground_proximity)
    J_alt_thrust = w.w_alt_thrust * dt * ca.sum2((Ts**2) * y_norm)
    return (
        J_time
        + J_thrust
        + J_gimbal
        + J_gimbal_rate
        + J_theta
        + J_alt_thrust
        + J_ground
        + J_landing_theta
        + J_landing_gimbal
        + J_inverse_h
    )
