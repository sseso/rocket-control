"""
solver.py
---------
CasADi + IPOPT optimal landing solver.
All problem parameters come from a RocketConfig (or subclass).
"""

import numpy as np
import casadi as ca

from .physics import calculate_com_and_I


def solve_optimal_landing(initial_state, config):
    """
    Set up and solve the time-optimal rocket landing NLP.

    Parameters
    ----------
    initial_state : array-like, shape (7,)
        [x, y, vx, vy, theta, omega, m]
    config : RocketConfig (or subclass)

    Returns
    -------
    states_sol   : ndarray, shape (N+1, 7)
    controls_sol : ndarray, shape (N+1, 2)
    tf_sol       : float
    success      : bool
    """
    print("Setting up CasADi + IPOPT NLP...")
    opti = ca.Opti()
    N    = config.N

    # --- Decision Variables ---
    X  = opti.variable(7, N + 1)  # States:   [x, y, vx, vy, theta, omega, m]
    U  = opti.variable(2, N + 1)  # Controls: [T, alpha]
    tf = opti.variable()           # Final time
    dt = tf / N

    xs, ys, vxs, vys, thetas, omegas, ms = (
        X[0, :], X[1, :], X[2, :], X[3, :], X[4, :], X[5, :], X[6, :]
    )
    Ts, alphas = U[0, :], U[1, :]

    # --- CasADi helpers for CoM and I ---
    def _calc_com_I(m):
        fuel_remaining  = m - config.m_dry
        com_from_nozzle = (
            config.m_dry  * config.dry_cm_from_nozzle +
            fuel_remaining * config.fuel_cm_from_nozzle
        ) / m

        I_dry_cm  = (1 / 12) * config.m_dry * config.rocket_height ** 2
        dry_dist  = config.dry_cm_from_nozzle - com_from_nozzle
        I_dry     = I_dry_cm + config.m_dry * dry_dist ** 2

        I_fuel_cm = (1 / 12) * fuel_remaining * config.fuel_tank_height ** 2
        fuel_dist = config.fuel_cm_from_nozzle - com_from_nozzle
        I_fuel    = I_fuel_cm + fuel_remaining * fuel_dist ** 2

        return com_from_nozzle, I_dry + I_fuel

    # --- Dynamics ---
    def _dynamics(state, control):
        x, y, vx, vy, theta, omega, m = (
            state[0], state[1], state[2], state[3], state[4], state[5], state[6]
        )
        T, alpha = control[0], control[1]

        l_t, I_z      = _calc_com_I(m)
        thrust_angle  = theta + alpha

        dx     = vx
        dy     = vy
        dvx    = (T / m) * ca.sin(thrust_angle)
        dvy    = (T / m) * ca.cos(thrust_angle) - config.g
        dtheta = omega
        domega = -(l_t * T / I_z) * ca.sin(alpha)
        dm     = -T / config.v_e

        return ca.vertcat(dx, dy, dvx, dvy, dtheta, domega, dm)

    # --- Collocation Constraints (Trapezoidal) ---
    for k in range(N):
        s_k   = X[:, k]
        s_kp1 = X[:, k + 1]
        u_k   = U[:, k]
        u_kp1 = U[:, k + 1]

        f_k   = _dynamics(s_k,   u_k)
        f_kp1 = _dynamics(s_kp1, u_kp1)

        opti.subject_to(s_kp1 == s_k + 0.5 * dt * (f_k + f_kp1))

    # --- Boundary Conditions ---
    opti.subject_to(X[:, 0] == initial_state)

    opti.subject_to(X[0, -1] == 0)   # x  = 0
    opti.subject_to(X[2, -1] == 0)   # vx = 0
    opti.subject_to(X[3, -1] == 0)   # vy = 0
    opti.subject_to(X[4, -1] == 0)   # theta = 0
    opti.subject_to(X[5, -1] == 0)   # omega = 0

    com_term, _ = _calc_com_I(X[6, -1])
    opti.subject_to(X[1, -1] == com_term)   # y = CoM height at touchdown
    opti.subject_to(U[1, -1] == 0.0)        # gimbal centred at touchdown

    # --- Variable Bounds ---
    opti.subject_to(opti.bounded(config.tf_min, tf, config.tf_max))
    opti.subject_to(opti.bounded(0.0, ys, ca.inf))
    opti.subject_to(opti.bounded(config.vx_min,  vxs,  config.vx_max))
    opti.subject_to(opti.bounded(config.vy_min,  vys,  config.vy_max))
    opti.subject_to(opti.bounded(config.omega_min, omegas, config.omega_max))
    opti.subject_to(opti.bounded(config.m_dry, ms, initial_state[6]))
    opti.subject_to(opti.bounded(config.T_min, Ts, config.T_max))

    landing_nodes = config.LANDING_NODES
    for k in range(N + 1):
        if k >= N + 1 - landing_nodes:
            opti.subject_to(opti.bounded(-config.tight_theta, thetas[k],  config.tight_theta))
            opti.subject_to(opti.bounded(-config.tight_alpha, alphas[k],  config.tight_alpha))
        else:
            opti.subject_to(opti.bounded(-config.theta_max_loose, thetas[k], config.theta_max_loose))
            opti.subject_to(opti.bounded(-config.alpha_max,       alphas[k], config.alpha_max))

    # --- Objective ---
    J_time       = config.w_time * tf
    J_thrust     = config.w_thrust * dt * ca.sumsqr(Ts)
    J_gimbal     = config.w_gimbal * dt * ca.sumsqr(alphas)
    J_gimbal_rate = config.w_gimbal_rate * dt * ca.sumsqr(ca.diff(alphas) / dt)
    J_theta      = config.w_theta * dt * ca.sumsqr(thetas)

    com_heights   = ca.horzcat(*[_calc_com_I(ms[k])[0] for k in range(N + 1)])
    nozzle_heights = ys - com_heights

    ground_proximity = ca.fmax(0, 1.0 - nozzle_heights / config.clearance_zone) ** 3
    ground_violation = ca.fmax(0, -nozzle_heights)
    h_nozzle_safe    = ca.fmax(config.epsilon, nozzle_heights)

    J_inverse_h     = config.w_v       * dt * ca.sum2((vxs ** 2 + vys ** 2) / h_nozzle_safe)
    J_ground        = 1e8              * ca.sumsqr(ground_violation)
    J_landing_theta = config.w_landing * dt * ca.sumsqr(thetas  * ground_proximity)
    J_landing_gimbal = config.w_landing * dt * ca.sumsqr(alphas * ground_proximity)

    y_norm       = ys / initial_state[1]
    J_alt_thrust = config.w_alt_thrust * dt * ca.sum2((Ts ** 2) * y_norm)

    J = (J_time + J_thrust + J_gimbal + J_gimbal_rate + J_theta +
         J_alt_thrust + J_ground + J_landing_theta + J_landing_gimbal + J_inverse_h)
    opti.minimize(J)

    # --- Initial Guesses ---
    opti.set_initial(tf, config.initial_tf_guess)

    target_com, _ = calculate_com_and_I(config.m_dry, config)
    target_state  = np.array([0, target_com, 0, 0, 0, 0, config.m_dry])
    for i in range(7):
        opti.set_initial(X[i, :], np.linspace(initial_state[i], target_state[i], N + 1))

    hover_thrust = initial_state[6] * config.g
    opti.set_initial(Ts,     np.clip(hover_thrust, config.T_min, config.T_max))
    opti.set_initial(alphas, 0.0)

    # --- Solve ---
    p_opts = {"expand": True}
    s_opts = {"max_iter": 500, "tol": 1e-6, "print_level": 5}
    opti.solver("ipopt", p_opts, s_opts)

    print("\nStarting Optimization with IPOPT...")
    try:
        sol     = opti.solve()
        success = True
        print("\n--- Solution Report ---")
        print(f"  Converged:        {success}")
    except RuntimeError:
        sol     = opti.debug
        success = False
        print("\n  WARNING: Optimizer did not fully converge.")
        print("  The solution may be approximate. Try adjusting initial conditions.")

    states_sol   = sol.value(X).T
    controls_sol = sol.value(U).T
    tf_sol       = sol.value(tf)

    com_final, _ = calculate_com_and_I(states_sol[-1, 6], config)
    final_pos_err = np.linalg.norm([states_sol[-1, 0], states_sol[-1, 1] - com_final])
    final_vel_err = np.linalg.norm(states_sol[-1, 2:4])

    print(f"  Landing time:     {tf_sol:.3f} s")
    print(f"  Final pos error:  {final_pos_err:.4f} m")
    print(f"  Final vel error:  {final_vel_err:.4f} m/s")
    print(f"  Fuel consumed:    {initial_state[6] - states_sol[-1, 6]:.2f} kg")

    return states_sol, controls_sol, tf_sol, success
