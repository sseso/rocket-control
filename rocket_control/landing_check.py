"""
landing_check.py
----------------
Post-solve quality evaluation: checks tolerances and prints a report.
"""

import numpy as np
from .physics import calculate_com_and_I


def evaluate_landing(states, controls, tf, config):
    """
    Evaluate the quality of an optimal-landing solution.

    Parameters
    ----------
    states   : ndarray, shape (N+1, 7)
    controls : ndarray, shape (N+1, 2)
    tf       : float
    config   : RocketConfig (or subclass)

    Returns
    -------
    LANDING_GOOD : bool
    metrics      : dict  - error_x, error_y, final_pos_err, final_vel_err,
                           min_nozzle, skidding_detected
    """
    coms     = np.array([calculate_com_and_I(m, config)[0] for m in states[:, 6]])
    y_nozzle = states[:, 1] - coms
    x_coms   = states[:, 0]

    com_term, _ = calculate_com_and_I(states[-1, 6], config)
    error_x     = states[-1, 0] - 0.0
    error_y     = states[-1, 1] - com_term

    final_pos_err = np.linalg.norm([error_x, error_y])
    final_vel_err = np.linalg.norm(states[-1, 2:4])
    min_nozzle    = float(np.min(y_nozzle))

    skidding_detected = any(
        y_nozzle[i] < 0.1 and abs(x_coms[i]) > config.skidding_x_threshold
        for i in range(len(y_nozzle) - 1)
    )

    LANDING_GOOD = (
        final_pos_err < config.landing_pos_tol and
        final_vel_err < config.landing_vel_tol and
        min_nozzle    >= config.min_nozzle_tol  and
        not skidding_detected
    )

    metrics = dict(
        error_x=error_x,
        error_y=error_y,
        final_pos_err=final_pos_err,
        final_vel_err=final_vel_err,
        min_nozzle=min_nozzle,
        skidding_detected=skidding_detected,
    )

    return LANDING_GOOD, metrics


def print_landing_report(LANDING_GOOD, metrics, config):
    """Print a human-readable landing quality report."""
    if LANDING_GOOD:
        print("\nLanding looks good within tolerances!")
        print(f"  X-offset:      {metrics['error_x']:6.3f} m")
        print(f"  Y-offset:      {metrics['error_y']:6.3f} m  (+ = floating, − = sunk)")
        print(f"  Velocity err:  {metrics['final_vel_err']:6.3f} m/s")
        print(f"  Min nozzle:    {metrics['min_nozzle']:6.3f} m")
    else:
        print("\nSolution found, but landing quality is NOT acceptable:")
        issues = []
        if abs(metrics['error_x'])     > config.landing_pos_tol: issues.append(f"X-position error too high: {metrics['error_x']:.3f} m")
        if abs(metrics['error_y'])     > config.landing_pos_tol: issues.append(f"Y-position error too high: {metrics['error_y']:.3f} m")
        if metrics['final_pos_err']   >= config.landing_pos_tol: issues.append(f"pos error = {metrics['final_pos_err']:.3f} m")
        if metrics['final_vel_err']   >= config.landing_vel_tol: issues.append(f"vel error = {metrics['final_vel_err']:.3f} m/s")
        if metrics['min_nozzle']       < config.min_nozzle_tol:  issues.append(f"ground clip = {abs(metrics['min_nozzle']):.3f} m")
        if metrics['skidding_detected']:                          issues.append("Unrealistic skidding detected while |x| > 5 m")
        for issue in issues:
            print(f"  • {issue}")
