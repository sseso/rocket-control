from .config import RocketConfig
import numpy as np


class RocketConfigTest(RocketConfig):
    """
    Configuration variant for the convergence / reachable-set grid test.
    Inherits all defaults from RocketConfig and overrides only what differs.
    """

    # ==========================================
    # 1. Physics & Constants
    # ==========================================
    g    = 9.81          # Local Gravity (m/s^2)
    I_sp = 500.0         # Specific Impulse (s)
    g0   = 9.81          # Standard gravity for Isp (m/s^2)
    v_e  = I_sp * g0     # Exhaust velocity (m/s) - computed automatically

    # ==========================================
    # 2. Vehicle Mass & Geometry
    # ==========================================
    m_dry  = 1250.0      # Dry mass (kg)
    m_fuel = 500.0       # Fuel mass (kg)
    T_max  = 50000.0     # Maximum thrust (N)
    T_min  = 4000.0      # Minimum throttleable thrust (N)

    rocket_height       = 95.0
    fuel_tank_height    = rocket_height / 2
    dry_cm_from_nozzle  = rocket_height / 2
    fuel_cm_from_nozzle = fuel_tank_height / 2

    # ==========================================
    # 3. Control Limits
    # ==========================================
    ALPHA_MAX_DEG = 10.0
    alpha_max     = np.deg2rad(ALPHA_MAX_DEG)  # Gimbal limit (rad)

    # ==========================================
    # 4. Simulation / Discretization
    # ==========================================
    N = 40  # Number of collocation nodes for direct collocation

    # ==========================================
    # 5. Optimization Bounds & Settings
    # ==========================================
    tf_min = 1.0   # Minimum final time (s)
    tf_max = 60.0  # Maximum final time (s)

    vx_min    = -50.0
    vx_max    =  50.0
    vy_min    = -100.0
    vy_max    =  50.0
    omega_min = -0.5
    omega_max =  0.5

    theta_max_loose  = np.pi / 4   # Loose attitude limit during flight (~45 deg)
    LANDING_NODES    = 6           # Number of final nodes with tight landing constraints
    TIGHT_THETA_DEG  = 2.0
    tight_theta      = np.deg2rad(TIGHT_THETA_DEG)
    TIGHT_ALPHA_DEG  = 2.0
    tight_alpha      = np.deg2rad(TIGHT_ALPHA_DEG)

    initial_tf_guess = 15.0  # Initial guess for solver

    # ==========================================
    # 6. Objective Function Weights
    # ==========================================
    epsilon        = 5.0     # Softening distance (m) to avoid 1/h singularity near ground
    w_time         = 10.0
    w_thrust       = 1e-10
    w_gimbal       = 0.05
    w_gimbal_rate  = 0.8
    w_theta        = 80.0
    w_alt_thrust   = 6.25e-6
    w_landing      = 4000.0
    w_v            = 50.0    # Weight for velocity penalty near ground

    # ==========================================
    # 7. Ground Clearance & Penalty Settings
    # ==========================================
    clearance_zone_factor = 2.0
    clearance_zone        = rocket_height * clearance_zone_factor

    # ==========================================
    # 8. Controllability Check Thresholds
    # ==========================================
    nozzle_ground_threshold    = -0.1   # Nozzle already touching/through ground
    hover_thrust_safety_factor = 0.6    # Must be able to provide at least this fraction of hover thrust
    dv_available_margin        = 0.6
    initial_theta_max_deg      = 25
    initial_omega_max_deg      = 35
    initial_x_max              = 800    # Maximum allowable initial downrange (m)
    dv_gravity_loss_factor     = 1.5
    dv_time_margin             = 5.0    # Crude gravity-loss time equivalent (s)

    # ==========================================
    # 9. Landing Quality Check Tolerances
    # ==========================================
    landing_pos_tol      = 0.10   # Maximum acceptable final position error (m)
    landing_vel_tol      = 0.15   # Maximum acceptable final velocity error (m/s)
    min_nozzle_tol       = -0.30  # Maximum allowed ground clip (m)
    skidding_x_threshold = 5.0    # If |x| > this while nozzle near ground --> skidding detected

    # ==========================================
    # 10. Grid test parameters
    # ==========================================
    x_grid_min  = -10.0
    x_grid_max  =  10.0
    x_grid_step =   5.0
    h_grid_min  =   0.0
    h_grid_max  = 100.0
    h_grid_step =  25.0

    # ========================================================
    # 11. Fixed velocity / attitude for all grid points
    # ========================================================
    test_vx_0    = 0.0
    test_vy_0    = 0.0
    test_theta_0 = 0.0
    test_omega_0 = 0.0