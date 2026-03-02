import numpy as np


class RocketConfig:
    """
    Central configuration for the rocket landing simulator.
    All tunable parameters, weights, bounds, geometry, tolerances,
    and initial conditions are gathered here.
    Modify only this section to tune the simulation.
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
    # 9. Animation & Visualization Parameters
    # ==========================================
    min_flame_length    = 15.0
    base_flame_length   = 35.0
    fps                 = 30
    hold_time_start     = 2.0
    hold_time_end       = 2.0
    flame_throttle_gamma = 0.5  # < 1 makes low-thrust flames relatively stronger

    visual_margin_factor      = 1.5   # Margin around rocket for plot limits
    visual_arrow_length_factor = 0.1  # Length of initial velocity arrow relative to plot span

    # Rocket geometry for drawing (local coordinates, nozzle base near (0,0))
    body_raw      = [[-7, 10], [7, 10], [7, 80], [-7, 80]]
    nose_raw      = [[-7, 80], [7, 80], [0, 95]]
    engine_attach = (0, 10)
    engine_raw    = [[-8.4, 0], [8.4, 0], [4.5, 10], [-4.5, 10]]
    flame_raw     = [[-5.6, 0], [5.6, 0], [0, -base_flame_length]]

    # Telemetry text formatting (longest text + 1)
    label_width = len("FUEL BURNED:") + 1
    value_width = 11

    # ==========================================
    # 10. Landing Quality Check Tolerances
    # ==========================================
    landing_pos_tol      = 0.10   # Maximum acceptable final position error (m)
    landing_vel_tol      = 0.15   # Maximum acceptable final velocity error (m/s)
    min_nozzle_tol       = -0.30  # Maximum allowed ground clip (m)
    skidding_x_threshold = 5.0    # If |x| > this while nozzle near ground --> skidding detected

    # ==========================================
    # 11. Initial Conditions (scenario setup)
    # ==========================================
    initial_nozzle_altitude = 160.42   # Nozzle altitude above ground at t=0 (m)
    initial_x_0  =  30.0              # Initial downrange offset (m)
    initial_vx_0 =  -8.0             # Initial horizontal velocity (m/s)
    initial_vy_0 = -30.0             # Initial vertical velocity (m/s, negative = downward)
    initial_theta = 0.0              # Initial attitude against the vertical (°)
    initial_omega = 0.0              # Initial angular velocity (positive = clockwise) (°/s)
