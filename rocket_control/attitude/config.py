"""
attitude/config.py
------------------
All parameters for the attitude control (rotation) simulation.
User-tunable values: initial conditions, rocket geometry, control limits.
"""


class AttitudeConfig:

    # ==========================================
    # 1. Rocket Geometry (drawing + physics)
    # ==========================================
    rocket_height    = 70
    rocket_width     = 7
    nose_height      = 15
    engine_height    = 10

    fuel_tank_height    = rocket_height / 2
    fuel_cm_from_nozzle = fuel_tank_height / 2
    dry_cm_from_nozzle  = rocket_height / 2

    # Drawing geometry (body-frame coords, origin at geometric centre)
    body_raw      = [[-7, -35], [7, -35], [7, 35], [-7, 35]]
    nose_raw      = [[-7,  35], [7,  35], [0,  50]]
    engine_attach = (0, -34)
    engine_raw    = [[-8.4, -45], [8.4, -45], [4.5, -34], [-4.5, -34]]
    flame_raw     = [[-5.6, -45], [5.6, -45], [0,   -70]]

    # ==========================================
    # 2. Control Limits
    # ==========================================
    gimbal_limit     = 10.0    # Maximum gimbal deflection (deg)
    max_gimbal_speed = 360.0   # Maximum gimbal slew rate (deg/s)

    # ==========================================
    # 3. Simulation Settings
    # ==========================================
    dt           = 0.01   # Time step (s)
    max_time     = 40.0   # Simulation wall-clock limit (s)
    post_hold    = 1.5    # Seconds of settled hold appended after manoeuvre

    # Bang-bang boundary layer width (deg) for gimbal smoothing
    boundary = 0.2

    # Settled threshold: |error| and |omega| below this → done
    settle_pos_tol = 0.2   # deg
    settle_vel_tol = 0.2   # deg/s

    # ==========================================
    # 4. Animation Settings
    # ==========================================
    fps        = 50
    step_size  = 2      # Downsample factor for animation frames
    hold_frames = 120   # Extra frozen frames prepended to animation

    # Arrow lengths for velocity / omega indicators
    arrow_length_rot   = 20          # Fixed length for rotation-only view
    arrow_length_trans_fraction = 0.2  # Fraction of fig_size for translation view

    # Figure size scale for translation view
    trans_scale_factor = 1.75   # fig_size = max(scale * max_disp, rocket_height + 30)
    trans_min_margin   = 30
