"""
animation.py
------------
Diagnostic plots and rocket animation for the landing simulation.
Depends only on NumPy / Matplotlib / SciPy – no CasADi.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon, FancyArrowPatch, Rectangle
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import interp1d

from .physics import calculate_com_and_I


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rotate_point(x, y, pivot_x, pivot_y, angle_deg):
    angle_rad = np.radians(angle_deg)
    nx, ny = x - pivot_x, y - pivot_y
    s, c   = np.sin(-angle_rad), np.cos(-angle_rad)
    return nx * c - ny * s + pivot_x, nx * s + ny * c + pivot_y


def _get_rotated_points(points_list, pivot, angle):
    return [_rotate_point(p[0], p[1], pivot[0], pivot[1], angle) for p in points_list]


# ---------------------------------------------------------------------------
# Diagnostic plots
# ---------------------------------------------------------------------------

def plot_diagnostics(states, controls, tf, config):
    """
    12-panel diagnostic plot: angles, rates, thrust, CoM, nozzle height, etc.

    Parameters
    ----------
    states   : ndarray (N+1, 7)
    controls : ndarray (N+1, 2)
    tf       : float
    config   : RocketConfig (or subclass)
    """
    times = np.linspace(0, tf, states.shape[0])

    angles_deg  = np.rad2deg(states[:, 4])
    omegas_deg  = np.rad2deg(states[:, 5])
    gimbals_deg = np.rad2deg(controls[:, 1])
    masses      = states[:, 6]
    thrusts     = controls[:, 0]

    coms    = np.array([calculate_com_and_I(m, config)[0] for m in masses])
    I_zs    = np.array([calculate_com_and_I(m, config)[1] for m in masses])
    torques = coms * thrusts * np.sin(np.radians(gimbals_deg))

    x_coms   = states[:, 0]
    y_nozzle = states[:, 1] - coms
    vxs      = states[:, 2]
    vys      = states[:, 3]

    fig, axs = plt.subplots(12, 1, figsize=(10, 26), sharex=True)
    plt.subplots_adjust(hspace=0.28)

    plot_data = [
        angles_deg, omegas_deg, gimbals_deg, masses, thrusts, torques,
        coms, I_zs, x_coms, y_nozzle, vxs, vys
    ]
    labels = [
        r'$\theta$ (deg)', r'$\omega$ (deg/s)', 'Gimbal (deg)', 'Mass (kg)',
        'Thrust (N)', 'Torque (N·m)', 'CoM from nozzle (m)', 'MoI (kg·m²)',
        'X position (m)', 'Nozzle height (m)', 'Vx (m/s)', 'Vy (m/s)'
    ]
    colors = [
        'cyan', 'lime', 'orange', 'red', 'blue', 'purple',
        'magenta', 'gold', 'teal', 'navy', 'brown', 'olive'
    ]

    for i, ax in enumerate(axs):
        ax.plot(times, plot_data[i], color=colors[i], lw=1.4)
        ax.set_ylabel(labels[i], fontsize=11)
        ax.grid(True, alpha=0.18, ls='--')
        ax.tick_params(labelsize=9)

    axs[9].axhspan(-5, 0, facecolor='gray', alpha=0.15)
    axs[9].axhline(0, color='darkred', lw=1.2, ls='--', alpha=0.7)

    plt.suptitle("Post-Flight Analysis", fontsize=16, y=0.995)
    plt.xlabel("Time (s)", fontsize=11)
    plt.show()


# ---------------------------------------------------------------------------
# Main animation
# ---------------------------------------------------------------------------

def animate_results(states, controls, tf, config, output_path="results/landing.mp4"):
    """
    Render and save a full rocket landing animation.

    Parameters
    ----------
    states      : ndarray (N+1, 7)
    controls    : ndarray (N+1, 2)
    tf          : float
    config      : RocketConfig (or subclass)
    output_path : str  – path for the .mp4 file
    """
    N         = states.shape[0] - 1
    sim_times = np.linspace(0, tf, N + 1)

    fps              = config.fps
    hold_time_start  = config.hold_time_start
    hold_time_end    = config.hold_time_end
    base_flame_length = config.base_flame_length
    flame_gamma      = config.flame_throttle_gamma

    num_hold_start  = int(hold_time_start * fps)
    num_hold_end    = int(hold_time_end   * fps)
    num_anim_frames = int(tf * fps) + 1
    anim_times_raw  = np.linspace(0, tf, num_anim_frames)
    dt_anim         = anim_times_raw[1] - anim_times_raw[0] if num_anim_frames > 1 else 0

    def _interp(arr):
        return interp1d(sim_times, arr)(anim_times_raw)

    x_coms  = _interp(states[:, 0])
    y_coms  = _interp(states[:, 1])
    vxs     = _interp(states[:, 2])
    vys     = _interp(states[:, 3])
    angles  = np.rad2deg(_interp(states[:, 4]))
    omegas  = np.rad2deg(_interp(states[:, 5]))
    masses  = _interp(states[:, 6])
    thrusts = _interp(controls[:, 0])
    gimbals = np.rad2deg(_interp(controls[:, 1]))

    thrusts[-1] = 0
    gimbals[-1] = 0

    coms = np.array([calculate_com_and_I(m, config)[0] for m in masses])

    # Add start hold
    def _prepend(arr, val=None):
        v = arr[0] if val is None else val
        return np.concatenate((np.full(num_hold_start, v), arr))

    def _append(arr, val=None):
        v = arr[-1] if val is None else val
        return np.append(arr, np.full(num_hold_end, v))

    x_coms  = _append(_prepend(x_coms))
    y_coms  = _append(_prepend(y_coms))
    vxs     = _append(_prepend(vxs),   val=0.0)
    vys     = _append(_prepend(vys),   val=0.0)
    angles  = _append(_prepend(angles))
    omegas  = _append(_prepend(omegas), val=0.0)
    masses  = _append(_prepend(masses))
    thrusts = _append(_prepend(thrusts), val=0.0)
    gimbals = _append(_prepend(gimbals))
    coms    = _append(_prepend(coms))

    t_start = np.linspace(-hold_time_start, -dt_anim, num_hold_start)
    t_end   = np.linspace(tf + dt_anim,     tf + hold_time_end, num_hold_end)
    anim_times = np.concatenate((t_start, anim_times_raw, t_end))
    num_anim_frames += num_hold_start + num_hold_end

    # ---- Plot limits ----
    margin  = config.rocket_height * config.visual_margin_factor
    x_min   = min(x_coms) - margin
    x_max   = max(x_coms) + margin
    y_min   = -config.rocket_height
    y_max   = max(y_coms) + margin
    x_span  = x_max - x_min
    y_span  = y_max - y_min
    if x_span < y_span:
        extra = (y_span - x_span) / 2;  x_min -= extra;  x_max += extra
    elif y_span < x_span:
        extra = (x_span - y_span) / 2;  y_min -= extra;  y_max += extra

    # ---- Figure setup ----
    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(12, 8),
                                       gridspec_kw={'width_ratios': [2, 1]})
    ax.set_xlim(x_min, x_max);  ax.set_ylim(y_min, y_max)
    ax.set_aspect('equal');      ax.axis('off')

    cmap = LinearSegmentedColormap.from_list("sky", ["#102C57", "#000000"])
    ax.imshow(np.linspace(0, 1, 100).reshape(-1, 1),
              extent=[x_min, x_max, y_min, y_max],
              origin='lower', cmap=cmap, aspect='auto')
    ax.axhspan(y_min, 0, color="#888888", zorder=1)
    ax.axhline(0, color="#B1B1B1", linewidth=2, zorder=2)

    ix_line, = ax.plot([x_coms[0], x_coms[0]], [0, y_max],
                       color='red',   ls='--', lw=2, zorder=1.5, alpha=0.4, label="Initial x")
    tx_line, = ax.plot([0, 0],         [0, y_max],
                       color='green', ls='--', lw=2, zorder=1.5, alpha=0.4, label="Target x")

    ax.add_patch(Rectangle((-20, -5), 40, 5, fc="gray", ec="black", zorder=3))

    body_patch   = Polygon([[0, 0]], fc="white",    ec="black", zorder=11)
    nose_patch   = Polygon([[0, 0]], fc="white",    ec="black", zorder=12)
    engine_patch = Polygon([[0, 0]], fc="#D0D0D0",  ec="black", zorder=10)
    flame_patch  = Polygon([[0, 0]], fc="orange",   ec="red",   lw=1.5, zorder=9, visible=False)
    for p in [body_patch, nose_patch, engine_patch, flame_patch]:
        ax.add_patch(p)

    arrow_length = abs(x_max - x_min) * config.visual_arrow_length_factor
    vel_arrow = FancyArrowPatch(
        (0, 0), (0, 0),
        arrowstyle="simple, head_width=9, head_length=9, tail_width=4",
        mutation_scale=1.5, facecolor='cyan', edgecolor='black',
        linewidth=1.5, zorder=20, visible=False, label="Initial Velocity"
    )
    ax.add_patch(vel_arrow)

    pv1 = ax.text(0.12, 0.06, '', transform=ax.transAxes, color='white',
                  fontsize=13, family='monospace', fontweight='bold', va='bottom', ha='left')
    pv2 = ax.text(0.12, 0.02, '', transform=ax.transAxes, color='white',
                  fontsize=13, family='monospace', fontweight='bold', va='bottom', ha='left')

    ax_info.set_facecolor('white');  ax_info.axis('off')
    lb, fs = -0.2, 15
    txt_alt       = ax_info.text(lb, 0.8, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_downrange = ax_info.text(lb, 0.7, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_theta     = ax_info.text(lb, 0.6, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_omega     = ax_info.text(lb, 0.5, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_gimbal    = ax_info.text(lb, 0.4, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_thrust    = ax_info.text(lb, 0.3, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_fuel      = ax_info.text(lb, 0.2, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    txt_time      = ax_info.text(lb, 0.1, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_info.text(0.5, 0.9, "TELEMETRY", color='black', fontsize=fs + 4, fontweight='bold', ha="center")

    # Geometry pre-processing
    body_raw      = config.body_raw
    nose_raw      = config.nose_raw
    engine_attach = config.engine_attach
    engine_rel    = [(ex - engine_attach[0], ey - engine_attach[1]) for ex, ey in config.engine_raw]
    flame_rel     = [(fx - engine_attach[0], fy - engine_attach[1]) for fx, fy in config.flame_raw]
    LW = config.label_width
    VW = config.value_width

    def _animate(i):
        t_val   = angles[i]
        g_val   = gimbals[i]
        thr_val = thrusts[i]
        cx, cy  = x_coms[i], y_coms[i]
        com     = coms[i]

        body_local = [[px, py - com] for px, py in body_raw]
        nose_local = [[px, py - com] for px, py in nose_raw]
        eng_attach_local = (0, 10 - com)

        body_rot   = _get_rotated_points(body_local, (0, 0), t_val)
        nose_rot   = _get_rotated_points(nose_local, (0, 0), t_val)
        eng_gimb   = _get_rotated_points(engine_rel, (0, 0), g_val)
        eng_local  = [(ex + eng_attach_local[0], ey + eng_attach_local[1]) for ex, ey in eng_gimb]
        engine_rot = _get_rotated_points(eng_local,  (0, 0), t_val)

        throttle        = thr_val / config.T_max if config.T_max > 0 else 0.0
        fl              = base_flame_length * (throttle ** flame_gamma)
        flame_scaled    = [(fx, fy * (fl / base_flame_length)) for fx, fy in flame_rel]
        flame_gimb      = _get_rotated_points(flame_scaled, (0, 0), g_val)
        flame_local     = [(fx + eng_attach_local[0], fy + eng_attach_local[1]) for fx, fy in flame_gimb]
        flame_rot       = _get_rotated_points(flame_local,  (0, 0), t_val)

        body_patch.set_xy(  [(px + cx, py + cy) for px, py in body_rot])
        nose_patch.set_xy(  [(px + cx, py + cy) for px, py in nose_rot])
        engine_patch.set_xy([(px + cx, py + cy) for px, py in engine_rot])
        flame_patch.set_xy( [(px + cx, py + cy) for px, py in flame_rot])
        flame_patch.set_visible(thr_val > 0)

        txt_alt.set_text(      f"{'ALTITUDE:':>{LW}}{y_coms[i] - coms[i]:>{VW}.2f} m")
        txt_downrange.set_text(f"{'DOWNRANGE:':>{LW}}{x_coms[i]:>{VW}.2f} m")
        txt_theta.set_text(    f"{'THETA:':>{LW}}{t_val:>{VW}.2f} °")
        txt_omega.set_text(    f"{'OMEGA:':>{LW}}{omegas[i]:>{VW}.2f} °/s")
        txt_gimbal.set_text(   f"{'GIMBAL:':>{LW}}{g_val:>{VW}.2f} °")
        txt_thrust.set_text(   f"{'THRUST:':>{LW}}{thr_val:>{VW}.0f} N")
        txt_fuel.set_text(     f"{'FUEL:':>{LW}}{masses[i] - config.m_dry:>{VW}.2f} kg")
        txt_time.set_text(     f"{'TIME:':>{LW}}{max(0, min(anim_times[i], tf)):>{VW}.2f} s")

        pv1.set_text(f" X: {x_coms[i]:10.2f} m   Alt: {y_coms[i] - coms[i]:10.2f} m")
        pv2.set_text(f"Vx: {vxs[i]:10.2f} m/s  Vy: {vys[i]:10.2f} m/s")

        show_arrow = anim_times[i] < 0
        speed = np.sqrt(vxs[i] ** 2 + vys[i] ** 2)
        if show_arrow and speed > 1e-3:
            dx = (vxs[i] / speed) * arrow_length
            dy = (vys[i] / speed) * arrow_length
            vel_arrow.set_positions((cx, cy), (cx + dx, cy + dy))
            vel_arrow.set_visible(True)
        else:
            vel_arrow.set_visible(False)

        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
        if show_arrow and speed > 1e-3:
            ax.legend([ix_line, tx_line, vel_arrow], ["Initial x", "Target x", "Initial v"],
                      loc='upper right', fontsize=14, framealpha=0.7)
        else:
            ax.legend([ix_line, tx_line], ["Initial x", "Target x"],
                      loc='upper right', fontsize=14, framealpha=0.7)

        return (body_patch, nose_patch, engine_patch, flame_patch,
                txt_alt, txt_downrange, txt_theta, txt_omega, txt_gimbal,
                txt_thrust, txt_fuel, txt_time, pv1, pv2, vel_arrow)

    ani = FuncAnimation(fig, _animate, frames=range(num_anim_frames),
                        interval=1000 // fps, blit=True)
    print("Saving animation...")
    writer = FFMpegWriter(fps=fps, bitrate=2500)
    ani.save(output_path, writer=writer)
    print("Done.")
    plt.show()
