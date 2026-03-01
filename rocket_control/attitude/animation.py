"""
attitude/animation.py
---------------------
Animation for the attitude control simulation.
Three modes: rotation-only, translation, dual side-by-side.
All modes share the same geometry helpers and are driven by a SimResult.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon, FancyArrowPatch
from matplotlib.colors import LinearSegmentedColormap

from .physics import wrap_angle


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _rotate_point(x, y, pivot_x, pivot_y, angle_deg):
    angle_rad = np.radians(angle_deg)
    nx, ny = x - pivot_x, y - pivot_y
    s, c   = np.sin(-angle_rad), np.cos(-angle_rad)
    return nx * c - ny * s + pivot_x, nx * s + ny * c + pivot_y


def _get_rotated_points(points_list, pivot, angle):
    return [_rotate_point(p[0], p[1], pivot[0], pivot[1], angle) for p in points_list]


def _sky_bg(ax, extent):
    cmap = LinearSegmentedColormap.from_list("sky", ["#102C57", "#000000"])
    ax.imshow(np.linspace(0, 1, 100).reshape(-1, 1),
              extent=extent, origin='lower', cmap=cmap, aspect='auto')


def _add_rocket_patches(ax):
    """Create and add the four body patches; return them."""
    body   = Polygon([(0, 0)], fc="white",   ec="black", zorder=11)
    nose   = Polygon([(0, 0)], fc="white",   ec="black", zorder=12)
    engine = Polygon([(0, 0)], fc="#D0D0D0", ec="black", zorder=10)
    flame  = Polygon([(0, 0)], fc="orange",  ec="red",   lw=1.5, zorder=9)
    for p in [body, nose, engine, flame]:
        ax.add_patch(p)
    return body, nose, engine, flame


def _anim_frames(result, config):
    s = config.step_size
    return [0] * config.hold_frames + list(range(0, len(result.times), s))


# ---------------------------------------------------------------------------
# Rotation-only mode
# ---------------------------------------------------------------------------

def _animate_rotation(result, theta_0, theta_target, omega_0,
                       dry_mass, fuel_mass, config, output_path):

    fig_size = config.rocket_height + 40
    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(12, 8),
                                       gridspec_kw={'width_ratios': [2, 1]})
    ax.set_xlim(-fig_size, fig_size);  ax.set_ylim(-fig_size, fig_size)
    ax.set_aspect('equal');             ax.axis('off')
    _sky_bg(ax, [-fig_size, fig_size, -fig_size, fig_size])

    body_p, nose_p, engine_p, flame_p = _add_rocket_patches(ax)

    rad1 = np.radians(theta_0)
    rad2 = np.radians(theta_target)
    init_line, = ax.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)],
                         color='red',   ls='--', alpha=0.6, zorder=5, label="Initial Angle")
    tgt_line,  = ax.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)],
                         color='green', ls='--', alpha=0.7, zorder=5, label="Target Angle")

    arrow_len = config.arrow_length_rot
    arrow_p   = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20,
                                color='yellow', visible=False)
    ax.add_patch(arrow_p)
    ax.text(0.5, 1.02, f"Initial Angular Velocity: {omega_0:.2f} deg/s",
            family='monospace', fontweight='bold', ha='center', va='bottom',
            color='black', fontsize=12, transform=ax.transAxes)

    ax_info.set_facecolor('white');  ax_info.axis('off')
    lb, fs = -0.2, 15
    LW, VW = len("FUEL BURNED:") + 1, 11
    tx_angle  = ax_info.text(lb, 0.8, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_omega  = ax_info.text(lb, 0.7, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_gimbal = ax_info.text(lb, 0.6, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_thrust = ax_info.text(lb, 0.5, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_fuel   = ax_info.text(lb, 0.4, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_burned = ax_info.text(lb, 0.3, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_torque = ax_info.text(lb, 0.2, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_time   = ax_info.text(lb, 0.1, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_info.text(0.5, 0.9, "LIVE TELEMETRY", color='black', fontsize=fs + 4,
                 fontweight='bold', ha="center")

    cfg      = config
    eng_rel  = [(ex - cfg.engine_attach[0], ey - cfg.engine_attach[1]) for ex, ey in cfg.engine_raw]
    flame_rel = [(fx - cfg.engine_attach[0], fy - cfg.engine_attach[1]) for fx, fy in cfg.flame_raw]
    dt       = cfg.dt

    def _frame(i):
        t_val   = result.angles[i]
        g_val   = result.gimbals[i]
        thr_val = result.thrusts[i]

        eng_rot   = _get_rotated_points(cfg.engine_raw, cfg.engine_attach, g_val)
        flame_rot = _get_rotated_points(cfg.flame_raw,  cfg.engine_attach, g_val)
        body_p.set_xy(  _get_rotated_points(cfg.body_raw, (0, 0), t_val))
        nose_p.set_xy(  _get_rotated_points(cfg.nose_raw, (0, 0), t_val))
        engine_p.set_xy(_get_rotated_points(eng_rot,      (0, 0), t_val))
        flame_p.set_xy( _get_rotated_points(flame_rot,    (0, 0), t_val))
        flame_p.set_visible(thr_val > 0 and i != 0)

        tx_angle.set_text(  f"{'BODY TILT:':>{LW}}{t_val:>{VW}.2f} [°]")
        tx_omega.set_text(  f"{'ANG VEL:':>{LW}}{result.omegas[i]:>{VW}.2f} [°/s]")
        tx_gimbal.set_text( f"{'GIMBAL:':>{LW}}{g_val:>{VW}.2f} [°]")
        tx_thrust.set_text( f"{'THRUST:':>{LW}}{thr_val:>{VW}.0f} [N]")
        tx_fuel.set_text(   f"{'FUEL LEFT:':>{LW}}{result.masses[i] - dry_mass:>{VW}.2f} [kg]")
        tx_burned.set_text( f"{'FUEL USED:':>{LW}}{fuel_mass - (result.masses[i] - dry_mass):>{VW}.2f} [kg]")
        tx_torque.set_text( f"{'TORQUE:':>{LW}}{result.torques[i]:>{VW}.0f} [N·m]")
        tx_time.set_text(   f"{'TIME:':>{LW}}{min(result.times[i], result.final_time):>{VW}.2f} [s]")

        leg = ax.get_legend()
        if leg is not None:
            leg.remove()

        show_arrow = result.times[i] < dt and abs(omega_0) > 1e-6
        if show_arrow:
            tip = _get_rotated_points([(0, 50)], (0, 0), t_val)[0]
            tx, ty = tip
            r = np.sqrt(tx ** 2 + ty ** 2) or 1
            dx = -np.sign(omega_0) * (-ty / r) * arrow_len
            dy = -np.sign(omega_0) * (tx  / r) * arrow_len
            arrow_p.set_positions((tx, ty), (tx + dx, ty + dy))
            arrow_p.set_visible(True)
            ax.legend([init_line, tgt_line, arrow_p],
                      ["Initial Angle", "Target Angle", r"Initial $\omega$"],
                      loc='upper right', fontsize=15, framealpha=0.7)
        else:
            arrow_p.set_visible(False)
            ax.legend([init_line, tgt_line], ["Initial Angle", "Target Angle"],
                      loc='upper right', fontsize=15, framealpha=0.7)

        return (body_p, nose_p, engine_p, flame_p,
                tx_angle, tx_omega, tx_gimbal, tx_thrust,
                tx_fuel, tx_burned, tx_torque, tx_time, arrow_p)

    frames = _anim_frames(result, config)
    ani    = FuncAnimation(fig, _frame, frames=frames, interval=20, blit=True)
    print("Saving rotation-only animation...")
    ani.save(output_path, writer=FFMpegWriter(fps=config.fps, bitrate=2500))
    print(f"Done. Saved to {output_path}")
    plt.close()


# ---------------------------------------------------------------------------
# Translation mode
# ---------------------------------------------------------------------------

def _animate_translation(result, theta_0, theta_target, omega_0,
                          dry_mass, fuel_mass, config, output_path):

    max_disp = max(
        max((abs(x) for x in result.x_coms), default=0),
        max((abs(y) for y in result.y_coms), default=0),
    )
    fig_size = max(config.trans_scale_factor * max_disp,
                   config.rocket_height + config.trans_min_margin)
    arrow_len = config.arrow_length_trans_fraction * fig_size

    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(12, 8),
                                       gridspec_kw={'width_ratios': [2, 1]})
    ax.set_xlim(-fig_size, fig_size);  ax.set_ylim(-fig_size + 25, fig_size)
    ax.set_aspect('equal');             ax.axis('off')
    _sky_bg(ax, [-fig_size, fig_size, -fig_size, fig_size])

    body_p, nose_p, engine_p, flame_p = _add_rocket_patches(ax)

    rad1 = np.radians(theta_0)
    rad2 = np.radians(theta_target)
    init_line, = ax.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)],
                         color='red',   ls='--', alpha=0.6, zorder=5, label="Initial Angle")
    tgt_line,  = ax.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)],
                         color='green', ls='--', alpha=0.7, zorder=5, label="Target Angle")

    arrow_p     = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20,
                                  color='yellow', visible=False, zorder=20)
    vel_arrow_p = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20,
                                  color='cyan',   visible=False, zorder=20)
    ax.add_patch(arrow_p);  ax.add_patch(vel_arrow_p)
    ax.text(0.5, 1.02, f"Initial Angular Velocity: {omega_0:.2f} deg/s",
            family='monospace', fontweight='bold', ha='center', va='bottom',
            color='black', fontsize=12, transform=ax.transAxes)

    ax_info.set_facecolor('white');  ax_info.axis('off')
    lb, fs = -0.2, 15
    LW, VW = len("FUEL BURNED:") + 1, 11
    tx_angle  = ax_info.text(lb, 0.8, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_omega  = ax_info.text(lb, 0.7, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_gimbal = ax_info.text(lb, 0.6, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_thrust = ax_info.text(lb, 0.5, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_fuel   = ax_info.text(lb, 0.4, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_burned = ax_info.text(lb, 0.3, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_torque = ax_info.text(lb, 0.2, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_time   = ax_info.text(lb, 0.1, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_info.text(0.5, 0.9, "LIVE TELEMETRY", color='black', fontsize=fs + 4,
                 fontweight='bold', ha="center")

    pv1 = ax.text(0.05, 0.06, '', transform=ax.transAxes, color='white',
                  fontsize=13, family='monospace', fontweight='bold', va='bottom', ha='left')
    pv2 = ax.text(0.05, 0.02, '', transform=ax.transAxes, color='white',
                  fontsize=13, family='monospace', fontweight='bold', va='bottom', ha='left')

    cfg       = config
    eng_rel   = [(ex - cfg.engine_attach[0], ey - cfg.engine_attach[1]) for ex, ey in cfg.engine_raw]
    flame_rel = [(fx - cfg.engine_attach[0], fy - cfg.engine_attach[1]) for fx, fy in cfg.flame_raw]
    dt        = cfg.dt

    def _frame(i):
        t_val = result.unwrapped_angles[i]
        g_val = result.gimbals[i]
        thr_val = result.thrusts[i]
        cx, cy  = result.x_coms[i], result.y_coms[i]
        local_com_y = result.coms[i] - cfg.dry_cm_from_nozzle

        body_local = [(bx, by - local_com_y) for bx, by in cfg.body_raw]
        nose_local = [(nx, ny - local_com_y) for nx, ny in cfg.nose_raw]
        eng_attach_local = (cfg.engine_attach[0], cfg.engine_attach[1] - local_com_y)

        eng_g   = _get_rotated_points(eng_rel,   (0, 0), g_val)
        flame_g = _get_rotated_points(flame_rel, (0, 0), g_val)
        eng_local   = [(ex + eng_attach_local[0], ey + eng_attach_local[1]) for ex, ey in eng_g]
        flame_local = [(fx + eng_attach_local[0], fy + eng_attach_local[1]) for fx, fy in flame_g]

        body_p.set_xy(  [(px + cx, py + cy) for px, py in _get_rotated_points(body_local,  (0, 0), t_val)])
        nose_p.set_xy(  [(px + cx, py + cy) for px, py in _get_rotated_points(nose_local,  (0, 0), t_val)])
        engine_p.set_xy([(px + cx, py + cy) for px, py in _get_rotated_points(eng_local,   (0, 0), t_val)])
        flame_p.set_xy( [(px + cx, py + cy) for px, py in _get_rotated_points(flame_local, (0, 0), t_val)])
        flame_p.set_visible(thr_val > 0 and i != 0)

        tx_angle.set_text(  f"{'BODY TILT:':>{LW}}{wrap_angle(t_val):>{VW}.2f} [°]")
        tx_omega.set_text(  f"{'ANG VEL:':>{LW}}{result.omegas[i]:>{VW}.2f} [°/s]")
        tx_gimbal.set_text( f"{'GIMBAL:':>{LW}}{g_val:>{VW}.2f} [°]")
        tx_thrust.set_text( f"{'THRUST:':>{LW}}{thr_val:>{VW}.0f} [N]")
        tx_fuel.set_text(   f"{'FUEL LEFT:':>{LW}}{result.masses[i] - dry_mass:>{VW}.2f} [kg]")
        tx_burned.set_text( f"{'FUEL USED:':>{LW}}{fuel_mass - (result.masses[i] - dry_mass):>{VW}.2f} [kg]")
        tx_torque.set_text( f"{'TORQUE:':>{LW}}{result.torques[i]:>{VW}.0f} [N·m]")
        tx_time.set_text(   f"{'TIME:':>{LW}}{min(result.times[i], result.final_time):>{VW}.2f} [s]")
        pv1.set_text(f"   X: {cx:10.2f} m      Y: {cy:10.2f} m")
        pv2.set_text(f"  Vx: {result.vxs[i]:10.2f} m/s   Vy: {result.vys[i]:10.2f} m/s")

        leg = ax.get_legend()
        if leg is not None:
            leg.remove()

        show_arrow = result.times[i] < dt and abs(omega_0) > 1e-6
        if show_arrow:
            tip = _get_rotated_points([(0, 50 - local_com_y)], (0, 0), t_val)[0]
            tx2, ty2 = tip[0] + cx, tip[1] + cy
            r = np.sqrt(tx2 ** 2 + ty2 ** 2) or 1
            dx = -np.sign(omega_0) * (-ty2 / r) * arrow_len
            dy = -np.sign(omega_0) * (tx2  / r) * arrow_len
            arrow_p.set_positions((tx2, ty2), (tx2 + dx, ty2 + dy))
            arrow_p.set_visible(True)
            ax.legend([init_line, tgt_line, arrow_p],
                      ["Initial Angle", "Target Angle", r"Initial $\omega$"],
                      loc='upper right', fontsize=15, framealpha=0.7)
        else:
            arrow_p.set_visible(False)
            if result.times[i] >= result.final_time:
                speed = np.sqrt(result.vxs[i] ** 2 + result.vys[i] ** 2)
                if speed > 1e-6:
                    gc  = _rotate_point(0, -local_com_y, 0, 0, t_val)
                    bx2 = gc[0] + cx;  by2 = gc[1] + cy
                    vel_arrow_p.set_positions(
                        (bx2, by2),
                        (bx2 + result.vxs[i] / speed * arrow_len,
                         by2 + result.vys[i] / speed * arrow_len))
                    vel_arrow_p.set_visible(True)
                    ax.legend([init_line, tgt_line, vel_arrow_p],
                              ["Initial Angle", "Target Angle", "Velocity"],
                              loc='upper right', fontsize=15, framealpha=0.7)
                else:
                    vel_arrow_p.set_visible(False)
                    ax.legend([init_line, tgt_line], ["Initial Angle", "Target Angle"],
                              loc='upper right', fontsize=15, framealpha=0.7)
            else:
                vel_arrow_p.set_visible(False)
                ax.legend([init_line, tgt_line], ["Initial Angle", "Target Angle"],
                          loc='upper right', fontsize=15, framealpha=0.7)

        return (body_p, nose_p, engine_p, flame_p,
                tx_angle, tx_omega, tx_gimbal, tx_thrust,
                tx_fuel, tx_burned, tx_torque, tx_time,
                arrow_p, vel_arrow_p, pv1, pv2)

    frames = _anim_frames(result, config)
    ani    = FuncAnimation(fig, _frame, frames=frames, interval=20, blit=True)
    print("Saving translation animation...")
    ani.save(output_path, writer=FFMpegWriter(fps=config.fps, bitrate=2500))
    print(f"Done. Saved to {output_path}")
    plt.close()


# ---------------------------------------------------------------------------
# Dual side-by-side mode
# ---------------------------------------------------------------------------

def _animate_dual(result, theta_0, theta_target, omega_0,
                  dry_mass, fuel_mass, config, output_path):

    max_disp = max(
        max((abs(x) for x in result.x_coms), default=0),
        max((abs(y) for y in result.y_coms), default=0),
    )
    fig_size_rot   = config.rocket_height + 40
    fig_size_trans = max(config.trans_scale_factor * max_disp,
                         config.rocket_height + config.trans_min_margin)
    arrow_len_rot   = config.arrow_length_rot
    arrow_len_trans = config.arrow_length_trans_fraction * fig_size_trans

    fig = plt.figure(figsize=(19.5, 13.5))
    gs  = fig.add_gridspec(2, 2, height_ratios=[1, 0.28],
                            width_ratios=[1, 1], wspace=0.08, hspace=0.15)
    ax_rot   = fig.add_subplot(gs[0, 0])
    ax_trans = fig.add_subplot(gs[0, 1])
    ax_tele  = fig.add_subplot(gs[1, :])

    # ---- Rotation view ----
    ax_rot.set_xlim(-fig_size_rot, fig_size_rot)
    ax_rot.set_ylim(-fig_size_rot, fig_size_rot)
    ax_rot.set_aspect('equal');  ax_rot.axis('off')
    ax_rot.set_title("Rotation-Only View", fontsize=20, pad=10)
    _sky_bg(ax_rot, [-fig_size_rot, fig_size_rot, -fig_size_rot, fig_size_rot])
    bp_r, np_r, ep_r, fp_r = _add_rocket_patches(ax_rot)

    rad1 = np.radians(theta_0);  rad2 = np.radians(theta_target)
    il_r, = ax_rot.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)],
                         color='red',   ls='--', alpha=0.6, zorder=5)
    tl_r, = ax_rot.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)],
                         color='green', ls='--', alpha=0.7, zorder=5)
    arr_r = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20,
                            color='yellow', visible=False)
    ax_rot.add_patch(arr_r)

    # ---- Translation view ----
    ax_trans.set_xlim(-fig_size_trans, fig_size_trans)
    ax_trans.set_ylim(-fig_size_trans + 25, fig_size_trans)
    ax_trans.set_aspect('equal');  ax_trans.axis('off')
    ax_trans.set_title("Full View (Rotation + Translation)", fontsize=20, pad=10)
    _sky_bg(ax_trans, [-fig_size_trans, fig_size_trans, -fig_size_trans, fig_size_trans])
    bp_t, np_t, ep_t, fp_t = _add_rocket_patches(ax_trans)

    il_t, = ax_trans.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)],
                           color='red',   ls='--', alpha=0.6, zorder=5)
    tl_t, = ax_trans.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)],
                           color='green', ls='--', alpha=0.7, zorder=5)
    arr_t     = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20,
                                color='yellow', visible=False, zorder=20)
    vel_arr_t = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20,
                                color='cyan',   visible=False, zorder=20)
    ax_trans.add_patch(arr_t);  ax_trans.add_patch(vel_arr_t)

    # ---- Telemetry ----
    ax_tele.set_facecolor('white');  ax_tele.axis('off')
    ax_tele.text(0.5, 0.95, "TELEMETRY", color='black', fontsize=24,
                 fontweight='bold', ha='center')
    ax_tele.text(0.5, 1.13, rf"Initial $\omega_0$ = {omega_0:.2f} °/s",
                 color='black', fontsize=16, ha='center')
    fs, VW = 20, 11
    ax_tele.text(0.04, 0.75, "BODY TILT:", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.04, 0.50, "ANG VEL: ", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.04, 0.25, "GIMBAL:  ", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.39, 0.75, "THRUST:   ", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.39, 0.50, "FUEL LEFT:", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.39, 0.25, "TIME:     ", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.72, 0.75, "X:   ", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.72, 0.50, "Y:   ", color='black', fontsize=fs, family='monospace', fontweight='bold')
    ax_tele.text(0.72, 0.25, "VEL:", color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_angle  = ax_tele.text(0.19, 0.75, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_omega  = ax_tele.text(0.19, 0.50, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_gimbal = ax_tele.text(0.19, 0.25, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_thrust = ax_tele.text(0.54, 0.75, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_fuel   = ax_tele.text(0.54, 0.50, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_time   = ax_tele.text(0.54, 0.25, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_x      = ax_tele.text(0.82, 0.75, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_y      = ax_tele.text(0.82, 0.50, '', color='black', fontsize=fs, family='monospace', fontweight='bold')
    tx_speed  = ax_tele.text(0.82, 0.25, '', color='black', fontsize=fs, family='monospace', fontweight='bold')

    cfg       = config
    eng_rel   = [(ex - cfg.engine_attach[0], ey - cfg.engine_attach[1]) for ex, ey in cfg.engine_raw]
    flame_rel = [(fx - cfg.engine_attach[0], fy - cfg.engine_attach[1]) for fx, fy in cfg.flame_raw]
    dt        = cfg.dt

    def _frame(i):
        t_wrapped   = result.angles[i]
        t_val       = result.unwrapped_angles[i]
        g_val       = result.gimbals[i]
        thr_val     = result.thrusts[i]
        cx, cy      = result.x_coms[i], result.y_coms[i]
        local_com_y = result.coms[i] - cfg.dry_cm_from_nozzle

        # Rotation-only view
        er = _get_rotated_points(cfg.engine_raw, cfg.engine_attach, g_val)
        fr = _get_rotated_points(cfg.flame_raw,  cfg.engine_attach, g_val)
        bp_r.set_xy(_get_rotated_points(cfg.body_raw, (0, 0), t_wrapped))
        np_r.set_xy(_get_rotated_points(cfg.nose_raw, (0, 0), t_wrapped))
        ep_r.set_xy(_get_rotated_points(er,            (0, 0), t_wrapped))
        fp_r.set_xy(_get_rotated_points(fr,            (0, 0), t_wrapped))
        fp_r.set_visible(thr_val > 0 and i != 0)

        # Translation view
        body_local = [(bx, by - local_com_y) for bx, by in cfg.body_raw]
        nose_local = [(nx, ny - local_com_y) for nx, ny in cfg.nose_raw]
        ea_local   = (cfg.engine_attach[0], cfg.engine_attach[1] - local_com_y)
        eg  = _get_rotated_points(eng_rel,   (0, 0), g_val)
        fg  = _get_rotated_points(flame_rel, (0, 0), g_val)
        el  = [(ex + ea_local[0], ey + ea_local[1]) for ex, ey in eg]
        fl  = [(fx + ea_local[0], fy + ea_local[1]) for fx, fy in fg]
        bp_t.set_xy([(px + cx, py + cy) for px, py in _get_rotated_points(body_local, (0, 0), t_val)])
        np_t.set_xy([(px + cx, py + cy) for px, py in _get_rotated_points(nose_local, (0, 0), t_val)])
        ep_t.set_xy([(px + cx, py + cy) for px, py in _get_rotated_points(el,          (0, 0), t_val)])
        fp_t.set_xy([(px + cx, py + cy) for px, py in _get_rotated_points(fl,          (0, 0), t_val)])
        fp_t.set_visible(thr_val > 0 and i != 0)

        # Telemetry
        speed = np.sqrt(result.vxs[i] ** 2 + result.vys[i] ** 2)
        tx_angle.set_text( f"{t_wrapped:>{VW}.2f} °")
        tx_omega.set_text( f"{result.omegas[i]:>{VW}.2f} °/s")
        tx_gimbal.set_text(f"{g_val:>{VW}.2f} °")
        tx_thrust.set_text(f"{thr_val:>{VW}.0f} N")
        tx_fuel.set_text(  f"{result.masses[i] - dry_mass:>{VW}.2f} kg")
        tx_time.set_text(  f"{min(result.times[i], result.final_time):>{VW}.2f} s")
        tx_x.set_text(     f"{cx:>{VW}.2f} m")
        tx_y.set_text(     f"{cy:>{VW}.2f} m")
        tx_speed.set_text( f"{speed:>{VW}.2f} m/s")

        show_arrow = result.times[i] < dt and abs(omega_0) > 1e-6

        # Rotation-only arrows
        leg = ax_rot.get_legend()
        if leg is not None: leg.remove()
        if show_arrow:
            tip  = _get_rotated_points([(0, 50)], (0, 0), t_wrapped)[0]
            tx2, ty2 = tip
            r = np.sqrt(tx2 ** 2 + ty2 ** 2) or 1
            dx = -np.sign(omega_0) * (-ty2 / r) * arrow_len_rot
            dy = -np.sign(omega_0) * (tx2  / r) * arrow_len_rot
            arr_r.set_positions((tx2, ty2), (tx2 + dx, ty2 + dy))
            arr_r.set_visible(True)
            ax_rot.legend([il_r, tl_r, arr_r],
                          ["Initial Angle", "Target Angle", r"Initial $\omega$"],
                          loc='upper right', fontsize=16, framealpha=0.75)
        else:
            arr_r.set_visible(False)
            ax_rot.legend([il_r, tl_r], ["Initial Angle", "Target Angle"],
                          loc='upper right', fontsize=16, framealpha=0.75)

        # Translation arrows
        leg = ax_trans.get_legend()
        if leg is not None: leg.remove()
        if show_arrow:
            tip  = _get_rotated_points([(0, 50 - local_com_y)], (0, 0), t_val)[0]
            tx2, ty2 = tip[0] + cx, tip[1] + cy
            r = np.sqrt(tx2 ** 2 + ty2 ** 2) or 1
            dx = -np.sign(omega_0) * (-ty2 / r) * arrow_len_trans
            dy = -np.sign(omega_0) * (tx2  / r) * arrow_len_trans
            arr_t.set_positions((tx2, ty2), (tx2 + dx, ty2 + dy))
            arr_t.set_visible(True)
            ax_trans.legend([il_t, tl_t, arr_t],
                            ["Initial Angle", "Target Angle", r"Initial $\omega$"],
                            loc='upper right', fontsize=16, framealpha=0.75)
        else:
            arr_t.set_visible(False)
            if result.times[i] >= result.final_time and speed > 1e-6:
                gc   = _rotate_point(0, -local_com_y, 0, 0, t_val)
                bx2  = gc[0] + cx;  by2 = gc[1] + cy
                vel_arr_t.set_positions(
                    (bx2, by2),
                    (bx2 + result.vxs[i] / speed * arrow_len_trans,
                     by2 + result.vys[i] / speed * arrow_len_trans))
                vel_arr_t.set_visible(True)
                ax_trans.legend([il_t, tl_t, vel_arr_t],
                                ["Initial Angle", "Target Angle", "Velocity"],
                                loc='upper right', fontsize=16, framealpha=0.75)
            else:
                vel_arr_t.set_visible(False)
                ax_trans.legend([il_t, tl_t], ["Initial Angle", "Target Angle"],
                                loc='upper right', fontsize=16, framealpha=0.75)

        return (bp_r, np_r, ep_r, fp_r, bp_t, np_t, ep_t, fp_t,
                arr_r, arr_t, vel_arr_t,
                tx_angle, tx_omega, tx_gimbal, tx_thrust,
                tx_fuel, tx_time, tx_x, tx_y, tx_speed)

    frames = _anim_frames(result, config)
    ani    = FuncAnimation(fig, _frame, frames=frames, interval=25, blit=True)
    print("Saving dual-view animation...")
    ani.save(output_path, writer=FFMpegWriter(fps=config.fps, bitrate=2800))
    print(f"Done. Saved to {output_path}")
    plt.close()


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def animate_attitude(result, theta_0, theta_target, omega_0,
                     dry_mass, fuel_mass, config,
                     mode="rotation", output_path=None):
    """
    Render and save the attitude animation.

    Parameters
    ----------
    mode : str  - "rotation" | "translation" | "dual"
    output_path : str or None  - auto-named if None
    """
    if output_path is None:
        output_path = f"results/attitude_{mode}.mp4"

    if mode == "dual":
        _animate_dual(result, theta_0, theta_target, omega_0,
                      dry_mass, fuel_mass, config, output_path)
    elif mode == "translation":
        _animate_translation(result, theta_0, theta_target, omega_0,
                             dry_mass, fuel_mass, config, output_path)
    else:
        _animate_rotation(result, theta_0, theta_target, omega_0,
                          dry_mass, fuel_mass, config, output_path)
