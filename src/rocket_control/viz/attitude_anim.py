"""Attitude animations. Drawing uses degrees; the plant stays in radians."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rocket_control.attitude.config import AttitudeSettings
from rocket_control.attitude.simulate import AttitudeResult
from rocket_control.core.vehicle import VehicleSpec

from .geometry import AttitudeVisual, attitude_visual, rotate_point, rotate_points


@dataclass
class _DegView:
    times: np.ndarray
    angles: np.ndarray
    unwrapped_angles: np.ndarray
    omegas: np.ndarray
    gimbals: np.ndarray
    masses: np.ndarray
    thrusts: np.ndarray
    torques: np.ndarray
    coms: np.ndarray
    x_coms: np.ndarray
    y_coms: np.ndarray
    vxs: np.ndarray
    vys: np.ndarray
    final_time: float


def _view(result: AttitudeResult) -> _DegView:
    return _DegView(
        times=result.times,
        angles=np.rad2deg(result.theta),
        unwrapped_angles=np.rad2deg(result.theta_unwrapped),
        omegas=np.rad2deg(result.omega),
        gimbals=np.rad2deg(result.gimbal),
        masses=result.mass,
        thrusts=result.thrust,
        torques=result.torque,
        coms=result.com,
        x_coms=result.x,
        y_coms=result.y,
        vxs=result.vx,
        vys=result.vy,
        final_time=result.final_time,
    )


def _sky(ax, extent):
    from matplotlib.colors import LinearSegmentedColormap
    import numpy as np

    cmap = LinearSegmentedColormap.from_list("sky", ["#102C57", "#000000"])
    ax.imshow(np.linspace(0, 1, 100).reshape(-1, 1), extent=extent, origin="lower", cmap=cmap, aspect="auto")


def _patches(ax):
    from matplotlib.patches import Polygon

    body = Polygon([(0, 0)], fc="white", ec="black", zorder=11)
    nose = Polygon([(0, 0)], fc="white", ec="black", zorder=12)
    engine = Polygon([(0, 0)], fc="#D0D0D0", ec="black", zorder=10)
    flame = Polygon([(0, 0)], fc="orange", ec="red", lw=1.5, zorder=9)
    for p in (body, nose, engine, flame):
        ax.add_patch(p)
    return body, nose, engine, flame


def _frames(result: _DegView, settings: AttitudeSettings):
    return [0] * settings.hold_frames + list(range(0, len(result.times), settings.step_size))


def animate_attitude(
    result: AttitudeResult,
    theta0: float,
    theta_target: float,
    omega0: float,
    vehicle: VehicleSpec,
    settings: AttitudeSettings,
    *,
    mode: str = "rotation",
    output_path: str | Path | None = None,
    show: bool = False,
) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation
    from matplotlib.patches import FancyArrowPatch

    view = _view(result)
    geom = attitude_visual(vehicle)
    theta0_d, target_d, omega0_d = np.rad2deg(theta0), np.rad2deg(theta_target), np.rad2deg(omega0)
    if output_path is None:
        output_path = Path("results") / f"attitude_{mode}.mp4"
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if mode == "dual":
        fig = _animate_dual(view, theta0_d, target_d, omega0_d, vehicle, settings, geom)
        bitrate = 2800
        interval = 25
    elif mode == "translation":
        fig = _animate_translation(view, theta0_d, target_d, omega0_d, vehicle, settings, geom)
        bitrate, interval = 2500, 20
    else:
        fig = _animate_rotation(view, theta0_d, target_d, omega0_d, vehicle, settings, geom)
        bitrate, interval = 2500, 20

    ani = FuncAnimation(fig, fig._rocket_frame, frames=_frames(view, settings), interval=interval, blit=True)
    ani.save(output_path, writer=FFMpegWriter(fps=settings.fps, bitrate=bitrate))
    if show:
        plt.show()
    else:
        plt.close(fig)


def _telemetry_panel(ax_info):
    ax_info.set_facecolor("white")
    ax_info.axis("off")
    lb, fs = -0.2, 15
    keys = (
        ("angle", 0.8), ("omega", 0.7), ("gimbal", 0.6), ("thrust", 0.5),
        ("fuel", 0.4), ("burned", 0.3), ("torque", 0.2), ("time", 0.1),
    )
    texts = {
        k: ax_info.text(lb, y, "", color="black", fontsize=fs, family="monospace", fontweight="bold")
        for k, y in keys
    }
    ax_info.text(0.5, 0.9, "LIVE TELEMETRY", color="black", fontsize=fs + 4, fontweight="bold", ha="center")
    return texts


def _fill_telemetry(texts, view: _DegView, i: int, dry_mass: float, fuel_mass: float):
    lw, vw = len("FUEL BURNED:") + 1, 11
    t_val = view.angles[i]
    texts["angle"].set_text(f"{'BODY TILT:':>{lw}}{t_val:>{vw}.2f} [°]")
    texts["omega"].set_text(f"{'ANG VEL:':>{lw}}{view.omegas[i]:>{vw}.2f} [°/s]")
    texts["gimbal"].set_text(f"{'GIMBAL:':>{lw}}{view.gimbals[i]:>{vw}.2f} [°]")
    texts["thrust"].set_text(f"{'THRUST:':>{lw}}{view.thrusts[i]:>{vw}.0f} [N]")
    texts["fuel"].set_text(f"{'FUEL LEFT:':>{lw}}{view.masses[i] - dry_mass:>{vw}.2f} [kg]")
    texts["burned"].set_text(f"{'FUEL USED:':>{lw}}{fuel_mass - (view.masses[i] - dry_mass):>{vw}.2f} [kg]")
    texts["torque"].set_text(f"{'TORQUE:':>{lw}}{view.torques[i]:>{vw}.0f} [N*m]")
    texts["time"].set_text(f"{'TIME:':>{lw}}{min(view.times[i], view.final_time):>{vw}.2f} [s]")


def _animate_rotation(view, theta0_d, target_d, omega0_d, vehicle, settings, geom: AttitudeVisual):
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch

    fig_size = geom.rocket_height + 40
    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(12, 8), gridspec_kw={"width_ratios": [2, 1]})
    ax.set_xlim(-fig_size, fig_size)
    ax.set_ylim(-fig_size, fig_size)
    ax.set_aspect("equal")
    ax.axis("off")
    _sky(ax, [-fig_size, fig_size, -fig_size, fig_size])
    body_p, nose_p, engine_p, flame_p = _patches(ax)
    rad1, rad2 = np.radians(theta0_d), np.radians(target_d)
    init_line, = ax.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)], color="red", ls="--", alpha=0.6, zorder=5)
    tgt_line, = ax.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)], color="green", ls="--", alpha=0.7, zorder=5)
    arrow_p = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20, color="yellow", visible=False)
    ax.add_patch(arrow_p)
    ax.text(0.5, 1.02, f"Initial Angular Velocity: {omega0_d:.2f} deg/s", family="monospace", fontweight="bold", ha="center", va="bottom", fontsize=12, transform=ax.transAxes)
    texts = _telemetry_panel(ax_info)
    dt = settings.dt
    dry, fuel = vehicle.m_dry, vehicle.m_fuel

    def _frame(i):
        t_val, g_val, thr_val = view.angles[i], view.gimbals[i], view.thrusts[i]
        eng_rot = rotate_points(geom.engine_raw, geom.engine_attach, g_val)
        flame_rot = rotate_points(geom.flame_raw, geom.engine_attach, g_val)
        body_p.set_xy(rotate_points(geom.body_raw, (0, 0), t_val))
        nose_p.set_xy(rotate_points(geom.nose_raw, (0, 0), t_val))
        engine_p.set_xy(rotate_points(eng_rot, (0, 0), t_val))
        flame_p.set_xy(rotate_points(flame_rot, (0, 0), t_val))
        flame_p.set_visible(thr_val > 0 and i != 0)
        _fill_telemetry(texts, view, i, dry, fuel)
        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
        show_arrow = view.times[i] < dt and abs(omega0_d) > 1e-6
        if show_arrow:
            tip = rotate_points([(0, 50)], (0, 0), t_val)[0]
            tx, ty = tip
            r = np.hypot(tx, ty) or 1
            dx = -np.sign(omega0_d) * (-ty / r) * settings.arrow_length_rot
            dy = -np.sign(omega0_d) * (tx / r) * settings.arrow_length_rot
            arrow_p.set_positions((tx, ty), (tx + dx, ty + dy))
            arrow_p.set_visible(True)
            ax.legend([init_line, tgt_line, arrow_p], ["Initial Angle", "Target Angle", r"Initial $\omega$"], loc="upper right", fontsize=15, framealpha=0.7)
        else:
            arrow_p.set_visible(False)
            ax.legend([init_line, tgt_line], ["Initial Angle", "Target Angle"], loc="upper right", fontsize=15, framealpha=0.7)
        return (body_p, nose_p, engine_p, flame_p, *texts.values(), arrow_p)

    fig._rocket_frame = _frame
    return fig


def _animate_translation(view, theta0_d, target_d, omega0_d, vehicle, settings, geom: AttitudeVisual):
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch

    max_disp = float(max(np.max(np.abs(view.x_coms)), np.max(np.abs(view.y_coms))))
    fig_size = max(settings.trans_scale_factor * float(max_disp), geom.rocket_height + settings.trans_min_margin)
    arrow_len = settings.arrow_length_trans_fraction * fig_size
    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(12, 8), gridspec_kw={"width_ratios": [2, 1]})
    ax.set_xlim(-fig_size, fig_size)
    ax.set_ylim(-fig_size + 25, fig_size)
    ax.set_aspect("equal")
    ax.axis("off")
    _sky(ax, [-fig_size, fig_size, -fig_size, fig_size])
    body_p, nose_p, engine_p, flame_p = _patches(ax)
    rad1, rad2 = np.radians(theta0_d), np.radians(target_d)
    init_line, = ax.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)], color="red", ls="--", alpha=0.6, zorder=5)
    tgt_line, = ax.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)], color="green", ls="--", alpha=0.7, zorder=5)
    arrow_p = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20, color="yellow", visible=False, zorder=20)
    vel_arrow_p = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20, color="cyan", visible=False, zorder=20)
    ax.add_patch(arrow_p)
    ax.add_patch(vel_arrow_p)
    texts = _telemetry_panel(ax_info)
    pv1 = ax.text(0.05, 0.06, "", transform=ax.transAxes, color="white", fontsize=13, family="monospace", fontweight="bold", va="bottom")
    pv2 = ax.text(0.05, 0.02, "", transform=ax.transAxes, color="white", fontsize=13, family="monospace", fontweight="bold", va="bottom")
    dt = settings.dt
    dry, fuel = vehicle.m_dry, vehicle.m_fuel
    eng_rel = [(ex - geom.engine_attach[0], ey - geom.engine_attach[1]) for ex, ey in geom.engine_raw]
    flame_rel = [(fx - geom.engine_attach[0], fy - geom.engine_attach[1]) for fx, fy in geom.flame_raw]

    def _frame(i):
        t_val, g_val, thr_val = view.unwrapped_angles[i], view.gimbals[i], view.thrusts[i]
        cx, cy = view.x_coms[i], view.y_coms[i]
        local_com_y = view.coms[i] - geom.dry_cm_from_nozzle
        body_local = [(bx, by - local_com_y) for bx, by in geom.body_raw]
        nose_local = [(nx, ny - local_com_y) for nx, ny in geom.nose_raw]
        ea = (geom.engine_attach[0], geom.engine_attach[1] - local_com_y)
        eng_g = rotate_points(eng_rel, (0, 0), g_val)
        flame_g = rotate_points(flame_rel, (0, 0), g_val)
        eng_local = [(ex + ea[0], ey + ea[1]) for ex, ey in eng_g]
        flame_local = [(fx + ea[0], fy + ea[1]) for fx, fy in flame_g]
        body_p.set_xy([(px + cx, py + cy) for px, py in rotate_points(body_local, (0, 0), t_val)])
        nose_p.set_xy([(px + cx, py + cy) for px, py in rotate_points(nose_local, (0, 0), t_val)])
        engine_p.set_xy([(px + cx, py + cy) for px, py in rotate_points(eng_local, (0, 0), t_val)])
        flame_p.set_xy([(px + cx, py + cy) for px, py in rotate_points(flame_local, (0, 0), t_val)])
        flame_p.set_visible(thr_val > 0 and i != 0)
        _fill_telemetry(texts, view, i, dry, fuel)
        pv1.set_text(f"   X: {cx:10.2f} m      Y: {cy:10.2f} m")
        pv2.set_text(f"  Vx: {view.vxs[i]:10.2f} m/s   Vy: {view.vys[i]:10.2f} m/s")
        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
        show_arrow = view.times[i] < dt and abs(omega0_d) > 1e-6
        speed = np.hypot(view.vxs[i], view.vys[i])
        if show_arrow:
            tip = rotate_points([(0, 50 - local_com_y)], (0, 0), t_val)[0]
            tx2, ty2 = tip[0] + cx, tip[1] + cy
            r = np.hypot(tx2, ty2) or 1
            dx = -np.sign(omega0_d) * (-ty2 / r) * arrow_len
            dy = -np.sign(omega0_d) * (tx2 / r) * arrow_len
            arrow_p.set_positions((tx2, ty2), (tx2 + dx, ty2 + dy))
            arrow_p.set_visible(True)
            vel_arrow_p.set_visible(False)
            ax.legend([init_line, tgt_line, arrow_p], ["Initial Angle", "Target Angle", r"Initial $\omega$"], loc="upper right", fontsize=15, framealpha=0.7)
        else:
            arrow_p.set_visible(False)
            if view.times[i] >= view.final_time and speed > 1e-6:
                gc = rotate_point(0, -local_com_y, 0, 0, t_val)
                bx2, by2 = gc[0] + cx, gc[1] + cy
                vel_arrow_p.set_positions((bx2, by2), (bx2 + view.vxs[i] / speed * arrow_len, by2 + view.vys[i] / speed * arrow_len))
                vel_arrow_p.set_visible(True)
                ax.legend([init_line, tgt_line, vel_arrow_p], ["Initial Angle", "Target Angle", "Velocity"], loc="upper right", fontsize=15, framealpha=0.7)
            else:
                vel_arrow_p.set_visible(False)
                ax.legend([init_line, tgt_line], ["Initial Angle", "Target Angle"], loc="upper right", fontsize=15, framealpha=0.7)
        return (body_p, nose_p, engine_p, flame_p, *texts.values(), arrow_p, vel_arrow_p, pv1, pv2)

    fig._rocket_frame = _frame
    return fig


def _animate_dual(view, theta0_d, target_d, omega0_d, vehicle, settings, geom: AttitudeVisual):
    """Side-by-side rotation and translation; telemetry along the bottom."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch

    max_disp = float(max(np.max(np.abs(view.x_coms)), np.max(np.abs(view.y_coms))))
    fig_size_rot = geom.rocket_height + 40
    fig_size_trans = max(settings.trans_scale_factor * float(max_disp), geom.rocket_height + settings.trans_min_margin)
    fig = plt.figure(figsize=(19.5, 13.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.28], width_ratios=[1, 1], wspace=0.08, hspace=0.15)
    ax_rot = fig.add_subplot(gs[0, 0])
    ax_trans = fig.add_subplot(gs[0, 1])
    ax_tele = fig.add_subplot(gs[1, :])
    ax_rot.set_xlim(-fig_size_rot, fig_size_rot)
    ax_rot.set_ylim(-fig_size_rot, fig_size_rot)
    ax_rot.set_aspect("equal")
    ax_rot.axis("off")
    ax_rot.set_title("Rotation-Only View", fontsize=20, pad=10)
    _sky(ax_rot, [-fig_size_rot, fig_size_rot, -fig_size_rot, fig_size_rot])
    bp_r, np_r, ep_r, fp_r = _patches(ax_rot)
    ax_trans.set_xlim(-fig_size_trans, fig_size_trans)
    ax_trans.set_ylim(-fig_size_trans + 25, fig_size_trans)
    ax_trans.set_aspect("equal")
    ax_trans.axis("off")
    ax_trans.set_title("Full View (Rotation + Translation)", fontsize=20, pad=10)
    _sky(ax_trans, [-fig_size_trans, fig_size_trans, -fig_size_trans, fig_size_trans])
    bp_t, np_t, ep_t, fp_t = _patches(ax_trans)
    rad1, rad2 = np.radians(theta0_d), np.radians(target_d)
    il_r, = ax_rot.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)], color="red", ls="--", alpha=0.6, zorder=5)
    tl_r, = ax_rot.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)], color="green", ls="--", alpha=0.7, zorder=5)
    il_t, = ax_trans.plot([0, 80 * np.sin(rad1)], [0, 80 * np.cos(rad1)], color="red", ls="--", alpha=0.6, zorder=5)
    tl_t, = ax_trans.plot([0, 80 * np.sin(rad2)], [0, 80 * np.cos(rad2)], color="green", ls="--", alpha=0.7, zorder=5)
    arr_r = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20, color="yellow", visible=False)
    arr_t = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20, color="yellow", visible=False, zorder=20)
    vel_arr_t = FancyArrowPatch((0, 0), (0, 0), mutation_scale=20, color="cyan", visible=False, zorder=20)
    ax_rot.add_patch(arr_r)
    ax_trans.add_patch(arr_t)
    ax_trans.add_patch(vel_arr_t)
    ax_tele.set_facecolor("white")
    ax_tele.axis("off")
    fs, vw = 20, 11
    ax_tele.text(0.5, 0.95, "TELEMETRY", color="black", fontsize=24, fontweight="bold", ha="center")
    tx = {
        "angle": ax_tele.text(0.19, 0.75, "", fontsize=fs, family="monospace", fontweight="bold"),
        "omega": ax_tele.text(0.19, 0.50, "", fontsize=fs, family="monospace", fontweight="bold"),
        "gimbal": ax_tele.text(0.19, 0.25, "", fontsize=fs, family="monospace", fontweight="bold"),
        "thrust": ax_tele.text(0.54, 0.75, "", fontsize=fs, family="monospace", fontweight="bold"),
        "fuel": ax_tele.text(0.54, 0.50, "", fontsize=fs, family="monospace", fontweight="bold"),
        "time": ax_tele.text(0.54, 0.25, "", fontsize=fs, family="monospace", fontweight="bold"),
        "x": ax_tele.text(0.82, 0.75, "", fontsize=fs, family="monospace", fontweight="bold"),
        "y": ax_tele.text(0.82, 0.50, "", fontsize=fs, family="monospace", fontweight="bold"),
        "speed": ax_tele.text(0.82, 0.25, "", fontsize=fs, family="monospace", fontweight="bold"),
    }
    ax_tele.text(0.04, 0.75, "BODY TILT:", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.04, 0.50, "ANG VEL: ", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.04, 0.25, "GIMBAL:  ", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.39, 0.75, "THRUST:   ", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.39, 0.50, "FUEL LEFT:", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.39, 0.25, "TIME:     ", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.72, 0.75, "X:   ", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.72, 0.50, "Y:   ", fontsize=fs, family="monospace", fontweight="bold")
    ax_tele.text(0.72, 0.25, "VEL:", fontsize=fs, family="monospace", fontweight="bold")
    dt = settings.dt
    dry = vehicle.m_dry
    eng_rel = [(ex - geom.engine_attach[0], ey - geom.engine_attach[1]) for ex, ey in geom.engine_raw]
    flame_rel = [(fx - geom.engine_attach[0], fy - geom.engine_attach[1]) for fx, fy in geom.flame_raw]
    arrow_len_rot = settings.arrow_length_rot
    arrow_len_trans = settings.arrow_length_trans_fraction * fig_size_trans

    def _frame(i):
        t_wrapped, t_val = view.angles[i], view.unwrapped_angles[i]
        g_val, thr_val = view.gimbals[i], view.thrusts[i]
        cx, cy = view.x_coms[i], view.y_coms[i]
        local_com_y = view.coms[i] - geom.dry_cm_from_nozzle
        er = rotate_points(geom.engine_raw, geom.engine_attach, g_val)
        fr = rotate_points(geom.flame_raw, geom.engine_attach, g_val)
        bp_r.set_xy(rotate_points(geom.body_raw, (0, 0), t_wrapped))
        np_r.set_xy(rotate_points(geom.nose_raw, (0, 0), t_wrapped))
        ep_r.set_xy(rotate_points(er, (0, 0), t_wrapped))
        fp_r.set_xy(rotate_points(fr, (0, 0), t_wrapped))
        fp_r.set_visible(thr_val > 0 and i != 0)
        body_local = [(bx, by - local_com_y) for bx, by in geom.body_raw]
        nose_local = [(nx, ny - local_com_y) for nx, ny in geom.nose_raw]
        ea = (geom.engine_attach[0], geom.engine_attach[1] - local_com_y)
        eg = rotate_points(eng_rel, (0, 0), g_val)
        fg = rotate_points(flame_rel, (0, 0), g_val)
        el = [(ex + ea[0], ey + ea[1]) for ex, ey in eg]
        fl = [(fx + ea[0], fy + ea[1]) for fx, fy in fg]
        bp_t.set_xy([(px + cx, py + cy) for px, py in rotate_points(body_local, (0, 0), t_val)])
        np_t.set_xy([(px + cx, py + cy) for px, py in rotate_points(nose_local, (0, 0), t_val)])
        ep_t.set_xy([(px + cx, py + cy) for px, py in rotate_points(el, (0, 0), t_val)])
        fp_t.set_xy([(px + cx, py + cy) for px, py in rotate_points(fl, (0, 0), t_val)])
        fp_t.set_visible(thr_val > 0 and i != 0)
        speed = np.hypot(view.vxs[i], view.vys[i])
        tx["angle"].set_text(f"{t_wrapped:>{vw}.2f} °")
        tx["omega"].set_text(f"{view.omegas[i]:>{vw}.2f} °/s")
        tx["gimbal"].set_text(f"{g_val:>{vw}.2f} °")
        tx["thrust"].set_text(f"{thr_val:>{vw}.0f} N")
        tx["fuel"].set_text(f"{view.masses[i] - dry:>{vw}.2f} kg")
        tx["time"].set_text(f"{min(view.times[i], view.final_time):>{vw}.2f} s")
        tx["x"].set_text(f"{cx:>{vw}.2f} m")
        tx["y"].set_text(f"{cy:>{vw}.2f} m")
        tx["speed"].set_text(f"{speed:>{vw}.2f} m/s")
        return (bp_r, np_r, ep_r, fp_r, bp_t, np_t, ep_t, fp_t, arr_r, arr_t, vel_arr_t, *tx.values())

    fig._rocket_frame = _frame
    return fig
