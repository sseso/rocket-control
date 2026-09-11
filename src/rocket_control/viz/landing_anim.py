"""Landing animation."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d

from rocket_control.core.mass_properties import com_and_inertia_array
from rocket_control.landing.scenarios import LandingProblem
from rocket_control.landing.types import Trajectory

from .geometry import LandingVizSettings, VisualGeometry, rotate_points


def animate_landing(
    traj: Trajectory,
    problem: LandingProblem,
    output_path: str | Path,
    *,
    show: bool = False,
    visual: VisualGeometry | None = None,
    settings: LandingVizSettings | None = None,
) -> None:
    """Angles are converted to degrees only for drawing."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle

    visual = visual or VisualGeometry()
    cfg = settings or LandingVizSettings()
    vehicle = problem.vehicle
    states, controls, tf = traj.states, traj.controls, traj.tf
    sim_times = traj.times
    fps = cfg.fps
    num_hold_start = int(cfg.hold_time_start * fps)
    num_hold_end = int(cfg.hold_time_end * fps)
    num_anim_frames = int(tf * fps) + 1
    anim_times_raw = np.linspace(0, tf, num_anim_frames)
    dt_anim = anim_times_raw[1] - anim_times_raw[0] if num_anim_frames > 1 else 0.0

    def _interp(arr):
        return interp1d(sim_times, arr)(anim_times_raw)

    x_coms = _interp(states[:, 0])
    y_coms = _interp(states[:, 1])
    vxs = _interp(states[:, 2])
    vys = _interp(states[:, 3])
    angles = np.rad2deg(_interp(states[:, 4]))
    omegas = np.rad2deg(_interp(states[:, 5]))
    masses = _interp(states[:, 6])
    thrusts = _interp(controls[:, 0])
    gimbals = np.rad2deg(_interp(controls[:, 1]))
    thrusts[-1] = 0
    gimbals[-1] = 0
    coms, _ = com_and_inertia_array(masses, vehicle)

    def _prepend(arr, val=None):
        v = arr[0] if val is None else val
        return np.concatenate((np.full(num_hold_start, v), arr))

    def _append(arr, val=None):
        v = arr[-1] if val is None else val
        return np.append(arr, np.full(num_hold_end, v))

    x_coms = _append(_prepend(x_coms))
    y_coms = _append(_prepend(y_coms))
    vxs = _append(_prepend(vxs), val=0.0)
    vys = _append(_prepend(vys), val=0.0)
    angles = _append(_prepend(angles))
    omegas = _append(_prepend(omegas), val=0.0)
    masses = _append(_prepend(masses))
    thrusts = _append(_prepend(thrusts), val=0.0)
    gimbals = _append(_prepend(gimbals))
    coms = _append(_prepend(coms))
    t_start = np.linspace(-cfg.hold_time_start, -dt_anim, num_hold_start) if num_hold_start else np.array([])
    t_end = np.linspace(tf + dt_anim, tf + cfg.hold_time_end, num_hold_end) if num_hold_end else np.array([])
    anim_times = np.concatenate((t_start, anim_times_raw, t_end))
    n_frames = len(anim_times)

    margin = vehicle.rocket_height * cfg.visual_margin_factor
    x_min, x_max = float(min(x_coms) - margin), float(max(x_coms) + margin)
    y_min, y_max = -vehicle.rocket_height, float(max(y_coms) + margin)
    x_span, y_span = x_max - x_min, y_max - y_min
    if x_span < y_span:
        extra = (y_span - x_span) / 2
        x_min -= extra
        x_max += extra
    elif y_span < x_span:
        extra = (x_span - y_span) / 2
        y_min -= extra
        y_max += extra

    fig, (ax, ax_info) = plt.subplots(1, 2, figsize=(12, 8), gridspec_kw={"width_ratios": [2, 1]})
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.axis("off")
    cmap = LinearSegmentedColormap.from_list("sky", ["#102C57", "#000000"])
    ax.imshow(
        np.linspace(0, 1, 100).reshape(-1, 1),
        extent=[x_min, x_max, y_min, y_max],
        origin="lower",
        cmap=cmap,
        aspect="auto",
    )
    ax.axhspan(y_min, 0, color="#888888", zorder=1)
    ax.axhline(0, color="#B1B1B1", linewidth=2, zorder=2)
    ix_line, = ax.plot([x_coms[0], x_coms[0]], [0, y_max], color="red", ls="--", lw=2, zorder=1.5, alpha=0.4)
    tx_line, = ax.plot([0, 0], [0, y_max], color="green", ls="--", lw=2, zorder=1.5, alpha=0.4)
    ax.add_patch(Rectangle((-20, -5), 40, 5, fc="gray", ec="black", zorder=3))

    body_patch = Polygon([[0, 0]], fc="white", ec="black", zorder=11)
    nose_patch = Polygon([[0, 0]], fc="white", ec="black", zorder=12)
    engine_patch = Polygon([[0, 0]], fc="#D0D0D0", ec="black", zorder=10)
    flame_patch = Polygon([[0, 0]], fc="orange", ec="red", lw=1.5, zorder=9, visible=False)
    for p in (body_patch, nose_patch, engine_patch, flame_patch):
        ax.add_patch(p)

    arrow_length = abs(x_max - x_min) * cfg.visual_arrow_length_factor
    vel_arrow = FancyArrowPatch(
        (0, 0), (0, 0),
        arrowstyle="simple, head_width=9, head_length=9, tail_width=4",
        mutation_scale=1.5, facecolor="cyan", edgecolor="black",
        linewidth=1.5, zorder=20, visible=False,
    )
    ax.add_patch(vel_arrow)
    pv1 = ax.text(0.12, 0.06, "", transform=ax.transAxes, color="white", fontsize=13, family="monospace", fontweight="bold", va="bottom")
    pv2 = ax.text(0.12, 0.02, "", transform=ax.transAxes, color="white", fontsize=13, family="monospace", fontweight="bold", va="bottom")

    ax_info.set_facecolor("white")
    ax_info.axis("off")
    lb, fs = -0.2, 15
    texts = {
        name: ax_info.text(lb, y, "", color="black", fontsize=fs, family="monospace", fontweight="bold")
        for name, y in (
            ("alt", 0.8), ("down", 0.7), ("th", 0.6), ("om", 0.5),
            ("gim", 0.4), ("thr", 0.3), ("fuel", 0.2), ("time", 0.1),
        )
    }
    ax_info.text(0.5, 0.9, "TELEMETRY", color="black", fontsize=fs + 4, fontweight="bold", ha="center")

    body_raw = visual.body_raw
    nose_raw = visual.nose_raw
    engine_attach = visual.engine_attach
    engine_rel = [(ex - engine_attach[0], ey - engine_attach[1]) for ex, ey in visual.engine_raw]
    flame_rel = [(fx - engine_attach[0], fy - engine_attach[1]) for fx, fy in visual.flame_raw]
    lw, vw = cfg.label_width, cfg.value_width
    base_fl = cfg.base_flame_length

    def _animate(i):
        t_val, g_val, thr_val = angles[i], gimbals[i], thrusts[i]
        cx, cy, com = x_coms[i], y_coms[i], coms[i]
        body_local = [[px, py - com] for px, py in body_raw]
        nose_local = [[px, py - com] for px, py in nose_raw]
        eng_attach_local = (0.0, engine_attach[1] - com)
        body_rot = rotate_points(body_local, (0, 0), t_val)
        nose_rot = rotate_points(nose_local, (0, 0), t_val)
        eng_gimb = rotate_points(engine_rel, (0, 0), g_val)
        eng_local = [(ex + eng_attach_local[0], ey + eng_attach_local[1]) for ex, ey in eng_gimb]
        engine_rot = rotate_points(eng_local, (0, 0), t_val)
        throttle = thr_val / vehicle.T_max if vehicle.T_max > 0 else 0.0
        fl = base_fl * (throttle ** cfg.flame_throttle_gamma)
        flame_scaled = [(fx, fy * (fl / base_fl)) for fx, fy in flame_rel]
        flame_gimb = rotate_points(flame_scaled, (0, 0), g_val)
        flame_local = [(fx + eng_attach_local[0], fy + eng_attach_local[1]) for fx, fy in flame_gimb]
        flame_rot = rotate_points(flame_local, (0, 0), t_val)
        body_patch.set_xy([(px + cx, py + cy) for px, py in body_rot])
        nose_patch.set_xy([(px + cx, py + cy) for px, py in nose_rot])
        engine_patch.set_xy([(px + cx, py + cy) for px, py in engine_rot])
        flame_patch.set_xy([(px + cx, py + cy) for px, py in flame_rot])
        flame_patch.set_visible(thr_val > 0)
        texts["alt"].set_text(f"{'ALTITUDE:':>{lw}}{y_coms[i] - coms[i]:>{vw}.2f} m")
        texts["down"].set_text(f"{'DOWNRANGE:':>{lw}}{x_coms[i]:>{vw}.2f} m")
        texts["th"].set_text(f"{'THETA:':>{lw}}{t_val:>{vw}.2f} °")
        texts["om"].set_text(f"{'OMEGA:':>{lw}}{omegas[i]:>{vw}.2f} °/s")
        texts["gim"].set_text(f"{'GIMBAL:':>{lw}}{g_val:>{vw}.2f} °")
        texts["thr"].set_text(f"{'THRUST:':>{lw}}{thr_val:>{vw}.0f} N")
        texts["fuel"].set_text(f"{'FUEL:':>{lw}}{masses[i] - vehicle.m_dry:>{vw}.2f} kg")
        texts["time"].set_text(f"{'TIME:':>{lw}}{max(0, min(anim_times[i], tf)):>{vw}.2f} s")
        pv1.set_text(f" X: {x_coms[i]:10.2f} m   Alt: {y_coms[i] - coms[i]:10.2f} m")
        pv2.set_text(f"Vx: {vxs[i]:10.2f} m/s  Vy: {vys[i]:10.2f} m/s")
        show_arrow = anim_times[i] < 0
        speed = np.hypot(vxs[i], vys[i])
        if show_arrow and speed > 1e-3:
            dx, dy = (vxs[i] / speed) * arrow_length, (vys[i] / speed) * arrow_length
            vel_arrow.set_positions((cx, cy), (cx + dx, cy + dy))
            vel_arrow.set_visible(True)
        else:
            vel_arrow.set_visible(False)
        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
        if show_arrow and speed > 1e-3:
            ax.legend([ix_line, tx_line, vel_arrow], ["Initial x", "Target x", "Initial v"], loc="upper right", fontsize=14, framealpha=0.7)
        else:
            ax.legend([ix_line, tx_line], ["Initial x", "Target x"], loc="upper right", fontsize=14, framealpha=0.7)
        return (body_patch, nose_patch, engine_patch, flame_patch, *texts.values(), pv1, pv2, vel_arrow)

    ani = FuncAnimation(fig, _animate, frames=range(n_frames), interval=1000 // fps, blit=True)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ani.save(output_path, writer=FFMpegWriter(fps=fps, bitrate=2500))

    plt.close(fig)
