#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter


def plot_sphere(ax, center, radius, color, alpha=0.12, wire_alpha=0.25):
    u = np.linspace(0.0, 2.0 * np.pi, 40)
    v = np.linspace(0.0, np.pi, 24)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)
    ax.plot_wireframe(
        x,
        y,
        z,
        color=color,
        alpha=wire_alpha,
        linewidth=0.35,
        rstride=3,
        cstride=3,
    )


def set_axes_equal(ax, points):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    radius = np.max(maxs - mins) * 0.55
    radius = max(radius, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def apply_axes_limits(ax, points, xlim=None, ylim=None, zlim=None):
    set_axes_equal(ax, points)
    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)
    if zlim is not None:
        ax.set_zlim(*zlim)
    ax.set_box_aspect((1.0, 1.0, 1.0))


def set_scatter_point(artist, point):
    artist._offsets3d = ([point[0]], [point[1]], [point[2]])


def hide_scatter_point(artist):
    artist._offsets3d = ([np.nan], [np.nan], [np.nan])


def frame_indices(length, stride):
    stride = max(1, stride)
    ids = list(range(0, length, stride))
    if not ids or ids[-1] != length - 1:
        ids.append(length - 1)
    return ids


def build_series(entity_id, group):
    group = group.sort_values("step")
    stalled = (
        group["stalled"].to_numpy(dtype=int)
        if "stalled" in group.columns
        else np.zeros(len(group), dtype=int)
    )
    return {
        "id": entity_id,
        "pos": group[["x", "y", "z"]].to_numpy(dtype=float),
        "vel": group[["vx", "vy", "vz"]].to_numpy(dtype=float),
        "goal": group[["goal_x", "goal_y", "goal_z"]].iloc[0].to_numpy(dtype=float),
        "radius": float(group["radius"].iloc[0]),
        "step": group["step"].to_numpy(dtype=int),
        "t": group["t"].to_numpy(dtype=float),
        "reached": group["reached"].to_numpy(dtype=int),
        "stalled": stalled,
        "min_clearance": group["min_clearance"].to_numpy(dtype=float),
    }


def min_clearance_series(agents, timeline_len):
    if not agents:
        return np.full(timeline_len, np.nan, dtype=float)
    result = np.full(timeline_len, np.nan, dtype=float)
    for idx in range(timeline_len):
        vals = [
            agent["min_clearance"][idx]
            for agent in agents
            if idx < len(agent["min_clearance"]) and np.isfinite(agent["min_clearance"][idx])
        ]
        if vals:
            result[idx] = float(np.min(vals))
    return result


def build_bounds(statics, agents, dynamics):
    pts = []
    for obs in statics:
        center = obs["center"]
        radius = obs["radius"]
        pts.append(center.reshape(1, 3))
        pts.append(center.reshape(1, 3) + np.array([[radius, radius, radius]]))
        pts.append(center.reshape(1, 3) - np.array([[radius, radius, radius]]))
    for series in agents + dynamics:
        pts.append(series["pos"])
        pts.append(series["goal"].reshape(1, 3))
    if not pts:
        pts = [np.zeros((1, 3), dtype=float)]
    return np.vstack(pts)


def load_trace_multi(df):
    required = {
        "step",
        "t",
        "kind",
        "id",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "radius",
        "goal_x",
        "goal_y",
        "goal_z",
        "min_clearance",
        "reached",
    }
    missing = required.difference(df.columns)
    if missing:
        raise ValueError(f"Trace CSV is missing columns: {sorted(missing)}")

    statics = []
    static_df = df[df["kind"] == "static"]
    for entity_id, group in static_df.groupby("id", sort=False):
        row = group.iloc[0]
        statics.append(
            {
                "id": entity_id,
                "center": row[["x", "y", "z"]].to_numpy(dtype=float),
                "radius": float(row["radius"]),
            }
        )

    agents = []
    agent_df = df[df["kind"] == "agent"]
    for entity_id, group in agent_df.groupby("id", sort=False):
        agents.append(build_series(entity_id, group))

    dynamics = []
    dynamic_df = df[df["kind"] == "dynamic"]
    for entity_id, group in dynamic_df.groupby("id", sort=False):
        dynamics.append(build_series(entity_id, group))

    if agents:
        steps = agents[0]["step"]
        times = agents[0]["t"]
    elif dynamics:
        steps = dynamics[0]["step"]
        times = dynamics[0]["t"]
    else:
        steps = static_df["step"].drop_duplicates().to_numpy(dtype=int)
        times = static_df["t"].drop_duplicates().to_numpy(dtype=float)

    return {
        "mode": "multi",
        "statics": statics,
        "agents": agents,
        "dynamics": dynamics,
        "steps": steps,
        "times": times,
        "d_min": min_clearance_series(agents, len(times)),
        "bounds": build_bounds(statics, agents, dynamics),
    }

def load_trace(trace_path):
    df = pd.read_csv(trace_path)
    if df.empty:
        raise ValueError("Trace CSV is empty.")
    return load_trace_multi(df)


def color_list(name, count):
    cmap = plt.get_cmap(name)
    if count <= 1:
        return [cmap(0.5)]
    return [cmap(i / (count - 1)) for i in range(count)]


def make_axes(data, title, xlim=None, ylim=None, zlim=None):
    fig = plt.figure(figsize=(10.5, 8.0))
    ax = fig.add_subplot(111, projection="3d")
    apply_axes_limits(ax, data["bounds"], xlim=xlim, ylim=ylim, zlim=zlim)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.view_init(elev=24, azim=43)
    ax.grid(True, alpha=0.25)
    ax.set_title(title)

    agent_colors = color_list("tab10", max(1, len(data["agents"])))
    dynamic_colors = color_list("Set1", max(1, len(data["dynamics"])))

    for idx, obs in enumerate(data["statics"]):
        plot_sphere(
            ax,
            obs["center"],
            obs["radius"],
            color="#ff7f0e",
            alpha=0.14,
            wire_alpha=0.28,
        )
        ax.scatter(
            *obs["center"],
            color="#ff7f0e",
            s=68,
            marker="s",
            label="Static obstacle" if idx == 0 else None,
        )

    for idx, agent in enumerate(data["agents"]):
        color = agent_colors[idx]
        ax.scatter(
            *agent["goal"],
            color=color,
            s=84,
            marker="*",
            label="Agent goal" if idx == 0 else None,
        )
        ax.scatter(
            *agent["pos"][0],
            color=color,
            s=48,
            marker="o",
            alpha=0.85,
            label="Agent start" if idx == 0 else None,
        )

    for idx, obs in enumerate(data["dynamics"]):
        color = dynamic_colors[idx]
        ax.scatter(
            *obs["pos"][0],
            color=color,
            s=44,
            marker="D",
            alpha=0.75,
            label="Dynamic start" if idx == 0 else None,
        )

    return fig, ax, agent_colors, dynamic_colors


def min_clearance_marker(data):
    best = None
    for agent in data["agents"]:
        clearance = agent["min_clearance"]
        finite_ids = np.where(np.isfinite(clearance))[0]
        if finite_ids.size == 0:
            continue
        local_idx = finite_ids[np.argmin(clearance[finite_ids])]
        value = float(clearance[local_idx])
        if best is None or value < best["value"]:
            best = {"value": value, "point": agent["pos"][local_idx]}
    return best


def first_stall_marker(agent):
    stalled_ids = np.where(agent["stalled"] != 0)[0]
    if stalled_ids.size == 0:
        return None
    return int(stalled_ids[0])


def save_static_plot(data, out_path, dpi, title, xlim=None, ylim=None, zlim=None):
    fig, ax, agent_colors, dynamic_colors = make_axes(
        data, title, xlim=xlim, ylim=ylim, zlim=zlim
    )

    stalled_label_used = False
    for idx, agent in enumerate(data["agents"]):
        color = agent_colors[idx]
        ax.plot(
            agent["pos"][:, 0],
            agent["pos"][:, 1],
            agent["pos"][:, 2],
            color=color,
            lw=2.2,
            label=f"Agent {agent['id']}",
        )
        if agent["stalled"][-1]:
            ax.scatter(
                *agent["pos"][-1],
                color="#d62728",
                s=110,
                marker="X",
                edgecolors="white",
                linewidths=0.8,
                label="Stalled agent" if not stalled_label_used else None,
            )
            stalled_label_used = True
        else:
            ax.scatter(*agent["pos"][-1], color=color, s=72, marker="^")

        stall_idx = first_stall_marker(agent)
        if stall_idx is not None:
            ax.scatter(
                *agent["pos"][stall_idx],
                color="#d62728",
                s=92,
                marker="X",
                edgecolors="white",
                linewidths=0.8,
                label="Stall point" if stalled_label_used else None,
            )
            stalled_label_used = True

    for idx, obs in enumerate(data["dynamics"]):
        color = dynamic_colors[idx]
        ax.plot(
            obs["pos"][:, 0],
            obs["pos"][:, 1],
            obs["pos"][:, 2],
            color=color,
            lw=1.8,
            linestyle="--",
            label=f"Dynamic {obs['id']}",
        )
        ax.scatter(*obs["pos"][-1], color=color, s=50, marker="o", alpha=0.85)

    marker = min_clearance_marker(data)
    if marker is not None:
        ax.scatter(
            *marker["point"],
            color="#9467bd",
            s=78,
            marker="x",
            label=f"Min clearance = {marker['value']:.3f}",
        )

    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    print(f"Saved 3D visualization to {out_path}")


def save_gif_animation(data, out_path, dpi, title, fps, stride, tail, xlim=None, ylim=None, zlim=None):
    fig, ax, agent_colors, dynamic_colors = make_axes(
        data, title, xlim=xlim, ylim=ylim, zlim=zlim
    )

    for idx, agent in enumerate(data["agents"]):
        color = agent_colors[idx]
        ax.plot(
            agent["pos"][:, 0],
            agent["pos"][:, 1],
            agent["pos"][:, 2],
            color=color,
            lw=1.0,
            alpha=0.18,
            linestyle=":",
        )

    for idx, obs in enumerate(data["dynamics"]):
        color = dynamic_colors[idx]
        ax.plot(
            obs["pos"][:, 0],
            obs["pos"][:, 1],
            obs["pos"][:, 2],
            color=color,
            lw=1.0,
            alpha=0.18,
            linestyle=":",
        )

    agent_now = []
    for idx, agent in enumerate(data["agents"]):
        color = agent_colors[idx]
        now = ax.scatter([], [], [], color=color, s=66, marker="o")
        agent_now.append(now)

    stalled_now = []
    has_stalled_agents = any(np.any(agent["stalled"]) for agent in data["agents"])
    if has_stalled_agents:
        for idx, agent in enumerate(data["agents"]):
            now = ax.scatter(
                [],
                [],
                [],
                color="#d62728",
                s=104,
                marker="X",
                edgecolors="white",
                linewidths=0.8,
                label="Stalled agent" if idx == 0 else None,
            )
            stalled_now.append(now)

    dynamic_now = []
    for idx, obs in enumerate(data["dynamics"]):
        color = dynamic_colors[idx]
        now = ax.scatter([], [], [], color=color, s=58, marker="D", label=f"Dynamic {obs['id']}")
        dynamic_now.append(now)

    time_text = ax.text2D(0.03, 0.94, "", transform=ax.transAxes, fontsize=10)
    dist_text = ax.text2D(0.03, 0.88, "", transform=ax.transAxes, fontsize=10)
    status_text = ax.text2D(0.03, 0.82, "", transform=ax.transAxes, fontsize=10)
    ax.legend(loc="upper left", fontsize=9)

    frame_ids = frame_indices(len(data["times"]), stride)
    trail_len = len(data["times"]) if tail <= 0 else tail

    def update(frame_id):
        idx = frame_ids[frame_id]
        start = max(0, idx - trail_len + 1)

        artists = []
        stalled_agents = 0
        reached_agents = 0
        for idx_agent, (series, now) in enumerate(zip(data["agents"], agent_now)):
            set_scatter_point(now, series["pos"][idx])
            artists.append(now)
            if series["reached"][idx]:
                reached_agents += 1
            if series["stalled"][idx]:
                stalled_agents += 1
                if stalled_now:
                    set_scatter_point(stalled_now[idx_agent], series["pos"][idx])
                    artists.append(stalled_now[idx_agent])
            elif stalled_now:
                hide_scatter_point(stalled_now[idx_agent])
                artists.append(stalled_now[idx_agent])

        for series, now in zip(data["dynamics"], dynamic_now):
            set_scatter_point(now, series["pos"][idx])
            artists.append(now)

        time_text.set_text(
            f"t = {data['times'][idx]:.1f}s    step = {int(data['steps'][idx])}"
        )
        clearance = data["d_min"][idx]
        if np.isfinite(clearance):
            dist_text.set_text(f"min clearance = {clearance:.3f} m")
        else:
            dist_text.set_text("min clearance = n/a")
        status_text.set_text(
            f"reached = {reached_agents}/{len(data['agents'])}    stalled = {stalled_agents}"
        )
        artists.extend([time_text, dist_text, status_text])
        return artists

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_ids),
        interval=1000 / max(1, fps),
        blit=False,
    )
    fig.tight_layout()
    anim.save(out_path, writer=PillowWriter(fps=max(1, fps)), dpi=dpi)
    print(f"Saved 3D animation to {out_path}")


def main():
    def axis_limit(value, axis_name):
        if value is None:
            return None
        lo, hi = value
        if lo >= hi:
            raise ValueError(f"{axis_name} expects MIN < MAX, got {lo} >= {hi}")
        return (lo, hi)

    parser = argparse.ArgumentParser(
        description="Visualize the 3D obstacle-avoidance simulation trace."
    )
    parser.add_argument("--csv", default="sim_trace.csv", help="Input trace CSV")
    parser.add_argument("--out", default="sim_3d.png", help="Output figure or GIF path")
    parser.add_argument("--dpi", type=int, default=200, help="Output DPI")
    parser.add_argument("--title", default="", help="Optional custom title")
    parser.add_argument("--fps", type=int, default=15, help="Animation FPS for GIF output")
    parser.add_argument(
        "--stride",
        type=int,
        default=2,
        help="Frame stride when exporting GIF to reduce size",
    )
    parser.add_argument(
        "--tail",
        type=int,
        default=80,
        help="Number of historical frames to keep in the animated trail; 0 keeps all",
    )
    parser.add_argument(
        "--xlim",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        help="Optional fixed X-axis range in meters",
    )
    parser.add_argument(
        "--ylim",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        help="Optional fixed Y-axis range in meters",
    )
    parser.add_argument(
        "--zlim",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        help="Optional fixed Z-axis range in meters",
    )
    args = parser.parse_args()

    trace_path = Path(args.csv)
    if not trace_path.exists():
        raise FileNotFoundError(f"Trace file not found: {trace_path}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    data = load_trace(trace_path)
    title = args.title.strip() or "3D VO Avoidance Visualization"
    xlim = axis_limit(args.xlim, "xlim")
    ylim = axis_limit(args.ylim, "ylim")
    zlim = axis_limit(args.zlim, "zlim")

    suffix = out_path.suffix.lower()
    if suffix == ".gif":
        save_gif_animation(
            data,
            out_path,
            dpi=args.dpi,
            title=title,
            fps=args.fps,
            stride=args.stride,
            tail=args.tail,
            xlim=xlim,
            ylim=ylim,
            zlim=zlim,
        )
    else:
        save_static_plot(
            data,
            out_path,
            dpi=args.dpi,
            title=title,
            xlim=xlim,
            ylim=ylim,
            zlim=zlim,
        )


if __name__ == "__main__":
    main()
