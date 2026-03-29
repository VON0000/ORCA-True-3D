#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


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


def main():
    parser = argparse.ArgumentParser(
        description="Visualize the 3D obstacle-avoidance simulation trace."
    )
    parser.add_argument("--csv", default="sim_trace.csv", help="Input trace CSV")
    parser.add_argument("--out", default="sim_3d.png", help="Output figure path")
    parser.add_argument("--dpi", type=int, default=200, help="Figure DPI")
    parser.add_argument("--title", default="", help="Optional custom title")
    args = parser.parse_args()

    trace_path = Path(args.csv)
    if not trace_path.exists():
        raise FileNotFoundError(f"Trace file not found: {trace_path}")

    df = pd.read_csv(trace_path)
    if df.empty:
        raise ValueError("Trace CSV is empty.")

    rpz = float(df["rpz"].iloc[0])
    rpf = float(df["rpf"].iloc[0])
    target = np.array(
        [df["target_x"].iloc[0], df["target_y"].iloc[0], df["target_z"].iloc[0]],
        dtype=float,
    )
    static_center = np.array(
        [df["stat_x"].iloc[0], df["stat_y"].iloc[0], df["stat_z"].iloc[0]],
        dtype=float,
    )

    uav = df[["uav_x", "uav_y", "uav_z"]].to_numpy(dtype=float)
    dyn = df[["dyn_x", "dyn_y", "dyn_z"]].to_numpy(dtype=float)
    dmin_idx = int(df["d_min"].idxmin())
    dmin_val = float(df["d_min"].iloc[dmin_idx])

    fig = plt.figure(figsize=(10.5, 8.0))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(uav[:, 0], uav[:, 1], uav[:, 2], color="#1f77b4", lw=2.2, label="UAV")
    ax.plot(
        dyn[:, 0],
        dyn[:, 1],
        dyn[:, 2],
        color="#d62728",
        lw=1.8,
        linestyle="--",
        label="Dynamic obstacle",
    )

    ax.scatter(*uav[0], color="#1f77b4", s=56, marker="o", label="Start")
    ax.scatter(*uav[-1], color="#2ca02c", s=70, marker="^", label="End")
    ax.scatter(*target, color="#2ca02c", s=80, marker="*", label="Target")
    ax.scatter(*static_center, color="#ff7f0e", s=70, marker="s", label="Static obstacle")

    plot_sphere(ax, static_center, rpz, color="#ff7f0e", alpha=0.14, wire_alpha=0.28)
    plot_sphere(ax, static_center, rpf, color="#bcbd22", alpha=0.06, wire_alpha=0.14)

    key_ids = [0, len(dyn) // 2, len(dyn) - 1]
    for i, idx in enumerate(key_ids):
        c = dyn[idx]
        alpha = 0.08 if i < 2 else 0.11
        plot_sphere(ax, c, rpz, color="#d62728", alpha=alpha, wire_alpha=0.15)

    closest_pt = uav[dmin_idx]
    ax.scatter(
        closest_pt[0],
        closest_pt[1],
        closest_pt[2],
        color="#9467bd",
        s=70,
        marker="x",
        label=f"Min distance = {dmin_val:.3f}",
    )

    bounds_pts = np.vstack(
        [
            uav,
            dyn,
            target.reshape(1, 3),
            static_center.reshape(1, 3),
            static_center.reshape(1, 3) + np.array([[rpf, rpf, rpf]]),
            static_center.reshape(1, 3) - np.array([[rpf, rpf, rpf]]),
        ]
    )
    set_axes_equal(ax, bounds_pts)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.view_init(elev=24, azim=43)
    ax.grid(True, alpha=0.25)
    title = args.title.strip() or "3D VO + Potential-Field Avoidance Visualization"
    ax.set_title(title)
    ax.legend(loc="best", fontsize=9)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=args.dpi)
    print(f"Saved 3D visualization to {out_path}")


if __name__ == "__main__":
    main()
