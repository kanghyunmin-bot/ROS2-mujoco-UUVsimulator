#!/usr/bin/env python3
"""Compare the recorded localization Path with the simulated odometry path."""

from __future__ import annotations

import csv
import json
import math
import sqlite3
import struct
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
REAL_BAG = ROOT.parents[1] / "localization.db3"
SIM_ODOM_CSV = ROOT / "simulated_odom.csv"

SIM_COLOR = "#2563EB"
REAL_COLOR = "#D97706"
INK = "#202124"
GRID = "#D9DEE7"


def load_real_path() -> list[tuple[float, float, float, float]]:
    """Read the newest pose from each rolling nav_msgs/Path message.

    Pose is the final fixed-size 56-byte element in each serialized message, so
    SQLite can return only its x/y/z fields instead of materializing every
    5,000-pose rolling history.
    """
    connection = sqlite3.connect(REAL_BAG)
    topic_id = connection.execute(
        "SELECT id FROM topics WHERE name = '/localization/path'"
    ).fetchone()[0]
    bag_start_ns = connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    rows = []
    query = """
        SELECT timestamp, substr(data, -56, 24)
        FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
    """
    for timestamp_ns, xyz_blob in connection.execute(query, (topic_id,)):
        x, y, z = struct.unpack("<ddd", xyz_blob)
        rows.append(((timestamp_ns - bag_start_ns) * 1.0e-9, x, y, z))
    connection.close()
    return rows


def load_sim_path() -> list[tuple[float, float, float, float]]:
    rows = []
    with SIM_ODOM_CSV.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((float(row["t_s"]), float(row["x_m"]), float(row["y_m"]), float(row["z_m"])))
    return rows


def path_length(rows: list[tuple[float, float, float, float]]) -> float:
    return sum(math.dist(a[1:4], b[1:4]) for a, b in zip(rows, rows[1:]))


def relative_xy(rows: list[tuple[float, float, float, float]]) -> tuple[list[float], list[float]]:
    x0, y0 = rows[0][1], rows[0][2]
    return [row[1] - x0 for row in rows], [row[2] - y0 for row in rows]


def export_real_path(rows: list[tuple[float, float, float, float]]) -> None:
    with (ROOT / "real_localization_path.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["bag_time_s", "x_m", "y_m", "z_m"])
        writer.writerows(rows)


def make_summary(real, sim) -> dict:
    real_delta = [real[-1][i] - real[0][i] for i in range(1, 4)]
    sim_delta = [sim[-1][i] - sim[0][i] for i in range(1, 4)]
    summary = {
        "real_source": "localization.db3:/localization/path latest pose per message",
        "real_frame": "odom",
        "sim_source": "sim_result:/odometry/filtered",
        "sim_frame": "map",
        "alignment": "translation only: subtract each trajectory's first x/y",
        "real_samples": len(real),
        "sim_samples": len(sim),
        "real_start_xyz_m": list(real[0][1:4]),
        "sim_start_xyz_m": list(sim[0][1:4]),
        "real_end_xyz_m": list(real[-1][1:4]),
        "sim_end_xyz_m": list(sim[-1][1:4]),
        "real_net_delta_xyz_m": real_delta,
        "sim_net_delta_xyz_m": sim_delta,
        "real_path_length_3d_m": path_length(real),
        "sim_path_length_3d_m": path_length(sim),
        "real_xy_ranges_m": {
            "x": [min(row[1] for row in real), max(row[1] for row in real)],
            "y": [min(row[2] for row in real), max(row[2] for row in real)],
        },
        "sim_xy_ranges_m": {
            "x": [min(row[1] for row in sim), max(row[1] for row in sim)],
            "y": [min(row[2] for row in sim), max(row[2] for row in sim)],
        },
    }
    (ROOT / "real_sim_path_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def style_axis(ax, title: str) -> None:
    ax.set_title(title, loc="left", color=INK, fontsize=13, fontweight="semibold")
    ax.set_xlabel("x [m]", color=INK)
    ax.set_ylabel("y [m]", color=INK)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.75)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)
    ax.axis("equal")


def plot(real, sim) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(15, 7.8), facecolor="white")

    ax = axes[0]
    ax.plot([row[1] for row in sim], [row[2] for row in sim], color=SIM_COLOR, linewidth=2.0, label="Sim odometry (map)")
    ax.plot([row[1] for row in real], [row[2] for row in real], color=REAL_COLOR, linewidth=1.8, linestyle="--", label="Real localization/path (odom)")
    ax.scatter(sim[0][1], sim[0][2], color=SIM_COLOR, marker="o", s=65, edgecolor="white", linewidth=1.0, zorder=4)
    ax.scatter(real[0][1], real[0][2], color=REAL_COLOR, marker="s", s=60, edgecolor="white", linewidth=1.0, zorder=4)
    style_axis(ax, "Recorded coordinates (no alignment)")
    ax.legend(loc="upper left", frameon=False)

    ax = axes[1]
    real_x, real_y = relative_xy(real)
    sim_x, sim_y = relative_xy(sim)
    ax.plot(sim_x, sim_y, color=SIM_COLOR, linewidth=2.0, label="Sim odometry")
    ax.plot(real_x, real_y, color=REAL_COLOR, linewidth=1.8, linestyle="--", label="Real localization/path")
    ax.scatter(0, 0, color=INK, marker="o", s=62, edgecolor="white", linewidth=1.0, label="Common start", zorder=4)
    ax.scatter(sim_x[-1], sim_y[-1], color=SIM_COLOR, marker="o", s=65, edgecolor="white", linewidth=1.0, zorder=4)
    ax.scatter(real_x[-1], real_y[-1], color=REAL_COLOR, marker="s", s=60, edgecolor="white", linewidth=1.0, zorder=4)
    style_axis(ax, "Start-aligned trajectories (translation only)")
    ax.legend(loc="upper left", frameon=False)

    fig.suptitle("Real localization path vs simulated odometry", x=0.06, y=0.97, ha="left", color=INK, fontsize=18, fontweight="bold")
    fig.text(
        0.06,
        0.925,
        "Real: localization.db3 /localization/path (5,642 current poses)  |  Sim: armed ALT_HOLD replay /odometry/filtered (4,630 samples)",
        ha="left",
        color="#5F6673",
        fontsize=10.5,
    )
    fig.text(
        0.06,
        0.025,
        "Note: real is in 'odom' and sim is in 'map'. Right panel removes initial translation only; no rotation, scaling, or fitted registration is applied.",
        ha="left",
        color="#5F6673",
        fontsize=9.5,
    )
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.11, wspace=0.22)
    fig.savefig(ROOT / "real_vs_sim_localization_path.png", dpi=170, facecolor="white")


def main() -> None:
    real = load_real_path()
    sim = load_sim_path()
    export_real_path(real)
    summary = make_summary(real, sim)
    plot(real, sim)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
