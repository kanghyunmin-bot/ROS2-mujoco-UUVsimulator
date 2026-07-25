#!/usr/bin/env python3
"""Compare real localization/path and simulated odometry for bag seconds 28..100."""

from __future__ import annotations

import csv
import json
import math
import os
import sqlite3
import struct
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State


ROOT = Path(os.environ.get("UUV_REPLAY_ANALYSIS_DIR", Path(__file__).resolve().parent)).resolve()
WORKSPACE = ROOT.parents[1]
REAL_BAG = WORKSPACE / "localization.db3"
SIM_DB = ROOT / "sim_result" / "sim_result_0.db3"
SOURCE_START_S = 28.0
SOURCE_END_S = 100.0

SIM_COLOR = "#2563EB"
REAL_COLOR = "#D97706"
INK = "#202124"
MUTED = "#5F6673"
GRID = "#D9DEE7"


def yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def load_real_path() -> list[tuple[float, float, float, float, float]]:
    con = sqlite3.connect(REAL_BAG)
    bag_start_ns = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    topic_id = con.execute("SELECT id FROM topics WHERE name='/localization/path'").fetchone()[0]
    start_ns = bag_start_ns + round(SOURCE_START_S * 1.0e9)
    end_ns = bag_start_ns + round(SOURCE_END_S * 1.0e9)
    records = []
    query = """
        SELECT timestamp, substr(data, -56, 56)
        FROM messages
        WHERE topic_id=? AND timestamp>=? AND timestamp<=?
        ORDER BY timestamp
    """
    for timestamp_ns, pose_blob in con.execute(query, (topic_id, start_ns, end_ns)):
        x, y, z, qx, qy, qz, qw = struct.unpack("<7d", pose_blob)
        records.append(((timestamp_ns - start_ns) * 1.0e-9, x, y, z, yaw(qx, qy, qz, qw)))
    con.close()
    return records


def load_sim_window():
    con = sqlite3.connect(SIM_DB)
    topics = {name: topic_id for topic_id, name in con.execute("SELECT id,name FROM topics")}
    rc_start_ns, rc_end_ns, rc_count = con.execute(
        "SELECT MIN(timestamp),MAX(timestamp),COUNT(*) FROM messages WHERE topic_id=?",
        (topics["/mavros/rc/override"],),
    ).fetchone()
    odom = []
    query = """
        SELECT timestamp,data FROM messages
        WHERE topic_id=? AND timestamp>=? AND timestamp<=?
        ORDER BY timestamp
    """
    first_header_ns = None
    for timestamp_ns, data in con.execute(query, (topics["/odometry/filtered"], rc_start_ns, rc_end_ns)):
        msg = deserialize_message(bytes(data), Odometry)
        header_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if first_header_ns is None:
            first_header_ns = header_ns
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        odom.append(((header_ns - first_header_ns) * 1.0e-9, p.x, p.y, p.z, yaw(q.x, q.y, q.z, q.w)))
    states = []
    for _, data in con.execute(query, (topics["/mavros/state"], rc_start_ns, rc_end_ns)):
        states.append(deserialize_message(bytes(data), State))
    con.close()
    return (
        odom,
        states,
        rc_count,
        (rc_end_ns - rc_start_ns) * 1.0e-9,
        odom[-1][0] if odom else 0.0,
    )


def normalize_start_pose(rows):
    x0, y0, z0, yaw0 = rows[0][1:5]
    c = math.cos(yaw0)
    s = math.sin(yaw0)
    output = []
    for t_s, x, y, z, heading in rows:
        dx, dy = x - x0, y - y0
        # Rotate world displacement into each run's initial body-heading frame.
        body_x = c * dx + s * dy
        body_y = -s * dx + c * dy
        output.append((t_s, body_x, body_y, z - z0, heading))
    headings = np.unwrap([row[4] for row in output])
    return [(row[0], row[1], row[2], row[3], headings[i] - headings[0]) for i, row in enumerate(output)]


def path_length_xy(rows) -> float:
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:]))


def export_csv(name: str, rows) -> None:
    with (ROOT / name).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "forward_m", "lateral_m", "relative_z_m", "relative_yaw_rad"])
        writer.writerows(rows)


def calculate_summary(real, sim, states, rc_count, rc_wall_span_s, sim_header_span_s):
    common_end = min(real[-1][0], sim[-1][0])
    sim_times = np.array([row[0] for row in sim])
    mask = sim_times <= common_end
    eval_t = sim_times[mask]
    real_t = np.array([row[0] for row in real])
    real_interp = np.column_stack([
        np.interp(eval_t, real_t, [row[col] for row in real]) for col in range(1, 5)
    ])
    sim_eval = np.array([[row[col] for col in range(1, 5)] for row, keep in zip(sim, mask) if keep])
    xy_error = np.linalg.norm(sim_eval[:, :2] - real_interp[:, :2], axis=1)
    z_error = sim_eval[:, 2] - real_interp[:, 2]
    yaw_error = np.arctan2(
        np.sin(sim_eval[:, 3] - real_interp[:, 3]),
        np.cos(sim_eval[:, 3] - real_interp[:, 3]),
    )
    summary = {
        "source_window_s": [SOURCE_START_S, SOURCE_END_S],
        "real_source": "localization.db3:/localization/path latest pose per message",
        "sim_source": "sim_result:/odometry/filtered during RC playback",
        "alignment": "initial xyz translation and initial yaw rotation only; no scale or fitted registration",
        "rc_override_messages": rc_count,
        "rc_wall_arrival_span_s": rc_wall_span_s,
        "sim_odom_header_span_s": sim_header_span_s,
        "real_samples": len(real),
        "sim_samples": len(sim),
        "sim_all_states_armed": all(state.armed for state in states),
        "sim_state_modes": sorted({state.mode for state in states}),
        "real_end_forward_lateral_z_m": list(real[-1][1:4]),
        "sim_end_forward_lateral_z_m": list(sim[-1][1:4]),
        "real_xy_path_length_m": path_length_xy(real),
        "sim_xy_path_length_m": path_length_xy(sim),
        "xy_rmse_m": float(np.sqrt(np.mean(xy_error ** 2))),
        "xy_final_error_m": float(xy_error[-1]),
        "z_rmse_m": float(np.sqrt(np.mean(z_error ** 2))),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
        "real_final_relative_yaw_deg": float(np.degrees(real[-1][4])),
        "sim_final_relative_yaw_deg": float(np.degrees(sim[-1][4])),
    }
    (ROOT / "comparison_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def style_axis(ax, title, ylabel=None):
    ax.set_title(title, loc="left", color=INK, fontweight="semibold")
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.75)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)


def plot(real, sim, rc_count):
    fig = plt.figure(figsize=(15, 13), facecolor="white")
    grid = fig.add_gridspec(3, 2, hspace=0.38, wspace=0.22)
    ax_xy = fig.add_subplot(grid[0, 0])
    ax_error = fig.add_subplot(grid[0, 1])
    ax_x = fig.add_subplot(grid[1, 0])
    ax_y = fig.add_subplot(grid[1, 1])
    ax_z = fig.add_subplot(grid[2, 0])
    ax_yaw = fig.add_subplot(grid[2, 1])

    def draw(ax, x_index, y_index, xlabel, ylabel):
        ax.plot([row[x_index] for row in sim], [row[y_index] for row in sim], color=SIM_COLOR, linewidth=2.0, label="Sim ALT_HOLD")
        ax.plot([row[x_index] for row in real], [row[y_index] for row in real], color=REAL_COLOR, linewidth=1.8, linestyle="--", label="Real localization/path")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)

    draw(ax_xy, 1, 2, "forward from start [m]", "lateral from start [m]")
    ax_xy.scatter(0, 0, color=INK, s=52, label="Common start", zorder=4)
    ax_xy.scatter(sim[-1][1], sim[-1][2], color=SIM_COLOR, s=58, edgecolor="white", zorder=4)
    ax_xy.scatter(real[-1][1], real[-1][2], color=REAL_COLOR, marker="s", s=58, edgecolor="white", zorder=4)
    style_axis(ax_xy, "XY trajectory — initial position and heading aligned")
    ax_xy.axis("equal")
    ax_xy.legend(loc="upper left", frameon=False, ncol=3)

    sim_t = np.array([row[0] for row in sim])
    real_t = np.array([row[0] for row in real])
    mask = sim_t <= real_t[-1]
    eval_t = sim_t[mask]
    real_x = np.interp(eval_t, real_t, [row[1] for row in real])
    real_y = np.interp(eval_t, real_t, [row[2] for row in real])
    sim_x = np.array([row[1] for row in sim])[mask]
    sim_y = np.array([row[2] for row in sim])[mask]
    separation = np.hypot(sim_x - real_x, sim_y - real_y)
    ax_error.plot(eval_t, separation, color=SIM_COLOR, linewidth=2.0)
    ax_error.fill_between(eval_t, 0.0, separation, color=SIM_COLOR, alpha=0.10)
    ax_error.set_xlabel("elapsed replay time [s]")
    ax_error.set_ylabel("XY separation [m]")
    style_axis(ax_error, "Planar separation after initial-pose alignment")

    for ax, index, title, unit in [
        (ax_x, 1, "Forward displacement", "forward [m]"),
        (ax_y, 2, "Lateral displacement", "lateral [m]"),
        (ax_z, 3, "Relative vertical position", "z - z0 [m]"),
    ]:
        draw(ax, 0, index, "elapsed replay time [s]", unit)
        style_axis(ax, title)

    ax_yaw.plot([row[0] for row in sim], np.degrees([row[4] for row in sim]), color=SIM_COLOR, linewidth=2.0, label="Sim ALT_HOLD")
    ax_yaw.plot([row[0] for row in real], np.degrees([row[4] for row in real]), color=REAL_COLOR, linewidth=1.8, linestyle="--", label="Real localization/path")
    ax_yaw.set_xlabel("elapsed replay time [s]")
    ax_yaw.set_ylabel("yaw - yaw0 [deg]")
    style_axis(ax_yaw, "Relative yaw")

    fig.suptitle("Real vs Sim path — bag seconds 28 to 100", x=0.065, y=0.975, ha="left", color=INK, fontsize=18, fontweight="bold")
    fig.text(
        0.065,
        0.942,
        f"Sim: armed ALT_HOLD, {rc_count:,} RC override messages on simulation clock"
        "  |  Real: /localization/path current pose",
        ha="left",
        color=MUTED,
        fontsize=10.5,
    )
    fig.text(0.065, 0.025, "Alignment removes only the initial position and heading. No trajectory fitting or scale correction is applied.", ha="left", color=MUTED, fontsize=9.5)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.91, bottom=0.075)
    fig.savefig(ROOT / "real_vs_sim_path_28_100.png", dpi=170, facecolor="white")


def main():
    real_raw = load_real_path()
    sim_raw, states, rc_count, rc_wall_span_s, sim_header_span_s = load_sim_window()
    real = normalize_start_pose(real_raw)
    sim = normalize_start_pose(sim_raw)
    export_csv("real_path_28_100_aligned.csv", real)
    export_csv("sim_path_28_100_aligned.csv", sim)
    summary = calculate_summary(
        real, sim, states, rc_count, rc_wall_span_s, sim_header_span_s
    )
    plot(real, sim, rc_count)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
