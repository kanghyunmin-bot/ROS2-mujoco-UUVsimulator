#!/usr/bin/env python3
"""Export and plot the ALT_HOLD RC-override replay result."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path(__file__).resolve().parent
BAG = ROOT / "sim_result"


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_bag():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(BAG), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}
    rows = {topic: [] for topic in topic_types}
    first_timestamp = None
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if first_timestamp is None:
            first_timestamp = timestamp
        msg = deserialize_message(raw, message_types[topic])
        rows[topic].append(((timestamp - first_timestamp) * 1.0e-9, msg))
    return rows


def export_odom(rows):
    path = ROOT / "simulated_odom.csv"
    output = []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps", "yaw_rad"])
        for t_s, msg in rows["/odometry/filtered"]:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            row = (t_s, p.x, p.y, p.z, v.x, v.y, v.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            writer.writerow(row)
            output.append(row)
    return output


def export_rc(rows):
    path = ROOT / "rc_override.csv"
    output = []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", *[f"ch{i}" for i in range(1, 19)]])
        for t_s, msg in rows["/mavros/rc/override"]:
            channels = list(msg.channels)
            channels += [0] * (18 - len(channels))
            row = (t_s, *channels[:18])
            writer.writerow(row)
            output.append(row)
    return output


def export_ground_truth(rows):
    output = []
    for t_s, msg in rows["/mujoco/ground_truth/pose"]:
        p = msg.pose.position
        output.append((t_s, p.x, p.y, p.z))
    return output


def summarize(odom, rc, rows):
    start = odom[0]
    end = odom[-1]
    xs = [row[1] for row in odom]
    ys = [row[2] for row in odom]
    zs = [row[3] for row in odom]
    speeds = [math.sqrt(row[4] ** 2 + row[5] ** 2 + row[6] ** 2) for row in odom]
    path_length = sum(
        math.dist((a[1], a[2], a[3]), (b[1], b[2], b[3])) for a, b in zip(odom, odom[1:])
    )
    states = rows["/mavros/state"]
    summary = {
        "duration_s": end[0] - start[0],
        "odom_samples": len(odom),
        "rc_override_samples": len(rc),
        "start_xyz_m": [start[1], start[2], start[3]],
        "end_xyz_m": [end[1], end[2], end[3]],
        "net_displacement_xyz_m": [end[i] - start[i] for i in range(1, 4)],
        "net_displacement_3d_m": math.dist(start[1:4], end[1:4]),
        "path_length_3d_m": path_length,
        "x_range_m": [min(xs), max(xs)],
        "y_range_m": [min(ys), max(ys)],
        "z_range_m": [min(zs), max(zs)],
        "max_speed_mps": max(speeds),
        "mean_speed_mps": sum(speeds) / len(speeds),
        "all_state_samples_armed": all(msg.armed for _, msg in states),
        "state_modes": sorted({msg.mode for _, msg in states}),
    }
    (ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def plot(odom, rc, ground_truth):
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)

    ax = axes[0, 0]
    ax.plot([row[1] for row in ground_truth], [row[2] for row in ground_truth], color="0.75", label="MuJoCo ground truth")
    ax.plot([row[1] for row in odom], [row[2] for row in odom], color="tab:blue", linewidth=1.5, label="/odometry/filtered")
    ax.scatter(odom[0][1], odom[0][2], color="green", s=55, label="start", zorder=4)
    ax.scatter(odom[-1][1], odom[-1][2], color="red", s=55, label="end", zorder=4)
    ax.set(title="XY trajectory", xlabel="x [m]", ylabel="y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[0, 1]
    t = [row[0] for row in odom]
    ax.plot(t, [row[1] - odom[0][1] for row in odom], label="x - x0")
    ax.plot(t, [row[2] - odom[0][2] for row in odom], label="y - y0")
    ax.plot(t, [row[3] - odom[0][3] for row in odom], label="z - z0")
    ax.set(title="Relative odometry position", xlabel="record time [s]", ylabel="relative position [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1, 0]
    ax.plot(t, [row[4] for row in odom], label="vx")
    ax.plot(t, [row[5] for row in odom], label="vy")
    ax.plot(t, [row[6] for row in odom], label="vz")
    ax.set(title="Odometry linear velocity", xlabel="record time [s]", ylabel="velocity [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1, 1]
    rc_t = [row[0] for row in rc]
    labels = {3: "heave ch3", 4: "yaw ch4", 5: "forward ch5", 6: "lateral ch6"}
    for channel, label in labels.items():
        # MAVROS uses 0 for release and UINT16_MAX for no-change. Plot only
        # actual PWM overrides so those sentinels do not obscure the command.
        pwm = [float("nan") if row[channel] in (0, 65535) else row[channel] for row in rc]
        ax.plot(rc_t, pwm, linewidth=0.9, label=label)
    ax.axhline(1500, color="0.4", linewidth=0.8, linestyle="--", label="neutral")
    ax.set(title="Replayed active RC override", xlabel="record time [s]", ylabel="PWM [us]", ylim=(1150, 1850))
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2, fontsize=8)

    fig.suptitle("UUV MuJoCo closed-loop replay — armed ALT_HOLD", fontsize=15)
    fig.savefig(ROOT / "althold_rc_replay_odom.png", dpi=160)


def main():
    rows = read_bag()
    odom = export_odom(rows)
    rc = export_rc(rows)
    ground_truth = export_ground_truth(rows)
    summary = summarize(odom, rc, rows)
    plot(odom, rc, ground_truth)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
