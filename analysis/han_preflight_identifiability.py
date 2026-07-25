#!/usr/bin/env python3
"""Audit whether a real ROS bag can support HAN-first plant identification.

This deliberately refuses to fit hydrodynamic coefficients when final real
actuator telemetry (RCOU) is absent.  RC override is a controller input, not a
plant input, especially in ALT_HOLD.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG = ROOT / "localization.db3"
DEFAULT_OUTPUT = ROOT / "analysis" / "han_preflight_28_100"
START_S = 28.0
END_S = 100.0
AXES = ("yaw", "forward", "lateral", "heave")


@dataclass(frozen=True)
class Stream:
    time_s: np.ndarray
    values: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--start", type=float, default=START_S)
    parser.add_argument("--end", type=float, default=END_S)
    return parser.parse_args()


def topic_map(connection: sqlite3.Connection) -> dict[str, tuple[int, str]]:
    return {
        name: (topic_id, message_type)
        for topic_id, name, message_type in connection.execute(
            "SELECT id,name,type FROM topics"
        )
    }


def rows_in_window(
    connection: sqlite3.Connection,
    topic_id: int,
    start_ns: int,
    end_ns: int,
):
    yield from connection.execute(
        """
        SELECT timestamp,data FROM messages
        WHERE topic_id=? AND timestamp>=? AND timestamp<=?
        ORDER BY timestamp
        """,
        (topic_id, start_ns, end_ns),
    )


def load_rc(
    connection: sqlite3.Connection,
    topics: dict[str, tuple[int, str]],
    start_ns: int,
    end_ns: int,
) -> Stream:
    topic_id, _ = topics["/mavros/rc/override"]
    last = np.full(18, 1500.0)
    records = []
    for timestamp_ns, data in rows_in_window(
        connection, topic_id, start_ns, end_ns
    ):
        message = deserialize_message(bytes(data), OverrideRCIn)
        for index, value in enumerate(message.channels):
            if value not in (0, 65535):
                last[index] = value
        records.append(
            (
                (timestamp_ns - start_ns) * 1.0e-9,
                last[3],
                last[4],
                last[5],
                last[2],
            )
        )
    values = np.asarray(records, dtype=float)
    if not values.size:
        return Stream(np.empty(0), np.empty((0, 4)))
    return Stream(values[:, 0], values[:, 1:])


def load_imu(
    connection: sqlite3.Connection,
    topics: dict[str, tuple[int, str]],
    start_ns: int,
    end_ns: int,
) -> Stream:
    topic_id, _ = topics["/mavros/imu/data"]
    records = []
    for timestamp_ns, data in rows_in_window(
        connection, topic_id, start_ns, end_ns
    ):
        message = deserialize_message(bytes(data), Imu)
        records.append(
            (
                (timestamp_ns - start_ns) * 1.0e-9,
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            )
        )
    values = np.asarray(records, dtype=float)
    return Stream(values[:, 0], values[:, 1:]) if values.size else Stream(np.empty(0), np.empty((0, 3)))


def load_dvl_twist(
    connection: sqlite3.Connection,
    topics: dict[str, tuple[int, str]],
    start_ns: int,
    end_ns: int,
) -> Stream:
    topic_id, _ = topics["/dvl/twist"]
    records = []
    for timestamp_ns, data in rows_in_window(
        connection, topic_id, start_ns, end_ns
    ):
        message = deserialize_message(bytes(data), TwistWithCovarianceStamped)
        records.append(
            (
                (timestamp_ns - start_ns) * 1.0e-9,
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            )
        )
    values = np.asarray(records, dtype=float)
    return Stream(values[:, 0], values[:, 1:]) if values.size else Stream(np.empty(0), np.empty((0, 3)))


def load_odom_twist(
    connection: sqlite3.Connection,
    topics: dict[str, tuple[int, str]],
    start_ns: int,
    end_ns: int,
) -> Stream:
    topic_id, _ = topics["/odometry/filtered"]
    records = []
    for timestamp_ns, data in rows_in_window(
        connection, topic_id, start_ns, end_ns
    ):
        message = deserialize_message(bytes(data), Odometry)
        records.append(
            (
                (timestamp_ns - start_ns) * 1.0e-9,
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
                message.twist.twist.angular.z,
            )
        )
    values = np.asarray(records, dtype=float)
    return Stream(values[:, 0], values[:, 1:]) if values.size else Stream(np.empty(0), np.empty((0, 4)))


def stream_quality(stream: Stream, expected_columns: int) -> dict[str, object]:
    count = int(stream.time_s.size)
    if count < 2:
        return {
            "samples": count,
            "rate_hz": None,
            "max_gap_s": None,
            "nonfinite_values": int(np.size(stream.values) - np.isfinite(stream.values).sum()),
            "columns": expected_columns,
        }
    span = float(stream.time_s[-1] - stream.time_s[0])
    gaps = np.diff(stream.time_s)
    return {
        "samples": count,
        "span_s": span,
        "rate_hz": float((count - 1) / span) if span > 0 else None,
        "median_period_s": float(np.median(gaps)),
        "max_gap_s": float(np.max(gaps)),
        "nonfinite_values": int(np.size(stream.values) - np.isfinite(stream.values).sum()),
        "columns": expected_columns,
    }


def standardized_singular_values(matrix: np.ndarray) -> tuple[np.ndarray, float]:
    centered = matrix - np.mean(matrix, axis=0)
    scale = np.std(centered, axis=0)
    keep = scale > 1.0e-9
    standardized = centered[:, keep] / scale[keep]
    singular_values = np.linalg.svd(standardized, compute_uv=False)
    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values.size and singular_values[-1] > 1.0e-12
        else math.inf
    )
    return singular_values, condition


def main() -> None:
    args = parse_args()
    if args.end <= args.start:
        raise SystemExit("--end must be greater than --start")
    args.output.mkdir(parents=True, exist_ok=True)

    connection = sqlite3.connect(args.bag)
    topics = topic_map(connection)
    bag_start_ns = int(connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0])
    start_ns = bag_start_ns + round(args.start * 1.0e9)
    end_ns = bag_start_ns + round(args.end * 1.0e9)

    required_observations = (
        "/mavros/rc/override",
        "/mavros/imu/data",
        "/dvl/twist",
        "/odometry/filtered",
    )
    missing_observations = [name for name in required_observations if name not in topics]
    if missing_observations:
        raise SystemExit(f"missing observation topics: {missing_observations}")

    rc = load_rc(connection, topics, start_ns, end_ns)
    imu = load_imu(connection, topics, start_ns, end_ns)
    dvl = load_dvl_twist(connection, topics, start_ns, end_ns)
    odom = load_odom_twist(connection, topics, start_ns, end_ns)
    connection.close()

    grid = np.arange(0.0, args.end - args.start, 0.1)
    commands = np.column_stack(
        [np.interp(grid, rc.time_s, rc.values[:, column]) for column in range(4)]
    )
    commands = (commands - 1500.0) / 400.0
    command_singular_values, command_condition = standardized_singular_values(commands)
    command_std = np.std(commands, axis=0)

    u = np.interp(grid, dvl.time_s, dvl.values[:, 0])
    v = np.interp(grid, dvl.time_s, dvl.values[:, 1])
    r = np.interp(grid, imu.time_s, imu.values[:, 2])
    horizontal_features = np.column_stack(
        (
            commands[:, 0],
            commands[:, 1],
            commands[:, 2],
            u,
            v,
            r,
            np.abs(u) * u,
            np.abs(v) * v,
            np.abs(r) * r,
            u * r,
            v * r,
        )
    )
    feature_names = (
        "rc_yaw",
        "rc_forward",
        "rc_lateral",
        "dvl_u",
        "dvl_v",
        "gyro_r",
        "|u|u",
        "|v|v",
        "|r|r",
        "u*r",
        "v*r",
    )
    feature_singular_values, feature_condition = standardized_singular_values(
        horizontal_features
    )
    feature_correlation = np.corrcoef(horizontal_features, rowvar=False)

    rc_out_aliases = (
        "/mavros/rc/out",
        "/mavros/servo_output_raw",
        "/uuv_mujoco/rc/out_override",
    )
    present_rc_out = [name for name in rc_out_aliases if name in topics]
    weak_axes = [
        AXES[index] for index, value in enumerate(command_std) if value < 0.05
    ]
    output = {
        "decision": "blocked_missing_real_rcou" if not present_rc_out else "eligible_for_bounded_fit",
        "fit_performed": False,
        "reason": (
            "Final real actuator telemetry is absent. RC override cannot replace RCOU in ALT_HOLD, "
            "so actuator/controller and hydrodynamic effects are not separable."
            if not present_rc_out
            else "Final real actuator telemetry is present; proceed only after source and frame checks."
        ),
        "source": str(args.bag.resolve()),
        "window_s_from_bag_start": [args.start, args.end],
        "topic_presence": {
            "real_rcou_aliases_checked": list(rc_out_aliases),
            "real_rcou_topics_present": present_rc_out,
            "required_observations_missing": missing_observations,
        },
        "stream_quality": {
            "rc_override": stream_quality(rc, 4),
            "imu": stream_quality(imu, 3),
            "dvl_twist": stream_quality(dvl, 3),
            "filtered_odom_twist": stream_quality(odom, 4),
        },
        "command_axis_order": list(AXES),
        "command_standard_deviation_normalized": command_std.tolist(),
        "command_singular_values_standardized": command_singular_values.tolist(),
        "command_condition_number_standardized": command_condition,
        "weakly_excited_axes": weak_axes,
        "horizontal_feature_names": list(feature_names),
        "horizontal_feature_singular_values_standardized": feature_singular_values.tolist(),
        "horizontal_feature_condition_number_standardized": feature_condition,
        "horizontal_feature_correlation": feature_correlation.tolist(),
        "safe_conclusions": [
            "IMU yaw rate and DVL x/y are available for residual validation.",
            "RC override can describe requested excitation but is not authoritative plant input.",
            "No hydrodynamic coefficient or thruster correction is accepted from this bag.",
        ],
        "next_required_data": [
            "final real /mavros/rc/out at the highest practical rate",
            "independent yaw, sway, and surge excitation segments",
            "IMU angular velocity and DVL x/y recorded on the same clock",
            "depth/Bar30-derived depth for vertical validation",
        ],
    }
    (args.output / "han_preflight.json").write_text(
        json.dumps(output, indent=2) + "\n", encoding="utf-8"
    )

    fig, axes = plt.subplots(2, 2, figsize=(14, 10), facecolor="white")
    ax_command, ax_response, ax_singular, ax_corr = axes.flat
    for index, name in enumerate(AXES):
        ax_command.plot(grid, commands[:, index], linewidth=1.4, label=name)
    ax_command.set_title("RC override excitation", loc="left", fontweight="semibold")
    ax_command.set_xlabel("bag window time [s]")
    ax_command.set_ylabel("normalized command")
    ax_command.legend(frameon=False, ncol=2)

    ax_response.plot(grid, r, linewidth=1.6, label="gyro yaw rate [rad/s]")
    ax_response.plot(grid, u, linewidth=1.4, label="DVL u [m/s]")
    ax_response.plot(grid, v, linewidth=1.4, label="DVL v [m/s]")
    ax_response.set_title("Measured horizontal response", loc="left", fontweight="semibold")
    ax_response.set_xlabel("bag window time [s]")
    ax_response.legend(frameon=False)

    ax_singular.semilogy(
        np.arange(1, feature_singular_values.size + 1),
        feature_singular_values,
        marker="o",
        color="#2563EB",
    )
    ax_singular.set_title("Horizontal feature singular values", loc="left", fontweight="semibold")
    ax_singular.set_xlabel("mode")
    ax_singular.set_ylabel("singular value")

    image = ax_corr.imshow(feature_correlation, cmap="coolwarm", vmin=-1.0, vmax=1.0)
    ax_corr.set_xticks(range(len(feature_names)), feature_names, rotation=45, ha="right")
    ax_corr.set_yticks(range(len(feature_names)), feature_names)
    ax_corr.set_title("Feature correlation", loc="left", fontweight="semibold")
    fig.colorbar(image, ax=ax_corr, fraction=0.046, pad=0.04)

    for axis in axes.flat:
        axis.grid(True, color="#D9DEE7", linewidth=0.7, alpha=0.7)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        "HAN plant-identification preflight — bag seconds 28 to 100",
        x=0.06,
        y=0.98,
        ha="left",
        fontsize=17,
        fontweight="bold",
    )
    fig.text(
        0.06,
        0.945,
        "Fit blocked: final real RCOU is absent; plots characterize available excitation and response only.",
        color="#5F6673",
    )
    fig.subplots_adjust(left=0.08, right=0.98, top=0.90, bottom=0.10, hspace=0.32, wspace=0.24)
    fig.savefig(args.output / "han_preflight.png", dpi=180, facecolor="white")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
