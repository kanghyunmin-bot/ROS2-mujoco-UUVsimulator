#!/usr/bin/env python3
"""Diagnose the verified single-owner replay and its RC transport fidelity."""

from __future__ import annotations

import csv
import json
import sqlite3
from collections import Counter
from pathlib import Path

import numpy as np
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut
from rclpy.serialization import deserialize_message


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "analysis/localization_rc_replay_28_100_althold_single_owner_contract_20260722"
OLD = ROOT / "analysis/localization_rc_replay_28_100_althold_single_owner_simtime"
DB = RUN / "sim_result/sim_result_0.db3"


def load_csv(path: Path) -> np.ndarray:
    with path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    keys = ("t_s", "forward_m", "lateral_m", "relative_z_m", "relative_yaw_rad")
    return np.asarray([[float(row[key]) for key in keys] for row in rows])


def load_messages(connection, topics, name, message_type):
    query = "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp"
    return [
        (timestamp, deserialize_message(bytes(data), message_type))
        for timestamp, data in connection.execute(query, (topics[name],))
    ]


def classify_override(message: OverrideRCIn) -> str:
    channels = list(message.channels)
    if channels[2] == 1500 and all(
        value == OverrideRCIn.CHAN_NOCHANGE
        for index, value in enumerate(channels)
        if index != 2
    ):
        return "ch3_only_neutral"
    if channels[8] == 1100:
        return "joy_full_frame"
    return "other"


def rc_input_fidelity(overrides, rc_inputs, channel_index: int) -> dict:
    override_times = np.asarray([timestamp for timestamp, _ in overrides])
    input_times = np.asarray([timestamp for timestamp, _ in rc_inputs])
    indices = np.searchsorted(override_times, input_times, side="right") - 1
    pairs = []
    invalid = {OverrideRCIn.CHAN_RELEASE, OverrideRCIn.CHAN_NOCHANGE}
    for index, (_, rc_input) in zip(indices, rc_inputs):
        if index < 0 or len(rc_input.channels) <= channel_index:
            continue
        command = overrides[index][1].channels[channel_index]
        applied = rc_input.channels[channel_index]
        if command in invalid or applied in invalid:
            continue
        pairs.append((command, applied))
    command = np.asarray([pair[0] for pair in pairs], dtype=float)
    applied = np.asarray([pair[1] for pair in pairs], dtype=float)
    correlation = np.corrcoef(command, applied)[0, 1]
    return {
        "samples": len(pairs),
        "exact_fraction": float(np.mean(command == applied)),
        "mae_pwm": float(np.mean(np.abs(command - applied))),
        "correlation": float(correlation),
    }


def main() -> None:
    connection = sqlite3.connect(DB)
    topics = dict(connection.execute("SELECT name,id FROM topics"))
    overrides = load_messages(connection, topics, "/mavros/rc/override", OverrideRCIn)
    rc_inputs = load_messages(connection, topics, "/mavros/rc/in", RCIn)
    rc_outputs = load_messages(connection, topics, "/mavros/rc/out", RCOut)
    start_ns, end_ns = overrides[0][0], overrides[-1][0]
    rc_inputs = [row for row in rc_inputs if start_ns <= row[0] <= end_ns]
    rc_outputs = [row for row in rc_outputs if start_ns <= row[0] <= end_ns]
    connection.close()

    real = load_csv(RUN / "real_path_28_100_aligned.csv")
    sim = load_csv(RUN / "sim_path_28_100_aligned.csv")
    real_xy = np.column_stack(
        [np.interp(sim[:, 0], real[:, 0], real[:, column]) for column in (1, 2)]
    )
    sim_xy = sim[:, 1:3]
    spatial_scale = float(np.sum(sim_xy * real_xy) / np.sum(sim_xy * sim_xy))
    scaled_error = np.linalg.norm(spatial_scale * sim_xy - real_xy, axis=1)
    complex_sim = sim_xy[:, 0] + 1j * sim_xy[:, 1]
    complex_real = real_xy[:, 0] + 1j * real_xy[:, 1]
    similarity = np.vdot(complex_sim, complex_real) / np.vdot(complex_sim, complex_sim)
    similarity_error = np.abs(similarity * complex_sim - complex_real)

    current = json.loads((RUN / "comparison_summary.json").read_text())
    previous = json.loads((OLD / "comparison_summary.json").read_text())
    diagnostics = {
        "ownership": {
            "publisher_count_during_replay": 1,
            "override_messages": len(overrides),
            "override_classes": dict(Counter(classify_override(msg) for _, msg in overrides)),
        },
        "transport": {
            "rc_in_samples_in_window": len(rc_inputs),
            "rc_out_samples_in_window": len(rc_outputs),
            "rc_in_vs_latest_override": {
                f"channel_{index + 1}": rc_input_fidelity(overrides, rc_inputs, index)
                for index in (2, 3, 4)
            },
        },
        "path": current,
        "repeatability_vs_previous_single_owner": {
            "xy_rmse_change_m": current["xy_rmse_m"] - previous["xy_rmse_m"],
            "final_xy_error_change_m": current["xy_final_error_m"]
            - previous["xy_final_error_m"],
            "sim_path_length_change_m": current["sim_xy_path_length_m"]
            - previous["sim_xy_path_length_m"],
            "yaw_rmse_change_deg": current["yaw_rmse_deg"] - previous["yaw_rmse_deg"],
        },
        "diagnostic_fits_not_used_for_reported_alignment": {
            "best_xy_spatial_scale_without_rotation": spatial_scale,
            "scaled_xy_rmse_m": float(np.sqrt(np.mean(scaled_error**2))),
            "best_similarity_scale": float(abs(similarity)),
            "best_similarity_rotation_deg": float(np.degrees(np.angle(similarity))),
            "similarity_xy_rmse_m": float(np.sqrt(np.mean(similarity_error**2))),
        },
    }
    output = RUN / "contract_diagnostics.json"
    output.write_text(json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(diagnostics, indent=2))


if __name__ == "__main__":
    main()
