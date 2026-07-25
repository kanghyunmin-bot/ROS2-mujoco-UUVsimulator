#!/usr/bin/env python3
"""Identify real/sim four-axis RC response and a regularized correction matrix."""

from __future__ import annotations

import csv
import json
import sqlite3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mavros_msgs.msg import OverrideRCIn
from rclpy.serialization import deserialize_message


ROOT = Path(__file__).resolve().parents[1]
ANALYSIS = ROOT / "analysis"
RUN = ANALYSIS / "localization_rc_replay_28_100_cal_v7"
RC_DB = ANALYSIS / "localization_rc_override_28_100_single_owner" / "localization_rc_override_28_100_single_owner_0.db3"
AXES = ("yaw", "forward", "lateral", "heave")
DT = 0.1
LAG_COUNT = 21
RIDGE = 0.5


def load_path(path: Path) -> np.ndarray:
    with path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    keys = ("t_s", "forward_m", "lateral_m", "relative_z_m", "relative_yaw_rad")
    values = np.asarray([[float(row[key]) for key in keys] for row in rows])
    _, indices = np.unique(values[:, 0], return_index=True)
    return values[np.sort(indices)]


def load_rc() -> np.ndarray:
    connection = sqlite3.connect(RC_DB)
    topic_id = connection.execute("SELECT id FROM topics WHERE name='/mavros/rc/override'").fetchone()[0]
    last = [1500] * 18
    rows = []
    for timestamp_ns, data in connection.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,)
    ):
        message = deserialize_message(bytes(data), OverrideRCIn)
        for index, value in enumerate(message.channels):
            if value not in (0, 65535):
                last[index] = value
        # RC4 yaw, RC5 forward, RC6 lateral, RC3 heave.
        rows.append((timestamp_ns * 1.0e-9, last[3], last[4], last[5], last[2]))
    connection.close()
    values = np.asarray(rows, dtype=float)
    values[:, 0] -= values[0, 0]
    return values


def smooth(values: np.ndarray, width: int = 11) -> np.ndarray:
    pad = width // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, np.ones(width) / width, mode="valid")


def body_response(path: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    state = np.column_stack([np.interp(time_s, path[:, 0], path[:, column]) for column in range(1, 5)])
    state = np.column_stack([smooth(state[:, column]) for column in range(4)])
    dx, dy, dz = np.gradient(state[:, :3], DT, axis=0).T
    heading = state[:, 3]
    cosine, sine = np.cos(heading), np.sin(heading)
    return np.column_stack(
        (
            np.gradient(heading, DT),
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            dz,
        )
    )


def lagged_design(commands: np.ndarray) -> np.ndarray:
    return np.column_stack([np.roll(commands, lag, axis=0) for lag in range(LAG_COUNT)])[LAG_COUNT - 1 :]


def fit_dc_gain(commands: np.ndarray, response: np.ndarray, selection: np.ndarray | None = None) -> np.ndarray:
    design = lagged_design(commands)
    target = response[LAG_COUNT - 1 :]
    if selection is not None:
        design, target = design[selection], target[selection]
    design = np.column_stack((design, np.ones(design.shape[0])))
    regularizer = np.eye(design.shape[1]) * RIDGE
    regularizer[-1, -1] = 0.0
    coefficients = np.linalg.solve(design.T @ design + regularizer, design.T @ target)[:-1]
    return coefficients.reshape(LAG_COUNT, len(AXES), len(AXES)).sum(axis=0).T


def diagonal_fit(sim_gain: np.ndarray, real_gain: np.ndarray) -> np.ndarray:
    denominator = np.sum(np.square(sim_gain), axis=0)
    return np.divide(
        np.sum(sim_gain * real_gain, axis=0),
        denominator,
        out=np.full_like(denominator, np.nan),
        where=denominator > 1.0e-10,
    )


def main() -> None:
    real_path = load_path(RUN / "real_path_28_100_aligned.csv")
    sim_path = load_path(RUN / "sim_path_28_100_aligned.csv")
    rc = load_rc()
    end = min(real_path[-1, 0], sim_path[-1, 0], rc[-1, 0])
    time_s = np.arange(0.0, end, DT)
    commands = np.column_stack([np.interp(time_s, rc[:, 0], rc[:, column]) for column in range(1, 5)])
    commands = (commands - 1500.0) / 400.0
    centered_commands = commands - np.mean(commands, axis=0)
    input_singular_values = np.linalg.svd(centered_commands, compute_uv=False)

    real_gain = fit_dc_gain(commands, body_response(real_path, time_s))
    sim_gain = fit_dc_gain(commands, body_response(sim_path, time_s))
    correction = np.linalg.pinv(sim_gain, rcond=0.05) @ real_gain
    diagonal = diagonal_fit(sim_gain, real_gain)
    correction_singular_values = np.linalg.svd(correction, compute_uv=False)

    fit_samples = time_s.size - LAG_COUNT + 1
    first_half = np.arange(fit_samples) < fit_samples // 2
    second_half = ~first_half
    real_response = body_response(real_path, time_s)
    sim_response = body_response(sim_path, time_s)
    split_diagonal = {}
    for name, selection in (("first_half", first_half), ("second_half", second_half)):
        split_real = fit_dc_gain(commands, real_response, selection)
        split_sim = fit_dc_gain(commands, sim_response, selection)
        split_values = diagonal_fit(split_sim, split_real)
        split_diagonal[name] = [float(value) if np.isfinite(value) else None for value in split_values]

    command_std = np.std(commands, axis=0)
    weak_axes = [AXES[index] for index, value in enumerate(command_std) if value < 0.05]
    summary = {
        "method": "21-tap FIR DC-gain identification at 10 Hz with ridge regularization; correction solves H_sim C ~= H_real",
        "axis_order": list(AXES),
        "input_standard_deviation": command_std.tolist(),
        "input_singular_values": input_singular_values.tolist(),
        "input_condition_number": float(input_singular_values[0] / input_singular_values[-1]),
        "weakly_excited_axes": weak_axes,
        "real_dc_gain": real_gain.tolist(),
        "sim_dc_gain": sim_gain.tolist(),
        "unconstrained_correction_matrix": correction.tolist(),
        "unconstrained_correction_singular_values": correction_singular_values.tolist(),
        "unconstrained_correction_condition_number": float(correction_singular_values[0] / correction_singular_values[-1]),
        "axis_preserving_diagonal_correction": diagonal.tolist(),
        "split_window_diagonal_correction": split_diagonal,
        "interpretation": {
            "yaw": "Identifiable and already close after ACRO_YAW_P=5.05.",
            "forward": "Moderately identifiable; the FIR DC-gain estimate indicates sim authority is high, but the trajectory itself is already close.",
            "lateral": "Not identifiable from this bag because RC6 excitation is too small; correction is confounded with forward-to-sway hydrodynamics/current.",
            "heave": "Not identifiable from this bag because RC3 excitation is too small and ALT_HOLD closes the depth loop.",
        },
        "recommendation": "Do not apply the full 4x4 or per-thruster correction from this bag. Record independent RC5, RC6, RC3, and RC4 excitation segments plus real RCOU before solving individual thruster gains.",
    }
    output_json = ANALYSIS / "thruster_axis_least_squares.json"
    output_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5), facecolor="white")
    matrices = (real_gain, sim_gain, correction)
    titles = ("Real FIR DC gain", "Sim FIR DC gain", "Unconstrained correction C")
    for axis, matrix, title in zip(axes, matrices, titles):
        limit = max(float(np.max(np.abs(matrix))), 1.0e-6)
        image = axis.imshow(matrix, cmap="coolwarm", vmin=-limit, vmax=limit)
        axis.set_xticks(range(len(AXES)), AXES, rotation=30, ha="right")
        axis.set_yticks(range(len(AXES)), AXES)
        axis.set_xlabel("input axis")
        axis.set_ylabel("response/corrected input axis")
        axis.set_title(title, loc="left", fontweight="semibold")
        for row in range(matrix.shape[0]):
            for column in range(matrix.shape[1]):
                axis.text(column, row, f"{matrix[row, column]:.2f}", ha="center", va="center", fontsize=8)
        fig.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
    fig.suptitle("Four-axis thruster least-squares identification", x=0.04, y=1.02, ha="left", fontsize=17, fontweight="bold")
    fig.tight_layout()
    output_plot = ANALYSIS / "thruster_axis_least_squares.png"
    fig.savefig(output_plot, dpi=170, bbox_inches="tight", facecolor="white")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
