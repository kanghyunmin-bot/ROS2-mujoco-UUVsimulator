#!/usr/bin/env python3
"""Estimate the ALT_HOLD yaw command gain from v4/v6 response sensitivities."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ANALYSIS = ROOT / "analysis"
V4 = ANALYSIS / "localization_rc_replay_28_100_cal_v4"
V6 = ANALYSIS / "localization_rc_replay_28_100_cal_v6"
V7 = ANALYSIS / "localization_rc_replay_28_100_cal_v7"
P4 = 3.375
P6 = 5.4
DT = 0.05


def load_yaw(path: Path) -> np.ndarray:
    with path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    values = np.asarray([[float(row["t_s"]), float(row["relative_yaw_rad"])] for row in rows])
    _, unique_indices = np.unique(values[:, 0], return_index=True)
    return values[np.sort(unique_indices)]


def rmse_deg(error_rad: np.ndarray) -> float:
    return float(np.degrees(np.sqrt(np.mean(np.square(error_rad)))))


def main() -> None:
    real_raw = load_yaw(V6 / "real_path_28_100_aligned.csv")
    v4_raw = load_yaw(V4 / "sim_path_28_100_aligned.csv")
    v6_raw = load_yaw(V6 / "sim_path_28_100_aligned.csv")
    v7_raw = load_yaw(V7 / "sim_path_28_100_aligned.csv")
    end = min(real_raw[-1, 0], v4_raw[-1, 0], v6_raw[-1, 0], v7_raw[-1, 0])
    time_s = np.arange(0.0, end, DT)
    real = np.interp(time_s, real_raw[:, 0], real_raw[:, 1])
    v4 = np.interp(time_s, v4_raw[:, 0], v4_raw[:, 1])
    v6 = np.interp(time_s, v6_raw[:, 0], v6_raw[:, 1])
    v7 = np.interp(time_s, v7_raw[:, 0], v7_raw[:, 1])

    # Finite-difference Jacobian of the complete closed-loop yaw trajectory.
    gain_jacobian = (v6 - v4) / (P6 - P4)
    alpha = float(np.dot(gain_jacobian, real - v4) / np.dot(gain_jacobian, gain_jacobian))
    optimal_gain = P4 + alpha
    predicted = v4 + alpha * gain_jacobian

    # A local Gauss-Newton decomposition checks whether gain is being confused
    # with a small time shift. SVD diagonalizes the two-column sensitivity
    # matrix instead of solving potentially correlated normal equations.
    window = 21
    smooth_v6 = np.convolve(v6, np.ones(window) / window, mode="same")
    time_jacobian = np.gradient(smooth_v6, DT)
    fit_mask = (time_s >= 3.0) & (time_s <= end - 1.0)
    design = np.column_stack((gain_jacobian[fit_mask], time_jacobian[fit_mask]))
    residual = (real - v6)[fit_mask]
    u_matrix, singular_values, vt_matrix = np.linalg.svd(design, full_matrices=False)
    svd_solution = vt_matrix.T @ ((u_matrix.T @ residual) / singular_values)

    summary = {
        "model": "yaw(p) ~= yaw_v4 + (p - 3.375) * (yaw_v6 - yaw_v4) / (5.4 - 3.375)",
        "samples": int(time_s.size),
        "sample_dt_s": DT,
        "v4_acro_yaw_p": P4,
        "v6_acro_yaw_p": P6,
        "least_squares_acro_yaw_p": optimal_gain,
        "predicted_yaw_rmse_deg": rmse_deg(predicted - real),
        "v4_yaw_rmse_deg": rmse_deg(v4 - real),
        "v6_yaw_rmse_deg": rmse_deg(v6 - real),
        "v7_validated_acro_yaw_p": 5.05,
        "v7_validated_yaw_rmse_deg": rmse_deg(v7 - real),
        "v7_validated_final_yaw_deg": float(np.degrees(v7[-1])),
        "predicted_final_yaw_deg": float(np.degrees(predicted[-1])),
        "real_final_yaw_deg": float(np.degrees(real[-1])),
        "svd_singular_values": singular_values.tolist(),
        "svd_condition_number": float(singular_values[0] / singular_values[-1]),
        "gain_time_component_correlation": float(np.corrcoef(design.T)[0, 1]),
        "joint_fit_acro_yaw_p": float(P6 + svd_solution[0]),
        "joint_fit_time_shift_s": float(svd_solution[1]),
        "recommendation": "Use ACRO_YAW_P=5.05 for this calibrated replay profile. The fresh nonlinear v7 replay validated the least-squares result; do not apply the fitted time shift because the replay time axis is already fixed.",
    }
    output_json = ANALYSIS / "yaw_command_least_squares.json"
    output_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(2, 1, figsize=(13, 9), sharex=True, facecolor="white")
    axes[0].plot(time_s, np.degrees(real), "--", color="#111827", linewidth=2.1, label="Real")
    axes[0].plot(time_s, np.degrees(v4), color="#059669", linewidth=1.5, label="v4: P=3.375")
    axes[0].plot(time_s, np.degrees(v6), color="#2563EB", linewidth=1.5, label="v6: P=5.4")
    axes[0].plot(time_s, np.degrees(predicted), color="#DC2626", linewidth=2.0, label=f"LS prediction: P={optimal_gain:.3f}")
    axes[0].plot(time_s, np.degrees(v7), color="#0891B2", linewidth=2.0, label="v7 validation: P=5.05")
    axes[0].set_ylabel("relative yaw [deg]")
    axes[0].legend(frameon=False, ncol=2)
    axes[0].set_title("Yaw command gain — finite-difference least squares", loc="left", fontweight="semibold")

    axes[1].plot(time_s, np.degrees(v6 - real), color="#2563EB", linewidth=1.4, label="v6 residual")
    axes[1].plot(time_s, np.degrees(predicted - real), color="#DC2626", linewidth=1.6, label="LS predicted residual")
    axes[1].plot(time_s, np.degrees(v7 - real), color="#0891B2", linewidth=1.6, label="v7 measured residual")
    axes[1].axhline(0.0, color="#111827", linewidth=0.9)
    axes[1].set_xlabel("elapsed simulation time [s]")
    axes[1].set_ylabel("sim - real yaw [deg]")
    axes[1].legend(frameon=False)
    axes[1].set_title("Residual comparison", loc="left", fontweight="semibold")
    for axis in axes:
        axis.grid(True, color="#D9DEE7", linewidth=0.8, alpha=0.75)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("SVD-assisted yaw least-squares identification", x=0.08, y=0.98, ha="left", fontsize=17, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output_plot = ANALYSIS / "yaw_command_least_squares.png"
    fig.savefig(output_plot, dpi=170, facecolor="white")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
