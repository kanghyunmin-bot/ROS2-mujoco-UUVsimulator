#!/usr/bin/env python3
"""Compare the verified single-owner baseline with path-calibration candidates."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ANALYSIS = ROOT / "analysis"
RUNS = [
    ("Verified baseline", "localization_rc_replay_28_100_althold_single_owner_contract_20260722", "#6B7280"),
    ("v1 translation drag", "localization_rc_replay_28_100_cal_v1", "#D97706"),
    ("v2 yaw moment 1.7", "localization_rc_replay_28_100_cal_v2", "#7C3AED"),
    ("v3 yaw moment 3.0", "localization_rc_replay_28_100_cal_v3", "#2563EB"),
    ("v4 reduced yaw drag", "localization_rc_replay_28_100_cal_v4", "#059669"),
    ("v5 yaw moment 5.0", "localization_rc_replay_28_100_cal_v5", "#DC2626"),
    ("v6 yaw command 5.4", "localization_rc_replay_28_100_cal_v6", "#0891B2"),
    ("v7 LS yaw command 5.05", "localization_rc_replay_28_100_cal_v7", "#DB2777"),
    ("v8 ridge 4-axis matrix", "localization_rc_replay_28_100_cal_v8", "#92400E"),
    ("v9 midpoint yaw command 5.225", "localization_rc_replay_28_100_cal_v9", "#65A30D"),
]


def load_csv(path: Path) -> np.ndarray:
    with path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    keys = ("t_s", "forward_m", "lateral_m", "relative_z_m", "relative_yaw_rad")
    return np.asarray([[float(row[key]) for key in keys] for row in rows])


def style(axis, title: str) -> None:
    axis.set_title(title, loc="left", fontweight="semibold")
    axis.grid(True, color="#D9DEE7", linewidth=0.8, alpha=0.75)
    axis.spines[["top", "right"]].set_visible(False)


def main() -> None:
    real = load_csv(ANALYSIS / RUNS[0][1] / "real_path_28_100_aligned.csv")
    runs = []
    for label, directory, color in RUNS:
        root = ANALYSIS / directory
        runs.append((label, load_csv(root / "sim_path_28_100_aligned.csv"), json.loads((root / "comparison_summary.json").read_text()), color))

    fig, axes = plt.subplots(2, 2, figsize=(14, 11), facecolor="white")
    ax_xy, ax_sep, ax_yaw, ax_metrics = axes.flat
    ax_xy.plot(real[:, 1], real[:, 2], "--", color="#111827", linewidth=2.2, label="Real localization")
    ax_yaw.plot(real[:, 0], np.degrees(real[:, 4]), "--", color="#111827", linewidth=2.2, label="Real localization")

    for label, rows, _, color in runs:
        ax_xy.plot(rows[:, 1], rows[:, 2], color=color, linewidth=1.8, label=label)
        t = rows[:, 0]
        dx = rows[:, 1] - np.interp(t, real[:, 0], real[:, 1])
        dy = rows[:, 2] - np.interp(t, real[:, 0], real[:, 2])
        ax_sep.plot(t, np.hypot(dx, dy), color=color, linewidth=1.8, label=label)
        ax_yaw.plot(t, np.degrees(rows[:, 4]), color=color, linewidth=1.8, label=label)

    ax_xy.axis("equal")
    ax_xy.set_xlabel("forward from start [m]")
    ax_xy.set_ylabel("lateral from start [m]")
    ax_xy.legend(frameon=False, fontsize=8)
    style(ax_xy, "XY trajectory")
    ax_sep.set_xlabel("elapsed simulation time [s]")
    ax_sep.set_ylabel("XY separation [m]")
    style(ax_sep, "Separation from Real")
    ax_yaw.set_xlabel("elapsed simulation time [s]")
    ax_yaw.set_ylabel("yaw - yaw0 [deg]")
    style(ax_yaw, "Relative yaw")

    labels = [item[0] for item in runs]
    x = np.arange(len(labels))
    xy = [item[2]["xy_rmse_m"] for item in runs]
    yaw = [item[2]["yaw_rmse_deg"] / 20.0 for item in runs]
    width = 0.38
    bars_xy = ax_metrics.bar(x - width / 2, xy, width, color=[item[3] for item in runs], label="XY RMSE [m]")
    bars_yaw = ax_metrics.bar(x + width / 2, yaw, width, color=[item[3] for item in runs], alpha=0.40, label="Yaw RMSE / 20")
    ax_metrics.bar_label(bars_xy, fmt="%.2f", fontsize=8, padding=2)
    ax_metrics.bar_label(bars_yaw, labels=[f"{item[2]['yaw_rmse_deg']:.1f}°" for item in runs], fontsize=8, padding=2)
    ax_metrics.set_xticks(x, [f"v{i}" if i else "base" for i in range(len(runs))])
    ax_metrics.set_ylabel("metric scale")
    ax_metrics.legend(frameon=False, fontsize=9)
    style(ax_metrics, "Outcome metrics")

    fig.suptitle("Sim-to-real path calibration — bag seconds 28 to 100", x=0.07, y=0.975, ha="left", fontsize=18, fontweight="bold")
    fig.text(0.07, 0.943, "Same single-owner RC stream, initial state, ALT_HOLD, and simulation-time axis; no fitted path registration or scale.", color="#5F6673", fontsize=10)
    fig.text(0.07, 0.02, "Yaw bars are divided by 20 only to share an axis with metre-scale XY RMSE; degree labels show the original values.", color="#5F6673", fontsize=9)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.91, bottom=0.08, hspace=0.30, wspace=0.24)
    output = ANALYSIS / "real_vs_path_calibration_v0_v9.png"
    fig.savefig(output, dpi=170, facecolor="white")
    print(output)


if __name__ == "__main__":
    main()
