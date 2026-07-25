#!/usr/bin/env python3
"""Focused comparison of the v6/v7 yaw-gain endpoints and their midpoint."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ANALYSIS = ROOT / "analysis"
RUNS = [
    ("v6  P=5.400", "localization_rc_replay_28_100_cal_v6", "#0891B2"),
    ("v9  P=5.225 midpoint", "localization_rc_replay_28_100_cal_v9", "#65A30D"),
    ("v7  P=5.050", "localization_rc_replay_28_100_cal_v7", "#DB2777"),
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
    report = {
        "interpretation": {
            "rmse_optimum": "v7",
            "balanced_midpoint": "v9",
            "note": "v9 improves final-yaw agreement over v7 but does not beat v7's aggregate XY/yaw RMSE.",
        },
        "runs": {},
    }
    for label, directory, color in RUNS:
        root = ANALYSIS / directory
        rows = load_csv(root / "sim_path_28_100_aligned.csv")
        summary = json.loads((root / "comparison_summary.json").read_text())
        runs.append((label, rows, summary, color))
        report["runs"][label.split()[0]] = {
            "xy_rmse_m": summary["xy_rmse_m"],
            "xy_final_error_m": summary["xy_final_error_m"],
            "yaw_rmse_deg": summary["yaw_rmse_deg"],
            "final_yaw_error_deg": summary["sim_final_relative_yaw_deg"]
            - summary["real_final_relative_yaw_deg"],
        }

    fig, axes = plt.subplots(2, 2, figsize=(13, 10), facecolor="white")
    ax_xy, ax_yaw, ax_xy_err, ax_metrics = axes.flat
    ax_xy.plot(real[:, 1], real[:, 2], "--", color="#111827", linewidth=2.4, label="Real")
    ax_yaw.plot(real[:, 0], np.degrees(real[:, 4]), "--", color="#111827", linewidth=2.4, label="Real")

    for label, rows, _, color in runs:
        ax_xy.plot(rows[:, 1], rows[:, 2], color=color, linewidth=2.0, label=label)
        ax_yaw.plot(rows[:, 0], np.degrees(rows[:, 4]), color=color, linewidth=2.0, label=label)
        dx = rows[:, 1] - np.interp(rows[:, 0], real[:, 0], real[:, 1])
        dy = rows[:, 2] - np.interp(rows[:, 0], real[:, 0], real[:, 2])
        ax_xy_err.plot(rows[:, 0], np.hypot(dx, dy), color=color, linewidth=2.0, label=label)

    ax_xy.axis("equal")
    ax_xy.set_xlabel("forward from start [m]")
    ax_xy.set_ylabel("lateral from start [m]")
    ax_xy.legend(frameon=False, fontsize=9)
    style(ax_xy, "XY trajectory")
    ax_yaw.set_xlabel("elapsed simulation time [s]")
    ax_yaw.set_ylabel("yaw - yaw0 [deg]")
    ax_yaw.legend(frameon=False, fontsize=9)
    style(ax_yaw, "Relative yaw")
    ax_xy_err.set_xlabel("elapsed simulation time [s]")
    ax_xy_err.set_ylabel("XY separation [m]")
    style(ax_xy_err, "Distance from real path")

    names = ["v6", "v9", "v7"]
    x = np.arange(3)
    xy = [run[2]["xy_rmse_m"] for run in runs]
    yaw = [run[2]["yaw_rmse_deg"] for run in runs]
    colors = [run[3] for run in runs]
    ax_metrics.bar(x - 0.19, xy, 0.38, color=colors, label="XY RMSE [m]")
    ax_metrics.bar(x + 0.19, np.asarray(yaw) / 20.0, 0.38, color=colors, alpha=0.4, label="Yaw RMSE / 20")
    for i, (xy_value, yaw_value) in enumerate(zip(xy, yaw)):
        ax_metrics.text(i - 0.19, xy_value + 0.025, f"{xy_value:.3f}", ha="center", fontsize=9)
        ax_metrics.text(i + 0.19, yaw_value / 20.0 + 0.025, f"{yaw_value:.1f}°", ha="center", fontsize=9)
    ax_metrics.set_xticks(x, names)
    ax_metrics.set_ylabel("metric scale")
    ax_metrics.legend(frameon=False, fontsize=9)
    style(ax_metrics, "Aggregate error")

    fig.suptitle("Yaw-gain midpoint test — real bag seconds 28 to 100", x=0.07, y=0.975, ha="left", fontsize=17, fontweight="bold")
    fig.text(0.07, 0.943, "Same RC samples, initial state, ALT_HOLD/ARMED mode, plant profile, and simulation-time alignment.", color="#5F6673", fontsize=10)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.90, bottom=0.07, hspace=0.30, wspace=0.24)

    plot_path = ANALYSIS / "real_vs_yaw_midpoint_v6_v7_v9.png"
    report_path = ANALYSIS / "yaw_midpoint_v6_v7_v9.json"
    fig.savefig(plot_path, dpi=180, facecolor="white")
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(plot_path)
    print(report_path)


if __name__ == "__main__":
    main()
