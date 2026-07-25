#!/usr/bin/env python3
"""Compare exact and filtered single-owner RC replays against the real path."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
EXACT = ROOT / "analysis/localization_rc_replay_28_100_althold_simtime"
SINGLE = ROOT / "analysis/localization_rc_replay_28_100_althold_single_owner_simtime"
CONTRACT = ROOT / "analysis/localization_rc_replay_28_100_althold_single_owner_contract_20260722"

REAL_COLOR = "#30343B"
EXACT_COLOR = "#D97706"
SINGLE_COLOR = "#2563EB"
CONTRACT_COLOR = "#059669"
GRID = "#D9DEE7"


def load_csv(path: Path) -> np.ndarray:
    with path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return np.asarray(
        [[float(row[key]) for key in ("t_s", "forward_m", "lateral_m", "relative_z_m", "relative_yaw_rad")] for row in rows]
    )


def xy_separation(reference: np.ndarray, candidate: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    t = candidate[:, 0]
    rx = np.interp(t, reference[:, 0], reference[:, 1])
    ry = np.interp(t, reference[:, 0], reference[:, 2])
    return t, np.hypot(candidate[:, 1] - rx, candidate[:, 2] - ry)


def style(ax, title: str) -> None:
    ax.set_title(title, loc="left", fontweight="semibold")
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.75)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)


def main() -> None:
    real = load_csv(EXACT / "real_path_28_100_aligned.csv")
    exact = load_csv(EXACT / "sim_path_28_100_aligned.csv")
    single = load_csv(SINGLE / "sim_path_28_100_aligned.csv")
    contract = load_csv(CONTRACT / "sim_path_28_100_aligned.csv")
    exact_summary = json.loads((EXACT / "comparison_summary.json").read_text())
    single_summary = json.loads((SINGLE / "comparison_summary.json").read_text())
    contract_summary = json.loads((CONTRACT / "comparison_summary.json").read_text())

    fig, axes = plt.subplots(3, 2, figsize=(15, 13), facecolor="white")
    ax_xy, ax_sep, ax_fwd, ax_lat, ax_yaw, ax_metrics = axes.flat

    series = [
        (real, "Real EKF path", REAL_COLOR, "--", 1.8),
        (exact, "Exact replay", EXACT_COLOR, "-", 2.0),
        (single, "Single-owner hypothesis", SINGLE_COLOR, "-", 2.1),
        (contract, "Verified single-owner rerun", CONTRACT_COLOR, "-", 2.2),
    ]
    for rows, label, color, linestyle, width in series:
        ax_xy.plot(rows[:, 1], rows[:, 2], label=label, color=color, linestyle=linestyle, linewidth=width)
        ax_fwd.plot(rows[:, 0], rows[:, 1], color=color, linestyle=linestyle, linewidth=width)
        ax_lat.plot(rows[:, 0], rows[:, 2], color=color, linestyle=linestyle, linewidth=width)
        ax_yaw.plot(rows[:, 0], np.degrees(rows[:, 4]), color=color, linestyle=linestyle, linewidth=width)

    ax_xy.scatter(0, 0, color=REAL_COLOR, s=45, zorder=4)
    ax_xy.axis("equal")
    ax_xy.set_xlabel("forward from start [m]")
    ax_xy.set_ylabel("lateral from start [m]")
    ax_xy.legend(frameon=False, ncol=3, loc="upper left")
    style(ax_xy, "XY trajectory")

    for rows, label, color in [
        (exact, "Exact replay", EXACT_COLOR),
        (single, "Single-owner hypothesis", SINGLE_COLOR),
        (contract, "Verified single-owner rerun", CONTRACT_COLOR),
    ]:
        t, separation = xy_separation(real, rows)
        ax_sep.plot(t, separation, label=label, color=color, linewidth=2.0)
    ax_sep.set_xlabel("elapsed simulation time [s]")
    ax_sep.set_ylabel("separation from Real [m]")
    ax_sep.legend(frameon=False)
    style(ax_sep, "XY separation from Real")

    for ax, title, ylabel in [
        (ax_fwd, "Forward displacement", "forward [m]"),
        (ax_lat, "Lateral displacement", "lateral [m]"),
        (ax_yaw, "Relative yaw", "yaw - yaw0 [deg]"),
    ]:
        ax.set_xlabel("elapsed simulation time [s]")
        ax.set_ylabel(ylabel)
        style(ax, title)

    labels = ["XY RMSE", "Final XY error", "XY path length"]
    exact_values = [exact_summary["xy_rmse_m"], exact_summary["xy_final_error_m"], exact_summary["sim_xy_path_length_m"]]
    single_values = [single_summary["xy_rmse_m"], single_summary["xy_final_error_m"], single_summary["sim_xy_path_length_m"]]
    contract_values = [contract_summary["xy_rmse_m"], contract_summary["xy_final_error_m"], contract_summary["sim_xy_path_length_m"]]
    x = np.arange(len(labels))
    width = 0.25
    bars_a = ax_metrics.bar(x - width, exact_values, width, color=EXACT_COLOR, label="Exact replay")
    bars_b = ax_metrics.bar(x, single_values, width, color=SINGLE_COLOR, label="Single-owner hypothesis")
    bars_c = ax_metrics.bar(x + width, contract_values, width, color=CONTRACT_COLOR, label="Verified rerun")
    ax_metrics.bar_label(bars_a, fmt="%.2f", padding=3, fontsize=9)
    ax_metrics.bar_label(bars_b, fmt="%.2f", padding=3, fontsize=9)
    ax_metrics.bar_label(bars_c, fmt="%.2f", padding=3, fontsize=9)
    ax_metrics.axhline(exact_summary["real_xy_path_length_m"], color=REAL_COLOR, linestyle="--", linewidth=1.2, label="Real path length 19.12 m")
    ax_metrics.set_xticks(x, labels)
    ax_metrics.set_ylabel("metres")
    ax_metrics.legend(frameon=False, fontsize=9)
    style(ax_metrics, "Outcome metrics")

    fig.suptitle("RC replay variants — bag seconds 28 to 100", x=0.07, y=0.975, ha="left", fontsize=18, fontweight="bold")
    fig.text(
        0.07,
        0.943,
        "Same initial pose, depth, velocity, ALT_HOLD, and simulation-time axis; verified rerun had exactly one RC publisher.",
        ha="left",
        color="#5F6673",
        fontsize=10.5,
    )
    fig.text(
        0.07,
        0.02,
        "Real is filtered odometry (not ground truth). Single-owner is a counterfactual filter, not a reconstruction of applied real RC input.",
        ha="left",
        color="#5F6673",
        fontsize=9.5,
    )
    fig.subplots_adjust(left=0.08, right=0.98, top=0.91, bottom=0.07, hspace=0.38, wspace=0.30)
    output = CONTRACT / "real_vs_replay_contract_variants.png"
    fig.savefig(output, dpi=170, facecolor="white")
    print(output)


if __name__ == "__main__":
    main()
