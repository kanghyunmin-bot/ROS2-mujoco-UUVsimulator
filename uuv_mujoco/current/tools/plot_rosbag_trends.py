"""Plot an offline trend report (optional matplotlib; no ROS or vehicle I/O)."""

import argparse
import json
from itertools import cycle
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report_dir", type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.report_dir / "report.json").read_text())
    data = np.load(args.report_dir / "aligned_signals.npz", allow_pickle=False)
    t = data["time_s"]
    runs = list(report["runs"])
    colors = dict(zip(runs, cycle(["#b9c3d1", "#167d9a", "#cb7a36", "#9674b8"])))
    fig, ax = plt.subplots(2, 2, figsize=(14, 10), layout="constrained")
    for topic, style, label in (
        ("__mavros__local_position__odom", "-", "Real FCU estimate"),
        ("__odometry__filtered", "--", "Real ROS EKF estimate"),
    ):
        p = data["real_path_" + topic]
        selected = (t >= 20) & (t < 70)
        ax[0, 0].plot(
            p[selected, 0], p[selected, 1], style, color="#182b3d", label=label
        )
    for name in runs:
        p = data[name + "_path_selected"]
        ax[0, 0].plot(
            p[selected, 0], p[selected, 1], color=colors[name], label="SIM " + name
        )
    ax[0, 0].set(
        xlabel="Initial-forward x [m]",
        ylabel="Initial-left y [m]",
        title="Actual MuJoCo rollout vs estimated real path",
    )
    ax[0, 0].set_aspect("equal", adjustable="datalim")
    for axis, signal, label in (
        (ax[0, 1], "surge", "Forward speed [m/s]"),
        (ax[1, 0], "yaw_rate", "Yaw rate [rad/s]"),
        (ax[1, 1], "relative_depth", "Depth change, down positive [m]"),
    ):
        axis.plot(
            t,
            data["real_" + signal],
            color="#182b3d",
            linewidth=1.5,
            label="Real sensor output",
        )
        for name in runs:
            axis.plot(
                t,
                data[name + "_" + signal + "_selected"],
                color=colors[name],
                label="SIM " + name,
                alpha=0.9,
            )
        for window, color in (
            ([20, 40], "#dcebf2"),
            ([40.5, 55], "#eae4d4"),
            ([55.5, 70], "#e3eedf"),
        ):
            axis.axvspan(*window, color=color, alpha=0.35, zorder=-1)
        axis.set(xlim=(18, 71), xlabel="Bag elapsed / receipt time [s]", ylabel=label)
    for axis in ax.flat:
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    fig.suptitle(
        "ROSBAG / SIM trend comparison — original sensor variation retained\nPhase fit 20–40 s | validation 40.5–55 s | report-only test 55.5–70 s",
        fontsize=14,
    )
    fig.savefig(args.report_dir / "trends.png", dpi=160)
    fig.savefig(args.report_dir / "trends.svg")
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.7), layout="constrained")
    for axis, sensor, unit in zip(
        axes, ["dvl", "gyro", "pressure"], ["m/s", "rad/s", "Pa"]
    ):
        n = report["noise"][sensor]
        train, test = np.array(n["fit"]["std"]), np.array(n["evaluation"]["std"])
        quantiles = np.array(n["sample_std_ensemble_p05_p50_p95"])
        x = np.arange(len(train))
        axis.bar(x - 0.2, train, 0.2, color="#167d9a", label="Real fit 90–115 s")
        axis.bar(x, test, 0.2, color="#182b3d", label="Real evaluation 115–140 s")
        axis.errorbar(
            x + 0.2,
            quantiles[1],
            yerr=[quantiles[1] - quantiles[0], quantiles[2] - quantiles[1]],
            fmt="o",
            color="#cb7a36",
            capsize=4,
            label="128 synthetic seeds / 5–95%",
        )
        axis.set(
            xticks=x,
            xticklabels=list("xyz") if len(x) == 3 else ["pressure"],
            ylabel="Output standard deviation [" + unit + "]",
            title=sensor.upper(),
        )
        axis.grid(axis="y", alpha=0.2)
    axes[0].legend(fontsize=7)
    fig.suptitle(
        "Low-motion observation envelopes — not raw hardware calibration\nMismatch in evaluation is preserved; no seed selection or noise added to RMSE",
        fontsize=13,
    )
    fig.savefig(args.report_dir / "noise.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
