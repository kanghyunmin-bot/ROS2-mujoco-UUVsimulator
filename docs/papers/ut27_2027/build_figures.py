# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Redraw UT27 figures from existing replay arrays without running simulation."""

import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Rectangle
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)
plt.rcParams.update(
    {
        "font.family": "Liberation Serif",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
    }
)
COLORS = ["#202020", "#ce752a", "#1763a6"]
STYLES = ["-", "--", "-."]
LABELS = ["Real record", "Calibrated / original clock", "Calibrated / corrected clock"]
clock = json.loads(
    (ROOT / "docs/assets/yaw-timing-20260915/full_comparison.json").read_text()
)
nominal = json.loads(
    (ROOT / "docs/assets/real2sim-application-20260915/comparison.json").read_text()
)


def aligned(t, pos, q, begin, arm):
    m = (t >= begin) & (t < arm - 0.3)
    assert m.sum() >= 3
    yaw = np.unwrap(Rotation.from_quat(q).as_euler("xyz")[:, 2])
    y0 = float(np.median(yaw[m]))
    return Rotation.from_euler("z", -y0).apply(
        pos - np.median(pos[m], axis=0)
    ), yaw - y0


def support(t, grid, gap):
    ix = np.searchsorted(t, grid, side="right")
    valid = (ix > 0) & (ix < len(t))
    ix = np.clip(ix, 1, len(t) - 1)
    return valid & (t[ix] - t[ix - 1] <= gap)


def interp(t, v, grid):
    return (
        np.interp(grid, t, v)
        if v.ndim == 1
        else np.column_stack([np.interp(grid, t, x) for x in v.T])
    )


fig, ax = plt.subplots(2, 2, figsize=(7, 3.6), gridspec_kw={"height_ratios": [1.1, 1]})
fig.subplots_adjust(
    left=0.09, right=0.985, bottom=0.12, top=0.86, hspace=0.55, wspace=0.32
)
audit = {
    "method": clock["method"],
    "source_files": {},
    "checks": [],
    "new_simulation_runs": 0,
}
rows = []
xy_sets = []
for j, bag in enumerate(["primary", "auxiliary"]):
    cfg = json.loads(
        (ROOT / f"outputs/yaw-timing-audit-20260915/{bag}_commands.json").read_text()
    )
    src = ROOT / cfg["source"]
    arr = np.load(src / "numeric.npz")
    profile = json.loads((src / "profile.json").read_text())
    t0 = min(v["start"] for v in profile.values())
    arm = next(v[0] for v in cfg["states"] if v[2])
    start, end = clock["bags"][bag]["evaluation_s"]
    grid = np.arange(start, end, 0.1)
    of = (
        ROOT / "outputs/real-bag-tuning-20260911/odometry.npz"
        if bag == "primary"
        else src / "odometry.npz"
    )
    od = np.load(of)["__mavros__local_position__odom"]
    rp, _ = aligned(od[:, 0], od[:, 2:5], od[:, 5:9], cfg["begin"], arm)
    imu = arr["__mavros__imu__data"]
    ti = imu[:, 0] - t0
    ry = np.unwrap(Rotation.from_quat(imu[:, 1:5]).as_euler("xyz")[:, 2])
    ry -= np.median(ry[(ti >= cfg["begin"]) & (ti < arm - 0.3)])
    realpos = interp(od[:, 0], rp, grid)
    realyaw = np.rad2deg(interp(ti, ry, grid))
    plot_realpos, plot_realyaw = realpos.copy(), realyaw.copy()
    plot_realpos[~support(od[:, 0], grid, 0.6)] = np.nan
    plot_realyaw[~support(ti, grid, 0.2)] = np.nan
    curves = [(plot_realpos, plot_realyaw)]
    for case in ["original_clock", "rounded_clock"]:
        path = ROOT / (
            f"outputs/real2sim-application-20260915/{bag}_effective"
            if case == "original_clock"
            else f"outputs/yaw-timing-audit-20260915/{bag}_round"
        )
        run = json.loads((path / "replay.json").read_text())
        assert run["complete"]
        a = np.asarray(run["tracks"]["/mujoco/ground_truth/pose"])
        t = a[:, 1] - run["origin_sim_time"] + cfg["begin"]
        pos, yaw = aligned(t, a[:, 2:5], a[:, 5:9], cfg["begin"], arm)
        pg, yg = interp(t, pos, grid), np.rad2deg(interp(t, yaw, grid))
        pv = support(t, grid, 0.3) & support(od[:, 0], grid, 0.6)
        yv = support(t, grid, 0.3) & support(ti, grid, 0.2)
        xyr = float(
            np.sqrt(np.mean(np.sum((pg[pv, :2] - realpos[pv, :2]) ** 2, axis=1)))
        )
        yr = float(np.sqrt(np.mean((yg[yv] - realyaw[yv]) ** 2)))
        expected = clock["bags"][bag]["cases"][case]
        assert abs(xyr - expected["xy_rmse_vs_fcu_estimate_m"]) < 1e-10
        assert abs(yr - expected["yaw_change_rmse_deg"]) < 1e-10
        audit["checks"].append(
            {
                "bag": bag,
                "case": case,
                "xy_rmse_m": xyr,
                "yaw_rmse_deg": yr,
                "matches_recorded_summary": True,
                "grid_points": len(grid),
                "valid_xy_points": int(pv.sum()),
                "valid_yaw_points": int(yv.sum()),
                "unsupported_plot_points_masked": True,
            }
        )
        pg[~pv], yg[~yv] = np.nan, np.nan
        curves.append((pg, yg))
        for f in [
            path / "replay.json",
            path / "profiles.json",
            src / "numeric.npz",
            of,
        ]:
            audit["source_files"][str(f.relative_to(ROOT))] = hashlib.sha256(
                f.read_bytes()
            ).hexdigest()
    for k, (pos, yaw) in enumerate(curves):
        ax[0, j].plot(
            pos[:, 0], pos[:, 1], c=COLORS[k], ls=STYLES[k], lw=1.2, label=LABELS[k]
        )
        ax[1, j].plot(grid, yaw, c=COLORS[k], ls=STYLES[k], lw=1.1)
        xy_sets.append(pos[:, :2])
    ax[0, j].set_title(f"Run {'A' if j == 0 else 'B'} ({start:g}-{end:g} s)", pad=3)
    ax[0, j].set(xlabel="Initial-forward x (m)", ylabel="Initial-left y (m)")
    ax[1, j].set(
        xlabel="Elapsed bag time (s)",
        ylabel="Yaw change (deg)",
        ylim=(-400, 15),
        yticks=[-360, -180, 0],
        xlim=(start, end),
    )
    ax[1, j].set_xticks([start, (start + end) / 2, end])
    for i in [0, 1]:
        ax[i, j].grid(True, color="#dedede", lw=0.5)
    cases = [
        ("Nominal", nominal["bags"][bag]["cases"]["nominal"]),
        ("Calibrated", clock["bags"][bag]["cases"]["original_clock"]),
        ("+ timing", clock["bags"][bag]["cases"]["rounded_clock"]),
    ]
    for name, m in cases:
        rows.append(
            {
                "run": "A" if j == 0 else "B",
                "model": name,
                "surge_rmse_mps": m["surge_rmse_mps"],
                "xy_rmse_m": m["xy_rmse_vs_fcu_estimate_m"],
                "yaw_rmse_deg": m["yaw_change_rmse_deg"],
                "relative_depth_rmse_m": m["relative_depth_rmse_m"],
            }
        )
allxy = np.vstack(xy_sets)
lo, hi = np.nanmin(allxy, axis=0), np.nanmax(allxy, axis=0)
for a in ax[0]:
    a.set_xlim(lo[0] - 0.15, hi[0] + 0.15)
    a.set_ylim(lo[1] - 0.15, hi[1] + 0.15)
    a.set_aspect("equal", adjustable="box")
fig.legend(
    *ax[0, 0].get_legend_handles_labels(),
    loc="upper center",
    ncol=3,
    frameon=False,
    bbox_to_anchor=(0.5, 1.015),
    handlelength=2.6,
    columnspacing=1.7,
)
for ext in ["png", "pdf", "svg"]:
    fig.savefig(FIG / f"fig2_clock_comparison.{ext}", dpi=400)
plt.close(fig)
with (HERE / "metrics.csv").open("w") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

# Topology, not a measured time series. Rates are the replay settings in source logs.
fig, a = plt.subplots(figsize=(7, 1.55))
fig.subplots_adjust(left=0.005, right=0.995, bottom=0.015, top=0.99)
a.set(xlim=(0, 7), ylim=(0, 1.55))
a.axis("off")


def box(x, y, w, h, text):
    a.add_patch(Rectangle((x, y), w, h, facecolor="white", edgecolor="#444444", lw=0.8))
    a.text(
        x + w / 2,
        y + h / 2,
        text,
        ha="center",
        va="center",
        fontsize=8.5,
        linespacing=1.2,
    )


def arrow(x1, y1, x2, y2, label=None, dy=0.1):
    a.add_patch(
        FancyArrowPatch(
            (x1, y1),
            (x2, y2),
            arrowstyle="-|>",
            mutation_scale=8,
            color="#303030",
            lw=0.9,
        )
    )
    if label:
        a.text(
            (x1 + x2) / 2,
            (y1 + y2) / 2 + dy,
            label,
            ha="center",
            va="center",
            fontsize=8,
        )


box(0.04, 0.79, 1.33, 0.52, "Recorded commands\nRC / mode / ARM")
box(1.75, 0.79, 1.3, 0.52, "ROS 2\nMAVROS")
box(3.44, 0.79, 1.3, 0.52, "ArduSub SITL\n400 Hz controller")
box(5.25, 0.79, 1.65, 0.52, "MuJoCo vehicle\n800 Hz integration")
arrow(1.37, 1.05, 1.75, 1.05)
arrow(3.05, 1.05, 3.44, 1.05)
arrow(4.74, 1.05, 5.25, 1.05, "PWM", 0.19)
a.plot([6.06, 6.06, 4.08], [0.79, 0.5, 0.5], color="#303030", lw=0.9)
arrow(4.08, 0.5, 4.08, 0.79)
a.text(5.09, 0.35, "JSON state + internal IMU / 400 Hz", ha="center", fontsize=8)
box(0.04, 0.02, 2.98, 0.43, "Real DVL / IMU / FCU estimate")
box(3.44, 0.02, 3.46, 0.22, "Offline errors and trajectory comparison")
arrow(3.02, 0.20, 3.44, 0.13)
arrow(6.73, 0.79, 6.73, 0.24)
a.text(6.52, 0.43, "truth", ha="right", fontsize=8)
for ext in ["png", "pdf", "svg"]:
    fig.savefig(FIG / f"fig1_replay_architecture.{ext}", dpi=400)
plt.close(fig)
(HERE / "figure_verification.json").write_text(json.dumps(audit, indent=2) + "\n")
print(
    json.dumps(
        {
            "figures": 2,
            "verified_comparisons": len(audit["checks"]),
            "new_simulation_runs": 0,
        }
    )
)
