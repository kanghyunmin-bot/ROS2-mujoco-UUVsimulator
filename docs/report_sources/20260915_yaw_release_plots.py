# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Source-backed path and yaw-release figures with fixed time/pose alignment."""

from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "outputs/yaw-release-real2sim-20260915"
fig_path, path_axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
fig_release, release_axes = plt.subplots(1, 2, figsize=(12, 4.4), layout="constrained")
for j, bag in enumerate(["primary", "auxiliary"]):
    cfg = json.loads((OUT / f"{bag}_commands.json").read_text())
    src = ROOT / cfg["source"]
    arm = next(x[0] for x in cfg["states"] if x[2])
    data = np.load(src / "numeric.npz")
    p = json.loads((src / "profile.json").read_text())
    t0 = min(x["start"] for x in p.values())
    imu = data["__mavros__imu__data"]
    ti = imu[:, 0] - t0
    real_rate = np.rad2deg(imu[:, 7])
    start, end = (20, 70) if bag == "primary" else (12, 110)
    release = 68.43714213371277 if bag == "primary" else 66.43121385574341
    of = (
        ROOT / "outputs/real-bag-tuning-20260911/odometry.npz"
        if bag == "primary"
        else src / "odometry.npz"
    )
    odom = np.load(of)["__mavros__local_position__odom"]

    def aligned(t, pos, q):
        mask = (t >= cfg["begin"]) & (t < arm - 0.3)
        yaw = np.unwrap(Rotation.from_quat(q).as_euler("xyz")[:, 2])
        angle = np.median(yaw[mask])
        return Rotation.from_euler("z", -angle).apply(
            pos - np.median(pos[mask], axis=0)
        )

    real_pos = aligned(odom[:, 0], odom[:, 2:5], odom[:, 5:9])
    mask = (odom[:, 0] >= start) & (odom[:, 0] <= end)
    ax = path_axes[j]
    ax.plot(*real_pos[mask, :2].T, color="#192b3b", lw=2, label="Real FCU estimate")
    mr = (ti >= release - 0.8) & (ti <= release + 4)
    ar = release_axes[j]
    ar.plot(ti[mr] - release, real_rate[mr], color="#192b3b", lw=2, label="Real IMU")
    for label, path, color in [
        (
            "Clock corrected",
            ROOT / f"outputs/yaw-timing-audit-20260915/{bag}_round",
            "#df7549",
        ),
        ("Response calibrated", OUT / f"{bag}_validated", "#008b7c"),
    ]:
        r = json.loads((path / "replay.json").read_text())
        assert r["complete"]
        a = np.array(r["tracks"]["/mujoco/ground_truth/pose"])
        t = a[:, 1] - r["origin_sim_time"] + cfg["begin"]
        pos = aligned(t, a[:, 2:5], a[:, 5:9])
        m = (t >= start) & (t <= end)
        ax.plot(*pos[m, :2].T, color=color, lw=1.8, label=label)
        f = np.genfromtxt(path / "forces.csv", delimiter=",", names=True)
        tf = f["sim_time"] - r["origin_sim_time"] + cfg["begin"]
        mf = (tf >= release - 0.8) & (tf <= release + 4)
        ar.plot(
            tf[mf] - release,
            np.rad2deg(f["ang_vel_body_z"][mf]),
            color=color,
            lw=1.8,
            label=label,
        )
    ax.set(
        title=(
            "Primary: calibration bag"
            if bag == "primary"
            else "Auxiliary: held-out bag"
        ),
        xlabel="Initial-forward x [m]",
        ylabel="Initial-left y [m]",
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.2)
    ax.legend(fontsize=8)
    ar.set(
        title=f"{bag.capitalize()}: final yaw release at {release:.3f} s",
        xlabel="Time after stick release [s]",
        ylabel="Yaw rate [deg/s]",
    )
    ar.axvline(0, color="#888", ls="--", lw=0.8)
    ar.axhline(0, color="#888", lw=0.8)
    ar.grid(alpha=0.2)
    ar.legend(fontsize=8)
fig_path.suptitle(
    "Real2Sim path comparison | initial pose alignment only\nFCU position is an estimate, not external ground truth",
    fontsize=13,
)
fig_release.suptitle(
    "Match measured rebound instead of removing it | same controller and PID\nPositive yaw rate after release is the brief reverse rotation",
    fontsize=13,
)
fig_path.savefig(OUT / "paths.png", dpi=160)
fig_release.savefig(OUT / "release.png", dpi=160)
plt.close("all")
