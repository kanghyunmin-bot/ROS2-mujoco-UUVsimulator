# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Fixed-time comparisons, using primary releases for candidate selection."""

from pathlib import Path
import json
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "outputs/yaw-release-real2sim-20260915"


def rmse(error: np.ndarray | list[float]) -> float:
    """Return root mean square error in the input units."""
    return float(np.sqrt(np.mean(np.asarray(error) ** 2)))


report = {
    "selection": "Primary bag release transients; auxiliary bag reserved for validation after selecting a candidate. Receipt time only; no fitted delay, phase, gain or initial pose beyond pre-arm median.",
    "bags": {},
}
for bag in ["primary", "auxiliary"]:
    cfg = json.loads((OUT / f"{bag}_commands.json").read_text())
    src = ROOT / cfg["source"]
    p = json.loads((src / "profile.json").read_text())
    t0 = min(x["start"] for x in p.values())
    data = np.load(src / "numeric.npz")
    imu = data["__mavros__imu__data"]
    ti = imu[:, 0] - t0
    real_rate = np.rad2deg(imu[:, 7])
    arm = next(x[0] for x in cfg["states"] if x[2])
    initial = (ti >= cfg["begin"]) & (ti < arm - 0.3)
    real_yaw = np.rad2deg(
        np.unwrap(Rotation.from_quat(imu[:, 1:5]).as_euler("xyz")[:, 2])
    )
    real_yaw -= np.median(real_yaw[initial])
    rc = np.array([[t, *ch[:8]] for t, ch in cfg["rc"]])
    near = abs(rc[:, 4] - 1500) < 5
    starts = np.flatnonzero(near[1:] & ~near[:-1]) + 1
    events = []
    for i in starts:
        t = rc[i, 0]
        j = np.searchsorted(rc[:, 0], t + 2.5)
        before = (ti > t - 0.3) & (ti < t)
        if t < 15 or t + 2.5 > cfg["end"] or not np.all(near[i:j]) or not before.any():
            continue
        speed = np.median(real_rate[before])
        direction = np.sign(speed)
        if abs(speed) < 10:
            continue
        mask = (ti >= t) & (ti < t + 2.5)
        events.append(
            {
                "release_s": float(t),
                "direction": float(direction),
                "real_peak_deg_s": float(max(0, np.max(-direction * real_rate[mask]))),
            }
        )
    start, end = (20, 70) if bag == "primary" else (12, 110)
    grid = np.arange(start, end, 0.1)
    dvl = data["__dvl__data"]
    td = dvl[:, 0] - t0
    idx = np.clip(np.searchsorted(td, grid), 1, len(td) - 1)
    valid = (
        (td[idx] - td[idx - 1] <= 0.3) & (dvl[idx, 5] > 0.5) & (dvl[idx - 1, 5] > 0.5)
    )
    report["bags"][bag] = {"events": events, "cases": {}}
    paths = [("baseline", ROOT / f"outputs/yaw-timing-audit-20260915/{bag}_round")] + [
        (p.name[len(bag) + 1 :], p) for p in sorted(OUT.glob(bag + "_*")) if p.is_dir()
    ]
    for name, path in paths:
        if not (path / "replay.json").exists():
            continue
        r = json.loads((path / "replay.json").read_text())
        assert r["complete"]
        f = np.genfromtxt(path / "forces.csv", delimiter=",", names=True)
        tf = f["sim_time"] - r["origin_sim_time"] + cfg["begin"]
        rate = np.rad2deg(f["ang_vel_body_z"])
        a = np.array(r["tracks"]["/mujoco/ground_truth/pose"])
        t = a[:, 1] - r["origin_sim_time"] + cfg["begin"]
        yaw = np.rad2deg(np.unwrap(Rotation.from_quat(a[:, 5:9]).as_euler("xyz")[:, 2]))
        yaw -= np.median(yaw[(t >= cfg["begin"]) & (t < arm - 0.3)])
        releases = []
        errors = []
        for e in events:
            release = e["release_s"]
            mt = (ti >= release) & (ti < release + 2.5)
            mf = (tf >= release) & (tf < release + 2.5)
            err = np.interp(ti[mt], tf, rate) - real_rate[mt]
            errors.extend(err)
            peak = float(max(0, np.max(-e["direction"] * rate[mf])))
            peak_idx = np.flatnonzero(mf)[np.argmax(-e["direction"] * rate[mf])]
            releases.append(
                {
                    "release_s": release,
                    "peak_deg_s": peak,
                    "real_peak_deg_s": e["real_peak_deg_s"],
                    "peak_error_deg_s": peak - e["real_peak_deg_s"],
                    "rate_rmse_deg_s": rmse(err),
                    "peak_delay_s": float(tf[peak_idx] - release),
                }
            )
        bodyinertia = None
        result = {
            "release_rmse_deg_s": rmse(errors),
            "mean_abs_peak_error_deg_s": float(
                np.mean([abs(e["peak_error_deg_s"]) for e in releases])
            ),
            "yaw_rmse_deg": rmse(
                np.interp(grid, t, yaw) - np.interp(grid, ti, real_yaw)
            ),
            "surge_rmse_mps": rmse(
                (
                    np.interp(grid, tf, f["lin_vel_body_x"])
                    - np.interp(grid, td, dvl[:, 1])
                )[valid]
            ),
            "releases": releases,
            "profile_sha256": hashlib.sha256(
                (path / "profiles.json").read_bytes()
            ).hexdigest(),
        }
        report["bags"][bag]["cases"][name] = result
        print(
            bag,
            name,
            {
                k: v
                for k, v in result.items()
                if k not in ["releases", "profile_sha256"]
            },
            flush=True,
        )
(OUT / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
