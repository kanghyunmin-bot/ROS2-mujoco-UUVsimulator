"""Compare real bag signals with a fresh research-pool SITL replay, without fitting."""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--evidence_root", type=Path, required=True)
parser.add_argument("--output_dir", type=Path, required=True)
parser.add_argument(
    "--run_dir", type=Path, default=ROOT / "outputs/research-pool-real2sim-20260916"
)
parser.add_argument("--bags", nargs="+", default=["primary", "auxiliary"])
args = parser.parse_args()
RUNS = args.run_dir
args.output_dir.mkdir(parents=True, exist_ok=True)


def interp(t, values, grid, max_gap):
    """Interpolate only within supported time spans; retain gaps as NaN."""
    t = np.asarray(t)
    assert len(t) > 1 and np.all(np.diff(t) >= 0)
    keep = np.r_[np.diff(t) > 0, True]
    t, values = t[keep], np.asarray(values)[keep]
    i = np.searchsorted(t, grid, side="right")
    valid = (i > 0) & (i < len(t))
    i = np.clip(i, 1, len(t) - 1)
    valid &= t[i] - t[i - 1] <= max_gap
    if values.ndim == 1:
        result = np.interp(grid, t, values)
    else:
        result = np.column_stack([np.interp(grid, t, col) for col in values.T])
    result[~valid] = np.nan
    return result


def initial_mask(t, begin, arm):
    mask = (t >= begin) & (t < arm - 0.3)
    assert mask.sum() >= 3, "Insufficient disarmed initialization samples"
    return mask


def relative_yaw(t, quaternion, begin, arm):
    yaw = np.unwrap(Rotation.from_quat(quaternion).as_euler("xyz")[:, 2])
    return np.rad2deg(yaw - np.median(yaw[initial_mask(t, begin, arm)]))


def relative_position(t, xyz, quaternion, begin, arm):
    initial = initial_mask(t, begin, arm)
    yaw = np.unwrap(Rotation.from_quat(quaternion).as_euler("xyz")[:, 2])
    return Rotation.from_euler("z", -np.median(yaw[initial])).apply(
        xyz - np.median(xyz[initial], axis=0)
    )


def metric(real, simulated):
    valid = np.isfinite(real) & np.isfinite(simulated)
    if real.ndim == 2:
        valid = np.all(valid, axis=1)
    error = simulated[valid] - real[valid]
    squared = np.sum(error**2, axis=1) if error.ndim == 2 else error**2
    return {
        "rmse": float(np.sqrt(np.mean(squared))) if len(error) else None,
        "valid_grid_count": int(valid.sum()),
    }


def contact_audit(track, t, start, end):
    """Check stored vehicle poses against the unchanged pool; not continuous contacts."""
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "uuv_mujoco/current/scenes/research_pool_slam_scene.xml")
    )
    data = mujoco.MjData(model)
    base = model.body("base_link").id
    address = int(model.jnt_qposadr[int(model.body_jntadr[base])])
    bodies = {base}
    for body in range(base + 1, model.nbody):
        if int(model.body_parentid[body]) in bodies:
            bodies.add(body)
    geoms = [
        g
        for g in range(model.ngeom)
        if int(model.geom_bodyid[g]) in bodies
        and (model.geom_contype[g] or model.geom_conaffinity[g])
    ]
    contacts, minimum_margin = {}, float("inf")
    rows = track[(t >= start) & (t < end)]
    for row in rows:
        data.qpos[address : address + 3] = row[2:5]
        data.qpos[address + 3 : address + 7] = row[[8, 5, 6, 7]]
        mujoco.mj_forward(model, data)
        margin = (
            np.array([5.0, 2.5])
            - abs(data.geom_xpos[geoms, :2])
            - model.geom_rbound[geoms, None]
        )
        minimum_margin = min(minimum_margin, float(margin.min()))
        for contact in data.contact:
            a, b = int(contact.geom1), int(contact.geom2)
            if (int(model.geom_bodyid[a]) in bodies) != (
                int(model.geom_bodyid[b]) in bodies
            ):
                key = f"{model.geom(a).name}/{model.geom(b).name}"
                contacts[key] = contacts.get(key, 0) + 1
    return {
        "sampled_poses": len(rows),
        "contacts_at_sampled_poses": contacts,
        "conservative_wall_margin_m": minimum_margin,
        "note": "Vehicle poses checked in original scene; other bodies at initial positions. Not a continuous contact log.",
    }


plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.labelcolor": "#253044",
        "text.color": "#253044",
        "savefig.facecolor": "white",
    }
)
report = {
    "method": "Fresh ArduSub + MuJoCo + ROS replay in the unchanged 10 x 5 x 5 m research pool. Real receipt times; simulated ROS receipt times. Initial disarmed position/yaw/z alignment only; no fitted phase shift, time warp, scale or physics retuning.",
    "scope": "Simulation only. 20 V assumption; no measured ESC rail or tether model. Cameras disabled during dynamics replay. Real FCU path is an estimate, not external truth.",
    "bags": {},
}
for bag in args.bags:
    config = json.loads((RUNS / f"{bag}_commands.json").read_text())
    folder = args.evidence_root / config["source"]
    source = np.load(folder / "numeric.npz")
    profile = json.loads((folder / "profile.json").read_text())
    t0 = min(item["start"] for item in profile.values())
    arm = next(row[0] for row in config["states"] if row[2])
    start, end = (20.0, 70.0) if bag == "primary" else (12.0, 110.0)
    grid = np.arange(start, end, 0.1)
    odom_file = (
        (args.evidence_root / "outputs/real-bag-tuning-20260911/odometry.npz")
        if bag == "primary"
        else folder / "odometry.npz"
    )
    odom = np.load(odom_file)["__mavros__local_position__odom"]
    dvl, imu, depth = (
        source[key] for key in ["__dvl__data", "__mavros__imu__data", "__depth__pose"]
    )
    td, ti, tz = (x[:, 0] - t0 for x in [dvl, imu, depth])
    real_dvl = dvl[:, 1].copy()
    real_dvl[dvl[:, 5] <= 0.5] = np.nan
    real_yaw = relative_yaw(ti, imu[:, 1:5], config["begin"], arm)
    real_z = depth[:, 1] - np.median(depth[initial_mask(tz, config["begin"], arm), 1])
    real_path = relative_position(
        odom[:, 0], odom[:, 2:5], odom[:, 5:9], config["begin"], arm
    )
    path = RUNS / f"{bag}_pool"
    replay = json.loads((path / "replay.json").read_text())
    assert replay["complete"], (
        "Incomplete replay cannot be reported as a completed comparison"
    )
    shift = config["begin"] - replay["origin_sim_time"]
    track = np.asarray(replay["tracks"]["/mujoco/ground_truth/pose"])
    st = track[:, 1] + shift
    sd, si, sz = (
        np.asarray(replay["sensors"][name]) for name in ["dvl", "imu", "depth"]
    )
    sdt, sit, szt = (x[:, 0] + shift for x in [sd, si, sz])
    sim_dvl = sd[:, 2].copy()
    sim_dvl[sd[:, 6] <= 0.5] = np.nan
    sim_yaw = relative_yaw(sit, si[:, 2:6], config["begin"], arm)
    sim_z = sz[:, 2] - np.median(sz[initial_mask(szt, config["begin"], arm), 2])
    sim_path = relative_position(st, track[:, 2:5], track[:, 5:9], config["begin"], arm)
    sim_pitch = Rotation.from_quat(si[:, 2:6]).as_euler("xyz")[:, 1]
    real_pitch = Rotation.from_quat(imu[:, 1:5]).as_euler("xyz")[:, 1]
    series = [
        ("surge", "Forward velocity", "m/s", td, real_dvl, sdt, sim_dvl, 0.3),
        ("yaw", "Yaw change", "deg", ti, real_yaw, sit, sim_yaw, 0.25),
        (
            "yaw_rate",
            "Yaw angular velocity",
            "deg/s",
            ti,
            np.rad2deg(imu[:, 7]),
            sit,
            np.rad2deg(si[:, 8]),
            0.25,
        ),
        (
            "relative_z",
            "Relative vertical position (+up)",
            "m",
            tz,
            real_z,
            szt,
            sim_z,
            0.8,
        ),
        (
            "pitch",
            "Pitch",
            "deg",
            ti,
            np.rad2deg(real_pitch),
            sit,
            np.rad2deg(sim_pitch),
            0.25,
        ),
    ]
    fig, axes = plt.subplots(3, 2, figsize=(13.5, 10.5), layout="constrained")
    metrics, columns, names = {}, [grid], ["bag_elapsed_s"]
    for ax, (key, title, unit, rt, rv, sim_t, sim_v, gap) in zip(axes.flat, series):
        real = interp(rt, rv, grid, gap)
        simulated = interp(sim_t, sim_v, grid, gap)
        metrics[key] = {**metric(real, simulated), "unit": unit}
        columns.extend([real, simulated])
        names.extend([f"real_{key}", f"sim_{key}"])
        ax.plot(grid, real, color="#283343", linewidth=1.8, label="Real bag")
        ax.plot(
            grid,
            simulated,
            color="#1779b5",
            linewidth=1.55,
            linestyle="--",
            label="Research pool + Real2Sim",
        )
        value = metrics[key]["rmse"]
        subtitle = (
            f"RMSE {value:.4f} {unit}"
            if value is not None
            else "No valid shared samples"
        )
        ax.set(
            title=f"{title}  |  {subtitle}",
            xlabel="Bag elapsed [s]",
            ylabel=f"{title} [{unit}]",
        )
        ax.grid(alpha=0.18)
        ax.legend(fontsize=8, loc="best")
    real_xy = interp(odom[:, 0], real_path[:, :2], grid, 0.6)
    sim_xy = interp(st, sim_path[:, :2], grid, 0.3)
    metrics["xy_vs_fcu_estimate"] = {**metric(real_xy, sim_xy), "unit": "m"}
    ax = axes.flat[5]
    ax.plot(*real_xy.T, color="#283343", linewidth=1.8, label="Real FCU estimate")
    ax.plot(
        *sim_xy.T,
        color="#1779b5",
        linewidth=1.55,
        linestyle="--",
        label="Simulation truth",
    )
    ax.set(
        title=f"Path vs FCU estimate  |  RMSE {metrics['xy_vs_fcu_estimate']['rmse']:.3f} m",
        xlabel="Initial-forward x [m]",
        ylabel="Initial-left y [m]",
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.18)
    ax.legend(fontsize=8)
    bag_label = (
        "2026-04-02 21:46:20 (calibration bag)"
        if bag == "primary"
        else "2026-04-02 21:40:11 (additional bag)"
    )
    fig.suptitle(
        f"Research pool / distributed physics — real vs simulated bag\n{bag_label} | {start:g}–{end:g} s | no time or amplitude fitting",
        fontsize=15,
        fontweight="bold",
    )
    fig.supxlabel(
        "Sensor panels compare real and simulated ROS signals; path compares FCU estimate with simulation truth.\n20 V assumed · unchanged 10 × 5 × 5 m pool · headless replay · identical recorded RC/mode/arm schedule",
        fontsize=10,
    )
    published = np.asarray(replay["published"])
    indices = published[:, 2].astype(int)
    expected = {
        i
        for i, row in enumerate(config["rc"])
        if config["begin"] < row[0] <= config["end"]
    }
    assert expected.issubset(set(indices)) and len(indices) == len(set(indices))
    params = json.loads((path / "fcu_param_readback.json").read_text())
    assert (
        abs(params["ATC_RAT_RLL_D"] - 0.009) < 1e-6
        and abs(params["PSC_ACCZ_P"] - 0.5) < 1e-6
    )
    force = np.genfromtxt(path / "forces.csv", delimiter=",", names=True)
    for corner in ("lf", "lr", "rf", "rr"):
        assert np.allclose(force[f"yaw_{corner}_direct_gain"], 0.12293316274117821)
        assert np.allclose(force[f"ver_{corner}_direct_gain"], 1.0)
    assert np.allclose(force["thruster_supply_voltage_v"], 20.0)
    assert params["SCHED_LOOP_RATE"] == 400.0
    assert not next(
        row for row in reversed(replay["logs"]) if row["kind"] == "state"
    )["armed"]
    fig.savefig(args.output_dir / f"{bag}_overlay.png", dpi=180)
    fig.savefig(args.output_dir / f"{bag}_overlay.svg")
    plt.close(fig)
    np.savetxt(
        args.output_dir / f"{bag}_aligned_signals.csv",
        np.column_stack(columns),
        delimiter=",",
        header=",".join(names),
        comments="",
    )
    truth_u = interp(force["sim_time"] + shift, force["lin_vel_body_x"], grid, 0.2)
    truth_yaw = interp(
        st, relative_yaw(st, track[:, 5:9], config["begin"], arm), grid, 0.3
    )
    truth_z = interp(st, sim_path[:, 2], grid, 0.3)
    report["bags"][bag] = {
        "bag_zip_sha256": config["source_sha256"],
        "window_s": [start, end],
        "metrics_ros_signals": metrics,
        "metrics_truth_for_previous_comparison": {
            "surge_mps": metric(interp(td, real_dvl, grid, 0.3), truth_u),
            "yaw_deg": metric(interp(ti, real_yaw, grid, 0.25), truth_yaw),
            "relative_z_m": metric(interp(tz, real_z, grid, 0.8), truth_z),
        },
        "replay_sim_s": config["end"] - config["begin"],
        "replay_wall_s": replay["wall_replay_s"],
        "rc_published": len(indices),
        "all_expected_rc_once": True,
        "rc_lateness_p50_p95_max_s": np.quantile(
            (published[:, 0] - published[:, 1])[1:], [0.5, 0.95, 1]
        ).tolist(),
        "final_vehicle_state": next(
            row for row in reversed(replay["logs"]) if row["kind"] == "state"
        ),
        "contacts": contact_audit(track, st, start, end),
        "manifest": json.loads((path / "manifest.json").read_text()),
        "fcu_parameter_readback": params,
        "evidence_sha256": {
            str(p.relative_to(args.evidence_root)): hashlib.sha256(
                p.read_bytes()
            ).hexdigest()
            for p in [folder / "numeric.npz", folder / "profile.json", odom_file]
        },
    }
    print(bag, json.dumps(report["bags"][bag], indent=2), flush=True)
(args.output_dir / "comparison.json").write_text(
    json.dumps(report, indent=2, allow_nan=False) + "\n"
)
