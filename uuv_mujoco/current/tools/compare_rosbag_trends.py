"""Compare an audited rosbag with recorded real MuJoCo/SITL rollouts.

This offline tool never publishes ROS commands or changes runtime defaults.
Input NPZ schemas are documented in docs/contracts/ROSBAG_TREND_ALIGNMENT.md.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np
from rosbag_trend_math import (
    clock_diagnostics,
    interpolate_valid,
    metrics,
    observation_envelope,
    point_velocity,
    sample_envelope,
    select_phase,
    trend,
)
from scipy.spatial.transform import Rotation

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def geometry(scene_path, profile_path, profile_name):
    """Read compiled geometry and declared runtime mass; never trust old bag TF."""
    import mujoco
    from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles

    # Recorded Docker scenes retain absolute /workspace asset paths. Relocate
    # only that explicit mount prefix in memory; preserve the evidence file.
    xml = scene_path.read_text().replace("/workspace/", str(CURRENT.parents[1]) + "/")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base < 0:
        raise ValueError("Scene has no base_link")
    rotation = data.xmat[base].reshape(3, 3)
    sites = {}
    for name in (
        "imu_site",
        "bar30_site",
        "dvl_site",
        "cam_left_site",
        "cam_right_site",
    ):
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        if i >= 0:
            sites[name] = (rotation.T @ (data.site_xpos[i] - data.xpos[base])).tolist()
    profiles, _ = load_sim_profiles(profile_path)
    profile = build_sim_profile(profiles, profile_name)
    components = profile.get("body_components", [])
    mass = sum(c["mass"] for c in components)
    com = (
        sum((c["mass"] * np.asarray(c["mass_pos"]) for c in components), np.zeros(3))
        / mass
        if mass
        else model.body_ipos[base]
    )
    if not np.allclose(model.body_iquat[base], [1, 0, 0, 0]):
        raise ValueError(
            "Recorded BODY-local force CSV needs an inertial-to-body rotation for this scene"
        )
    return {
        "engine_version_for_geometry_read": mujoco.__version__,
        "scene_sha256": digest(scene_path),
        "profile_sha256": digest(profile_path),
        "profile_name": profile_name,
        "compiled_base_mass_kg": float(model.body_mass[base]),
        "runtime_component_mass_kg": mass,
        "runtime_component_com_body_m": com.tolist(),
        "compiled_base_inertia_kg_m2": model.body_inertia[base].tolist(),
        "sites_body_m": sites,
        "body_components": components,
        "geom_count": model.ngeom,
        "note": "CAD/configuration quantities, not measurements of the April vehicle. Compiled inertia is before runtime profile overrides.",
    }


def initial_path(t, pos, quat, window):
    """Align only initial translation/heading, never fit path scale or closure."""
    mask = (t >= window[0]) & (t < window[1])
    if sum(mask) < 5:
        raise ValueError("Insufficient initial stationary pose support")
    if (
        not np.all(np.isfinite(quat))
        or np.max(abs(np.linalg.norm(quat, axis=1) - 1)) > 0.01
    ):
        raise ValueError("Invalid path quaternions")
    yaw = np.unwrap(Rotation.from_quat(quat).as_euler("xyz")[:, 2])
    yaw0 = np.median(yaw[mask])
    center = np.median(pos[mask], axis=0)
    aligned = Rotation.from_euler("z", -yaw0).apply(pos - center)
    return (
        aligned,
        yaw - yaw0,
        {"initial_position_m": center.tolist(), "initial_yaw_rad": float(yaw0)},
    )


def run(args):
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise ValueError("Refusing to overwrite existing results")
    source = json.loads(args.source_manifest.read_text())
    if (
        source.get("sha256")
        != "195aca87161fb180bc7c7ee0574523f7840399e439f7b1d245313d8d2faf2d52"
        or source.get("sqlite_sha256")
        != "e3f60d33dbf72f9c4df223f8afe7e389f8c49ffc6ff7031647ce3da32ec3c154"
    ):
        raise ValueError("This reviewed window preset applies only to the April 2 bag")
    # Inputs remain untouched. Hash every derivative, not just the source manifest.
    hashes = {
        str(p): digest(p)
        for p in (
            args.numeric_npz,
            args.odometry_npz,
            args.profile_json,
            args.source_manifest,
        )
    }
    for name, path in (
        ("numeric.npz", args.numeric_npz),
        ("odometry.npz", args.odometry_npz),
        ("profile.json", args.profile_json),
    ):
        if source.get("derived_sha256", {}).get(name) != hashes[str(path)]:
            raise ValueError("Derivative does not match its source manifest: " + name)
    numeric = np.load(args.numeric_npz, allow_pickle=False)
    odometry = np.load(args.odometry_npz, allow_pickle=False)
    topics = json.loads(args.profile_json.read_text())
    origin = min(v["start"] for v in topics.values())
    imu, dvl = numeric["__mavros__imu__data"], numeric["__dvl__data"]
    depth = numeric["__depth__pose"]
    pressure = numeric["__mavros__imu__static_pressure"]
    grid = np.arange(10.0, 72.0, 0.05)
    train, validation, test = (20.0, 40.0), (40.5, 55.0), (55.5, 70.0)
    noise = {}
    inputs = {
        "dvl": (dvl, slice(1, 4), dvl[:, 5] > 0.5, 0.3),
        "gyro": (imu, slice(5, 8), np.ones(len(imu), dtype=bool), 0.15),
        "accel": (imu, slice(8, 11), np.ones(len(imu), dtype=bool), 0.15),
        "pressure": (pressure, slice(1, 2), np.ones(len(pressure), dtype=bool), 0.85),
    }
    for name, (rows, cols, valid, gap) in inputs.items():
        pair = {}
        for label, window in (("fit", (90, 115)), ("evaluation", (115, 140))):
            mask = (
                (rows[:, 0] - origin >= window[0])
                & (rows[:, 0] - origin < window[1])
                & valid
            )
            pair[label] = observation_envelope(
                rows[mask, 0] - origin, rows[mask, cols], max_gap_s=gap
            )
            pair[label]["unit"] = {
                "dvl": "m/s",
                "gyro": "rad/s",
                "accel": "m/s^2",
                "pressure": "Pa",
            }[name]
        # Ensemble statistics, not a selected lucky noise seed; no noise in RMSE.
        simulated_std = np.array(
            [
                sample_envelope(pair["fit"], pair["evaluation"]["n"], s).std(
                    axis=0, ddof=1
                )
                for s in range(128)
            ]
        )
        pair["sample_std_ensemble_p05_p50_p95"] = np.quantile(
            simulated_std, [0.05, 0.5, 0.95], axis=0
        ).tolist()
        pair["observed_eval_std_inside_ensemble"] = (
            (
                np.asarray(pair["evaluation"]["std"])
                >= np.quantile(simulated_std, 0.05, axis=0)
            )
            & (
                np.asarray(pair["evaluation"]["std"])
                <= np.quantile(simulated_std, 0.95, axis=0)
            )
        ).tolist()
        noise[name] = pair
    real_signals = {
        "surge": interpolate_valid(
            dvl[:, 0] - origin, dvl[:, 1], grid, max_gap_s=0.3, valid=dvl[:, 5] > 0.5
        ),
        "yaw_rate": interpolate_valid(
            imu[:, 0] - origin, imu[:, 7], grid, max_gap_s=0.15
        ),
        "relative_depth": -interpolate_valid(
            depth[:, 0] - origin, depth[:, 1], grid, max_gap_s=0.85
        ),
    }
    d0 = (grid >= 10) & (grid < 17)
    real_signals["relative_depth"] -= np.nanmedian(real_signals["relative_depth"][d0])
    real_paths = {}
    for key in ("__mavros__local_position__odom", "__odometry__filtered"):
        a = odometry[key]
        p, yaw, anchor = initial_path(a[:, 0], a[:, 2:5], a[:, 5:9], (10, 17))
        real_paths[key] = {
            "position": np.column_stack(
                [
                    interpolate_valid(a[:, 0], p[:, j], grid, max_gap_s=0.65)
                    for j in range(3)
                ]
            ),
            "yaw": interpolate_valid(a[:, 0], yaw, grid, max_gap_s=0.65),
            "anchor": anchor,
        }
    report = {
        "schema": "uuv.rosbag_trends.v1",
        "source_zip_sha256": source["sha256"],
        "input_sha256": hashes,
        "operator_context": {
            "tether_connected": True,
            "thrusters_similar": True,
            "buoyancy": "slightly_positive_near_neutral",
            "old_tf_trusted": False,
            "sensor_mounts_changed": True,
        },
        "time_basis": "record/receipt time; real RCOut header known ~4295 s anomaly; no inferred clock rewrite",
        "clock_audit": {
            key.removesuffix("__times_ns"): clock_diagnostics(numeric[key])
            for key in numeric.files
            if key.endswith("__times_ns") and "rc__override" not in key
        },
        "splits_s": {
            "phase_fit": train,
            "phase_validation": validation,
            "report_only_test": test,
        },
        "limitations": [
            "Same bag; previous physical profile was already selected with portions of this recording.",
            "Real paths are FCU/ROS estimates, not independent position ground truth.",
            "Unknown April mount/axes and tether forces; current CAD lever arm is sensitivity only.",
            "Output noise envelope includes residual motion and filtering; not raw IMU calibration.",
            "No frame-by-frame warp, path scaling, test-set fitting or copied sensor input into plant.",
        ],
        "noise": noise,
        "runs": {},
    }
    arrays = {"time_s": grid, **{"real_" + k: v for k, v in real_signals.items()}}
    for name, directory in args.runs:
        replay_path, force_path = directory / "replay.json", directory / "forces.csv"
        replay = json.loads(replay_path.read_text())
        if not replay.get("complete"):
            raise ValueError("Incomplete simulation replay")
        forces = np.genfromtxt(force_path, delimiter=",", names=True)
        offset = replay["begin_bag_time"] - replay["origin_sim_time"]
        st = forces["sim_time"] + offset
        gt = np.asarray(replay["tracks"]["/mujoco/ground_truth/pose"])
        gt_t = gt[:, 1] + offset
        pos, yaw, anchor = initial_path(gt_t, gt[:, 2:5], gt[:, 5:9], (10, 17))
        recorded_commands = json.loads((directory / "commands.json").read_text())
        command = recorded_commands["mujoco"]
        profile_name = args.sim_profile or command[command.index("--profile") + 1]
        geo = geometry(
            directory / "clearance_tank.xml",
            directory / "profiles.json",
            profile_name,
        )
        signal_report = {}
        model_signals = {
            "surge": forces["lin_vel_body_x"],
            "yaw_rate": forces["ang_vel_body_z"],
            "relative_depth": forces["base_depth_m"]
            - np.median(forces["base_depth_m"][(st >= 10) & (st < 17)]),
        }
        for key, prediction in model_signals.items():
            sigma = {
                "surge": noise["dvl"]["fit"]["std"][0],
                "yaw_rate": noise["gyro"]["fit"]["std"][2],
                "relative_depth": noise["pressure"]["fit"]["std"][0] / (997 * 9.80665),
            }[key]
            phase, baseline, selected, candidate, support = select_phase(
                grid,
                real_signals[key],
                st,
                prediction,
                train=train,
                validation=validation,
                noise_sigma=sigma,
                max_lag_s=args.max_lag_s,
            )
            parts = {}
            margin = max(0.5, phase["split_margin_s"])
            for label, window in (
                ("train", train),
                ("validation", validation),
                ("test", test),
            ):
                # Leave support around each boundary for all compared curves.
                mask = (
                    support & (grid >= window[0] + margin) & (grid < window[1] - margin)
                )
                parts[label] = {}
                for mode, curve in (
                    ("unaligned", baseline),
                    ("selected", selected),
                    ("candidate", candidate),
                ):
                    parts[label][mode] = {
                        "raw": metrics(
                            real_signals[key][mask], curve[mask], noise_sigma=sigma
                        ),
                        "trend": metrics(
                            trend(real_signals[key], 7)[mask],
                            trend(curve, 7)[mask],
                            noise_sigma=sigma,
                        ),
                    }
            signal_report[key] = {
                "phase": phase,
                "metrics": parts,
                "stationary_output_sigma": sigma,
                "report_split_margin_s": margin,
            }
            arrays[name + "_" + key + "_unaligned"] = baseline
            arrays[name + "_" + key + "_selected"] = selected
        # Path gets only the selected gyro offset, not its own trajectory fit.
        lag = signal_report["yaw_rate"]["phase"]["selected_lag_s"]
        path_report = {}
        paths = {
            mode: interpolate_valid(gt_t, pos, grid - shift, max_gap_s=0.15)
            for mode, shift in (("unaligned", 0.0), ("selected", lag))
        }
        margin = signal_report["yaw_rate"]["report_split_margin_s"]
        for ref_name, ref in real_paths.items():
            path_report[ref_name] = {}
            common = np.all(np.isfinite(ref["position"]), axis=1)
            for path in paths.values():
                common &= np.all(np.isfinite(path), axis=1)
            for mode, path in paths.items():
                for label, window in (
                    ("train", train),
                    ("validation", validation),
                    ("test", test),
                ):
                    mask = (
                        (grid >= window[0] + margin)
                        & (grid < window[1] - margin)
                        & common
                    )
                    delta = path[mask] - ref["position"][mask]
                    path_report[ref_name][mode + "_" + label] = {
                        "n": int(sum(mask)),
                        "xy_rmse_m": float(
                            np.sqrt(np.mean(np.sum(delta[:, :2] ** 2, axis=1)))
                        )
                        if sum(mask) >= 3
                        else None,
                        "z_rmse_m": float(np.sqrt(np.mean(delta[:, 2] ** 2)))
                        if sum(mask) >= 3
                        else None,
                    }
                arrays[name + "_path_" + mode] = path
            arrays["real_path_" + ref_name] = ref["position"]
        velocities = np.column_stack([forces["lin_vel_body_" + axis] for axis in "xyz"])
        angular = np.column_stack([forces["ang_vel_body_" + axis] for axis in "xyz"])
        lever = (
            np.asarray(geo["sites_body_m"]["dvl_site"])
            - geo["runtime_component_com_body_m"]
        )
        effect = point_velocity(velocities, angular, lever) - velocities
        active = (st >= 20) & (st < 70)
        report["runs"][name] = {
            "input_sha256": {
                str(p): digest(p)
                for p in (replay_path, force_path, directory / "commands.json")
            },
            "geometry": geo,
            "signals": signal_report,
            "path": path_report,
            "initial_anchor": anchor,
            "current_dvl_lever_arm_sensitivity": {
                "lever_body_m": lever.tolist(),
                "velocity_delta_abs_p95_mps": np.quantile(
                    abs(effect[active]), 0.95, axis=0
                ).tolist(),
                "applied_to_bag": False,
            },
        }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    np.savez_compressed(args.output_dir / "aligned_signals.npz", **arrays)
    envelope = {
        "schema": "uuv.empirical_observation_envelope.v1",
        "calibration_status": "empirical_output_envelope_unvalidated",
        "runtime_default_changed": False,
        "sample_interval_must_match": True,
        "source_zip_sha256": source["sha256"],
        "sensors": {k: v["fit"] for k, v in noise.items()},
    }
    (args.output_dir / "observation_envelope.json").write_text(
        json.dumps(envelope, indent=2) + "\n"
    )
    print(
        json.dumps(
            {
                name: {
                    k: {
                        "lag_s": v["phase"]["selected_lag_s"],
                        "candidate_lag_s": v["phase"]["candidate_lag_s"],
                        "test": v["metrics"]["test"]["selected"],
                    }
                    for k, v in run["signals"].items()
                }
                for name, run in report["runs"].items()
            },
            indent=2,
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "numeric_npz",
        "odometry_npz",
        "profile_json",
        "source_manifest",
        "output_dir",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        help="label=directory containing recorded replay.json/forces.csv/scene/profiles",
    )
    parser.add_argument(
        "--sim_profile",
        help="Override recorded profile name for geometry inspection only",
    )
    parser.add_argument("--max_lag_s", type=float, default=0.3)
    args = parser.parse_args()
    if any(not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*=.+", entry) for entry in args.run):
        raise ValueError("Use label=directory with an alphanumeric/underscore label")
    args.runs = [
        (entry.split("=", 1)[0], Path(entry.split("=", 1)[1])) for entry in args.run
    ]
    if len({name for name, _ in args.runs}) != len(args.runs):
        raise ValueError("Duplicate run labels")
    run(args)


if __name__ == "__main__":
    main()
