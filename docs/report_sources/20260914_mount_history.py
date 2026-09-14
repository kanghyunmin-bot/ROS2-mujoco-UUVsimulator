"""Audit June sensor coordinates and their kinematic effect on the April bag.

Read historical launch code as syntax only. Do not execute it or change ROS TF.
"""

import argparse
import ast
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def launch_defaults(path):
    defaults = {}
    for node in ast.walk(ast.parse(path.read_text())):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        name = ast.literal_eval(node.args[0])
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(
                keyword.value, ast.Constant
            ):
                defaults[name] = keyword.value.value
    return defaults


def stats(values):
    return {
        "rms": np.sqrt(np.mean(values**2, axis=0)).tolist(),
        "abs_p95": np.quantile(abs(values), 0.95, axis=0).tolist(),
        "abs_max": np.max(abs(values), axis=0).tolist(),
        "n": len(values),
    }


def rollout_sensor_points(directory, historical):
    """Compare frozen body signals with historical sensor points; no refitting."""
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[2] / "uuv_mujoco/current/tools")
    )
    from rosbag_trend_math import interpolate_valid, metrics, point_velocity, trend

    original = json.loads((directory / "report.json").read_text())
    arrays = np.load(directory / "aligned_signals.npz", allow_pickle=False)
    grid = arrays["time_s"]
    results = {}
    for name, run in original["runs"].items():
        paths = {Path(p).name: Path(p) for p in run["input_sha256"]}
        for path in paths.values():
            if (
                hashlib.sha256(path.read_bytes()).hexdigest()
                != run["input_sha256"][str(path)]
            ):
                raise ValueError("Changed recorded rollout input: " + str(path))
        base = ET.parse(paths["forces.csv"].parent / "clearance_tank.xml").find(
            ".//body[@name='base_link']"
        )
        inertia = base.find("inertial")
        if inertia is None or not np.allclose(
            np.fromstring(inertia.get("quat", "1 0 0 0"), sep=" "), [1, 0, 0, 0]
        ):
            raise ValueError("BODY-local CSV requires aligned inertial axes")
        forces = np.genfromtxt(paths["forces.csv"], delimiter=",", names=True)
        replay = json.loads(paths["replay.json"].read_text())
        offset = replay["begin_bag_time"] - replay["origin_sim_time"]
        st = forces["sim_time"] + offset
        gt = np.asarray(replay["tracks"]["/mujoco/ground_truth/pose"])
        gt_t = gt[:, 1] + offset
        velocity = np.column_stack([forces["lin_vel_body_" + axis] for axis in "xyz"])
        omega = np.column_stack([forces["ang_vel_body_" + axis] for axis in "xyz"])
        lever = (
            np.asarray(historical["dvl_site"]["position_body_m"])
            - run["geometry"]["runtime_component_com_body_m"]
        )
        sensor_velocity = point_velocity(velocity, omega, lever)
        pressure_position = historical["bar30_site"]["position_body_m"]
        depth_offset = -Rotation.from_quat(gt[:, 5:9]).apply(
            np.tile(pressure_position, (len(gt), 1))
        )[:, 2]
        depth_offset -= np.median(depth_offset[(gt_t >= 10) & (gt_t < 17)])
        predictions = {
            "surge": interpolate_valid(
                st,
                sensor_velocity[:, 0],
                grid - run["signals"]["surge"]["phase"]["selected_lag_s"],
                max_gap_s=0.15,
            ),
            "relative_depth": arrays[name + "_relative_depth_selected"]
            + interpolate_valid(
                gt_t,
                depth_offset,
                grid - run["signals"]["relative_depth"]["phase"]["selected_lag_s"],
                max_gap_s=0.15,
            ),
        }
        results[name] = {}
        for signal, point in predictions.items():
            body, real = (
                arrays[name + "_" + signal + "_selected"],
                arrays["real_" + signal],
            )
            margin = run["signals"][signal]["report_split_margin_s"]
            window = original["splits_s"]["report_only_test"]
            mask = (
                (grid >= window[0] + margin)
                & (grid < window[1] - margin)
                & np.isfinite(trend(real, 7))
                & np.isfinite(trend(body, 7))
                & np.isfinite(trend(point, 7))
            )
            results[name][signal] = {
                label: {
                    "raw": metrics(real[mask], curve[mask]),
                    "trend": metrics(trend(real, 7)[mask], trend(curve, 7)[mask]),
                }
                for label, curve in (
                    ("body_reference", body),
                    ("historical_sensor_point", point),
                )
            }
    return {
        "selected_phase_frozen": True,
        "historical_tf_applied_to_recorded_bag": False,
        "source_report_sha256": hashlib.sha256(
            (directory / "report.json").read_bytes()
        ).hexdigest(),
        "runs": results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "historic_launch",
        "historic_scene",
        "numeric_npz",
        "topic_profile",
        "bag_audit",
        "current_mounts",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument(
        "--comparison_dir",
        type=Path,
        help="Optional frozen comparison report/NPZ for sensor-point sensitivity",
    )
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Refusing to overwrite historical audit")
    for path, expected in (
        (
            args.historic_launch,
            "204dd23efde5cc334e543c714186c3cda5f19d91ccaf33b889d5036a3e10749e",
        ),
        (
            args.historic_scene,
            "d4650929a59afe662b96bd59fc4e891eba5055762963a1ea131cb5645fcb00ac",
        ),
    ):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(
                "Historical source does not match the recorded Git reference"
            )
    defaults = launch_defaults(args.historic_launch)
    historical = {}
    prefixes = {"imu_site": "base_to_fcu", "dvl_site": "dvl", "bar30_site": "depth"}
    for site, prefix in prefixes.items():
        historical[site] = {
            "position_body_m": [float(defaults[prefix + "_" + axis]) for axis in "xyz"],
            "rpy_rad": [
                float(defaults[prefix + "_" + axis])
                for axis in ("roll", "pitch", "yaw")
            ],
        }
    if any(
        float(defaults["imu_" + axis]) != 0
        for axis in ("x", "y", "z", "roll", "pitch", "yaw")
    ):
        raise ValueError(
            "Compose the nonidentity fcu-to-imu transform before using this audit"
        )
    if historical["imu_site"]["rpy_rad"] != [0, 0, 0]:
        raise ValueError("Rotate AHRS gyro/orientation into body before this audit")
    scene = ET.parse(args.historic_scene).find(".//body[@name='base_link']")
    current = json.loads(args.current_mounts.read_text())["site_positions"]
    bag = json.loads(args.bag_audit.read_text())["static_transforms"]
    bag_children = {v["child"]: v for v in bag if v["parent"] == "base_link"}
    children = {"imu_site": "fcu_link", "dvl_site": "dvl", "bar30_site": "depth_link"}
    for site, value in historical.items():
        entry = scene.find("site[@name='" + site + "']")
        scene_position = np.fromstring(entry.attrib["pos"], sep=" ")
        old = np.asarray(value["position_body_m"])
        delta = np.asarray(current[site]) - old
        value.update(
            {
                "june_scene_position_body_m": scene_position.tolist(),
                "june_scene_quat_wxyz": np.fromstring(
                    entry.attrib["quat"], sep=" "
                ).tolist(),
                "scene_rounding_difference_m": float(
                    np.linalg.norm(scene_position - old)
                ),
                "current_cad_position_body_m": current[site],
                "current_minus_historical_m": delta.tolist(),
                "position_distance_m": float(np.linalg.norm(delta)),
                "bag_tf_position_m": bag_children[children[site]]["xyz"],
                "bag_tf_position_matches_launch": bool(
                    np.allclose(
                        old, bag_children[children[site]]["xyz"], atol=1e-10, rtol=0
                    )
                ),
            }
        )
    numeric = np.load(args.numeric_npz, allow_pickle=False)
    topics = json.loads(args.topic_profile.read_text())
    origin = min(v["start"] for v in topics.values())
    imu = numeric["__mavros__imu__data"]
    time = imu[:, 0] - origin
    if not np.all(np.isfinite(imu)) or np.any(np.diff(time) <= 0):
        raise ValueError("Invalid AHRS samples")
    quat = imu[:, 1:5]
    if np.max(abs(np.linalg.norm(quat, axis=1) - 1)) > 0.01:
        raise ValueError("Invalid AHRS quaternions")
    active = (time >= 20) & (time < 70)
    initial = (time >= 10) & (time < 17)
    if min(sum(active), sum(initial)) < 20:
        raise ValueError("Insufficient active/initial measurements")
    dvl_delta = np.asarray(historical["dvl_site"]["current_minus_historical_m"])
    pressure_delta = np.asarray(historical["bar30_site"]["current_minus_historical_m"])
    # Translational mount effect only; no unknown DVL frame reinterpretation.
    velocity_delta = np.cross(imu[:, 5:8], dvl_delta)
    depth_delta = -Rotation.from_quat(quat).apply(
        np.tile(pressure_delta, (len(imu), 1))
    )[:, 2]
    relative_depth_delta = depth_delta - np.median(depth_delta[initial])
    report = {
        "schema": "uuv.historical_mount_audit.v1",
        "historical_references": {
            "simulator_commit": "9251007ec5c439c777fd61a65a08a5c89c2edb39",
            "simulator_commit_date": "2026-06-25",
            "scene_path": "uuv_mujoco/v2.2/scenes/tank_current_scene.xml",
            "ros_gitlink_path": "rospkg/kmu26_auv",
            "ros_repository": "https://github.com/kanghyunmin-bot/kmu26_auv",
            "ros_commit": "9f50aab20aa43fae09c616ef36f8c35b0d79234c",
            "ros_commit_date": "2026-05-04",
            "ros_launch_path": "launch/rov_start.launch.py",
        },
        "input_sha256": {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for k, p in vars(args).items()
            if k not in ("output", "comparison_dir")
        },
        "sensors": historical,
        "effects_from_recorded_real_ahrs": {
            "window_s": [20, 70],
            "initial_window_s": [10, 17],
            "dvl_delta_velocity_body_mps": stats(velocity_delta[active]),
            "pressure_absolute_depth_delta_m": stats(depth_delta[active]),
            "pressure_relative_depth_delta_m": stats(relative_depth_delta[active]),
        },
        "interpretation": [
            "Historical TF is a supported configuration reference, not measured April extrinsics.",
            "Current CAD positions are also estimates; coordinate differences do not prove physical relocation.",
            "Uses recorded real AHRS and rigid-body kinematics; no new physical rollout or parameter fitting.",
            "Old DVL published TF is identity while its MuJoCo site has roll pi. Do not infer output axes from site rotation alone.",
            "Translation alone does not change gyro readings. Unknown sensor axis conventions remain separate.",
            "Pressure height change includes a constant offset; initial-zero relative depth removes that constant.",
        ],
    }
    if args.comparison_dir:
        report["recorded_rollout_sensor_point_sensitivity"] = rollout_sensor_points(
            args.comparison_dir, historical
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(
        json.dumps(
            {
                "positions": {
                    k: v["position_distance_m"] for k, v in historical.items()
                },
                "effects": report["effects_from_recorded_real_ahrs"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
