#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from replay_april1_real_commands_in_mujoco import (  # noqa: E402
    CURRENT_SCENE,
    LEGACY_SCENE,
    OfflineUuvReplay,
)


OUT_PATH = SCRIPT_DIR / "runs" / "rosbag" / "real_bag_2026_04_01_replay" / "frame_validation.json"


def set_freejoint_state(sim: OfflineUuvReplay, qvel6: np.ndarray, z: float = 0.0) -> None:
    sim.data.qpos[:7] = np.array([0.0, 0.0, z, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    sim.data.qvel[:] = 0.0
    sim.data.qvel[:6] = np.asarray(qvel6, dtype=np.float64)
    mujoco.mj_forward(sim.model, sim.data)


def vector_error(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def validate_scene(name: str, scene: Path, profile: str, fluid_model: str) -> dict[str, Any]:
    sim = OfflineUuvReplay(
        scene=scene,
        profile_name=profile,
        fluid_model=fluid_model,
        thruster_dt_mode="current-code",
    )
    linear_cases = {
        "body_plus_x": np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        "body_plus_y": np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
        "body_plus_z": np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
    }
    angular_cases = {
        "body_roll_rate": np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
        "body_pitch_rate": np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
        "body_yaw_rate": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
    }
    linear_results: dict[str, Any] = {}
    dvl_max_corrected_error = 0.0
    for case, qvel6 in linear_cases.items():
        set_freejoint_state(sim, qvel6)
        lin_body, _ = sim.body_velocity_local()
        gyro_body = np.zeros(3, dtype=np.float64)
        dvl_raw = sim.sensor_value("dvl_vel_body")
        dvl_corrected = sim.dvl_velocity_body_from_sensor(dvl_raw, gyro_body)
        err = vector_error(lin_body, dvl_corrected)
        dvl_max_corrected_error = max(dvl_max_corrected_error, err)
        linear_results[case] = {
            "truth_body_velocity": lin_body.tolist(),
            "dvl_raw_sensor_frame": dvl_raw.tolist() if dvl_raw is not None else None,
            "dvl_bridge_corrected_body": dvl_corrected.tolist() if dvl_corrected is not None else None,
            "corrected_error_norm": err,
        }

    angular_results: dict[str, Any] = {}
    imu_max_corrected_error = 0.0
    for case, qvel6 in angular_cases.items():
        set_freejoint_state(sim, qvel6)
        _, ang_body = sim.body_velocity_local()
        gyro_raw = sim.sensor_value("imu_gyro")
        gyro_body = sim.vector_from_site_to_body(sim.site_ids["imu"], gyro_raw)
        err = vector_error(ang_body, gyro_body)
        imu_max_corrected_error = max(imu_max_corrected_error, err)
        angular_results[case] = {
            "truth_body_angular_velocity": ang_body.tolist(),
            "imu_raw_sensor_frame": gyro_raw.tolist() if gyro_raw is not None else None,
            "imu_bridge_corrected_body": gyro_body.tolist() if gyro_body is not None else None,
            "corrected_error_norm": err,
        }

    set_freejoint_state(sim, np.zeros(6, dtype=np.float64), z=-1.0)
    bar30_id = sim.site_ids["bar30"]
    base_depth = max(0.0, sim.water_surface_z - float(sim.data.xpos[sim.base_id][2]))
    bar30_depth = max(0.0, sim.water_surface_z - float(sim.data.site_xpos[bar30_id][2]))
    bar30_local_z = float(sim.model.site_pos[bar30_id][2]) if bar30_id >= 0 else 0.0
    depth_expected_difference = -bar30_local_z
    depth_observed_difference = bar30_depth - base_depth

    pass_threshold = 1e-8
    return {
        "scene": name,
        "linear_velocity_cases": linear_results,
        "angular_velocity_cases": angular_results,
        "depth_case": {
            "base_depth_truth_m": float(base_depth),
            "bar30_depth_sensor_m": float(bar30_depth),
            "bar30_local_z_m": float(bar30_local_z),
            "observed_bar30_minus_base_depth_m": float(depth_observed_difference),
            "expected_bar30_minus_base_depth_m": float(depth_expected_difference),
            "error_m": float(abs(depth_observed_difference - depth_expected_difference)),
        },
        "pass": bool(
            dvl_max_corrected_error < pass_threshold
            and imu_max_corrected_error < pass_threshold
            and abs(depth_observed_difference - depth_expected_difference) < pass_threshold
        ),
        "max_errors": {
            "dvl_corrected_vs_body_truth": dvl_max_corrected_error,
            "imu_corrected_vs_body_truth": imu_max_corrected_error,
            "depth_offset": float(abs(depth_observed_difference - depth_expected_difference)),
        },
    }


def main() -> None:
    payload = {
        "purpose": (
            "Validate the replay/ROS2 bridge sensor-frame contract before physical "
            "parameter A/B tests. Raw DVL uses the MuJoCo dvl_site frame; replay "
            "control and odometry paths use the bridge-corrected body-frame value, "
            "while /dvl/twist path comparisons use the published DVL-frame value."
        ),
        "scenes": [
            validate_scene("current", CURRENT_SCENE, "current", "current"),
            validate_scene("legacy", LEGACY_SCENE, "legacy", "legacy"),
        ],
    }
    payload["pass"] = all(scene["pass"] for scene in payload["scenes"])
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(payload, indent=2))
    print(json.dumps(payload, indent=2))
    if not payload["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
