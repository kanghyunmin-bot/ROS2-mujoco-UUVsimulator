#!/usr/bin/env python3
"""Run a constrained UUV MuJoCo parameter sweep against a real ROS bag.

This intentionally keeps ArduPilot untouched.  It edits only the MuJoCo
profile/scene files while each candidate is running, then restores the original
files unless --apply-best is explicitly requested.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parents[1]
UUV_DIR = ROOT_DIR / "uuv_mujoco" / "v2.2"
PROFILE_PATH = UUV_DIR / "config" / "sim_profiles.json"
SCENE_PATH = UUV_DIR / "scenes" / "tank_current_scene.xml"
RUN_REPLAY = SCRIPT_DIR / "run_closed_loop_april1_replay.sh"
DEFAULT_BAG = ROOT_DIR / (
    "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
)
DEFAULT_OUT = SCRIPT_DIR / "runs" / "autotune"

FLUID_GEOMS = (
    "fluid_center_enclosure",
    "fluid_port_lower_body",
    "fluid_starboard_lower_body",
)

CURRENT_PROC: subprocess.Popen[str] | None = None


@dataclass(frozen=True)
class Candidate:
    name: str
    profile_updates: dict[str, Any] = field(default_factory=dict)
    fluid_linear_scale: float = 1.0
    fluid_rot_scale: float = 1.0
    fluid_blunt_scale: float = 1.0
    fluid_slender_scale: float = 1.0
    fluid_angular_scale: float = 1.0
    fluid_kutta: float | None = None
    fluid_magnus: float | None = None


def default_candidates(candidate_set: str) -> list[Candidate]:
    if candidate_set == "ellipsoid5":
        return [
            Candidate("baseline"),
            Candidate("blunt_p05", fluid_blunt_scale=1.05),
            Candidate("blunt_p10", fluid_blunt_scale=1.10),
            Candidate("slender_p05", fluid_slender_scale=1.05),
            Candidate("slender_p10", fluid_slender_scale=1.10),
            Candidate("angular_p05", fluid_angular_scale=1.05),
            Candidate("angular_p10", fluid_angular_scale=1.10),
            Candidate("drag_all_p08", fluid_linear_scale=1.08),
            Candidate("drag_all_p12", fluid_linear_scale=1.12),
            Candidate("kutta_025", fluid_kutta=0.025),
            Candidate("kutta_050", fluid_kutta=0.050),
            Candidate("magnus_025", fluid_magnus=0.025),
            Candidate("magnus_050", fluid_magnus=0.050),
            Candidate("kutta_050_magnus_025", fluid_kutta=0.050, fluid_magnus=0.025),
            Candidate("drag_p10_angular_p05", fluid_linear_scale=1.10, fluid_angular_scale=1.05),
        ]
    if candidate_set == "ellipsoid5-refine":
        return [
            Candidate("baseline"),
            Candidate("drag_p08_angular_p03", fluid_linear_scale=1.08, fluid_angular_scale=1.03),
            Candidate("drag_p08_angular_p05", fluid_linear_scale=1.08, fluid_angular_scale=1.05),
            Candidate("drag_p10_angular_p03", fluid_linear_scale=1.10, fluid_angular_scale=1.03),
            Candidate("drag_p10_angular_p05", fluid_linear_scale=1.10, fluid_angular_scale=1.05),
            Candidate("drag_p10_angular_p08", fluid_linear_scale=1.10, fluid_angular_scale=1.08),
            Candidate("drag_p12_angular_p05", fluid_linear_scale=1.12, fluid_angular_scale=1.05),
            Candidate("drag_p12_angular_p08", fluid_linear_scale=1.12, fluid_angular_scale=1.08),
            Candidate("blunt_p10_angular_p05", fluid_blunt_scale=1.10, fluid_angular_scale=1.05),
            Candidate("slender_p10_angular_p05", fluid_slender_scale=1.10, fluid_angular_scale=1.05),
        ]
    if candidate_set == "ellipsoid5-final":
        return [
            Candidate("baseline"),
            Candidate("drag_p12_angular_p08", fluid_linear_scale=1.12, fluid_angular_scale=1.08),
            Candidate("drag_p12_angular_p10", fluid_linear_scale=1.12, fluid_angular_scale=1.10),
            Candidate("drag_p14_angular_p08", fluid_linear_scale=1.14, fluid_angular_scale=1.08),
            Candidate("drag_p14_angular_p10", fluid_linear_scale=1.14, fluid_angular_scale=1.10),
            Candidate("drag_p14_angular_p12", fluid_linear_scale=1.14, fluid_angular_scale=1.12),
            Candidate("drag_p16_angular_p08", fluid_linear_scale=1.16, fluid_angular_scale=1.08),
            Candidate("drag_p16_angular_p10", fluid_linear_scale=1.16, fluid_angular_scale=1.10),
            Candidate("drag_p18_angular_p10", fluid_linear_scale=1.18, fluid_angular_scale=1.10),
            Candidate("drag_p20_angular_p12", fluid_linear_scale=1.20, fluid_angular_scale=1.12),
        ]
    if candidate_set == "rosbag-yaw":
        return [
            Candidate("baseline"),
            Candidate("drag_all_p12", fluid_linear_scale=1.12),
            Candidate("yaw_p05", {"yaw_torque_scale": 1.05}),
            Candidate("yaw_p10", {"yaw_torque_scale": 1.10}),
            Candidate("drag_p12_yaw_p05", {"yaw_torque_scale": 1.05}, fluid_linear_scale=1.12),
            Candidate("drag_p12_yaw_p10", {"yaw_torque_scale": 1.10}, fluid_linear_scale=1.12),
            Candidate("angular_m05", fluid_angular_scale=0.95),
            Candidate("angular_m10", fluid_angular_scale=0.90),
            Candidate("drag_p12_angular_m05", fluid_linear_scale=1.12, fluid_angular_scale=0.95),
            Candidate("drag_p12_angular_m10", fluid_linear_scale=1.12, fluid_angular_scale=0.90),
        ]
    if candidate_set == "closed-loop-refine":
        return [
            Candidate("baseline"),
            Candidate("angular_p10", fluid_angular_scale=1.10),
            Candidate("angular_p20", fluid_angular_scale=1.20),
            Candidate("angular_p35", fluid_angular_scale=1.35),
            Candidate("linear_p10_angular_p10", fluid_linear_scale=1.10, fluid_angular_scale=1.10),
            Candidate("linear_p20_angular_p20", fluid_linear_scale=1.20, fluid_angular_scale=1.20),
        ]
    if candidate_set == "lift-refine":
        return [
            Candidate("baseline"),
            Candidate("kutta_075_magnus_025", fluid_kutta=0.075, fluid_magnus=0.025),
            Candidate("kutta_100_magnus_025", fluid_kutta=0.100, fluid_magnus=0.025),
            Candidate("kutta_075_magnus_050", fluid_kutta=0.075, fluid_magnus=0.050),
            Candidate("kutta_100_magnus_050", fluid_kutta=0.100, fluid_magnus=0.050),
        ]
    if candidate_set == "fluid-focus":
        return [
            Candidate("baseline"),
            Candidate("fluid_linear_p03", fluid_linear_scale=1.03),
            Candidate("fluid_linear_p05", fluid_linear_scale=1.05),
            Candidate("fluid_linear_p08", fluid_linear_scale=1.08),
            Candidate("fluid_linear_p10", fluid_linear_scale=1.10),
            Candidate("fluid_linear_p12", fluid_linear_scale=1.12),
            Candidate("fluid_linear_p15", fluid_linear_scale=1.15),
            Candidate("fluid_rot_p05", fluid_rot_scale=1.05),
            Candidate("fluid_rot_p10", fluid_rot_scale=1.10),
            Candidate("fluid_linear_p10_rot_p05", fluid_linear_scale=1.10, fluid_rot_scale=1.05),
            Candidate("fluid_linear_p12_rot_p05", fluid_linear_scale=1.12, fluid_rot_scale=1.05),
        ]
    if candidate_set == "broad":
        return [
            Candidate("baseline"),
            Candidate(
                "heave_damping_low",
                {"surface_heave_damping": 10.0, "heave_damping_scale": 3.2},
            ),
            Candidate(
                "heave_damping_high",
                {"surface_heave_damping": 15.0, "heave_damping_scale": 5.0},
            ),
            Candidate(
                "restore_stronger",
                {"cob_torque_scale": 2.2, "cob_z_offset": 0.050},
            ),
            Candidate(
                "restore_softer",
                {"cob_torque_scale": 1.45, "cob_z_offset": 0.032},
            ),
            Candidate(
                "surge_drag_low",
                {"linear_drag": 0.78},
                fluid_linear_scale=0.85,
            ),
            Candidate(
                "surge_drag_high",
                {"linear_drag": 1.10},
                fluid_linear_scale=1.18,
            ),
            Candidate(
                "angular_drag_low",
                {"angular_drag": 0.60},
                fluid_rot_scale=0.85,
            ),
            Candidate(
                "angular_drag_high",
                {"angular_drag": 0.92},
                fluid_rot_scale=1.15,
            ),
        ]
    if candidate_set == "rosbag-axis":
        return [
            Candidate("baseline"),
            Candidate(
                "xy_drag_xup_ylow",
                {
                    "linear_damping_linear": [1.30, 0.90, 1.54],
                    "quadratic_damping_linear": [1.80, 1.20, 2.40],
                },
            ),
            Candidate(
                "y_drag_low",
                {
                    "linear_damping_linear": [1.10, 0.80, 1.54],
                    "quadratic_damping_linear": [1.40, 1.10, 2.40],
                },
            ),
            Candidate(
                "z_drag_low",
                {
                    "linear_damping_linear": [1.10, 1.32, 0.90],
                    "quadratic_damping_linear": [1.40, 2.00, 1.20],
                    "surface_heave_damping": 4.0,
                    "heave_damping_scale": 2.5,
                },
            ),
            Candidate(
                "xyz_drag_shaped",
                {
                    "linear_damping_linear": [1.30, 0.85, 0.95],
                    "quadratic_damping_linear": [1.80, 1.15, 1.25],
                    "surface_heave_damping": 4.0,
                    "heave_damping_scale": 2.8,
                },
            ),
            Candidate(
                "x_drag_up_only",
                {
                    "linear_damping_linear": [1.35, 1.32, 1.54],
                    "quadratic_damping_linear": [1.90, 2.00, 2.40],
                },
            ),
        ]
    if candidate_set == "physics-ls-current":
        return [
            Candidate("baseline"),
            Candidate(
                "ls_heave_yaw",
                {
                    "heave_damping_scale": 1.0,
                    "yaw_torque_scale": 1.25,
                },
            ),
            Candidate(
                "ls_heave_yaw_buoy",
                {
                    "buoyancy_scale": 1.012,
                    "heave_damping_scale": 1.0,
                    "yaw_torque_scale": 1.25,
                },
            ),
        ]
    return [
        Candidate("baseline"),
        Candidate(
            "heave_damping_m10",
            {"surface_heave_damping": 10.8, "heave_damping_scale": 3.6},
        ),
        Candidate(
            "heave_damping_p10",
            {"surface_heave_damping": 13.2, "heave_damping_scale": 4.4},
        ),
        Candidate(
            "fluid_linear_m10",
            fluid_linear_scale=0.90,
        ),
        Candidate(
            "fluid_linear_p10",
            fluid_linear_scale=1.10,
        ),
        Candidate(
            "fluid_rot_m10",
            fluid_rot_scale=0.90,
        ),
        Candidate(
            "fluid_rot_p10",
            fluid_rot_scale=1.10,
        ),
        Candidate(
            "restore_softer_m10",
            {"cob_torque_scale": 1.62, "cob_z_offset": 0.036},
        ),
        Candidate(
            "restore_stronger_p10",
            {"cob_torque_scale": 1.98, "cob_z_offset": 0.044},
        ),
        Candidate(
            "inertia_rp_m10",
            {"body_inertia_scale_xyz": [0.90, 0.90, 1.0]},
        ),
        Candidate(
            "inertia_rp_p10",
            {"body_inertia_scale_xyz": [1.10, 1.10, 1.0]},
        ),
    ]


def print_flush(text: str) -> None:
    print(text, flush=True)


def resolve_bag(path: Path) -> Path:
    path = path.expanduser()
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        direct = sorted(path.glob("*.db3"))
        if direct:
            return direct[0]
        nested = sorted(path.glob("**/*.db3"))
        if nested:
            return nested[0]
    raise FileNotFoundError(f"No .db3 bag found at {path}")


def load_profile(text: str) -> dict[str, Any]:
    return json.loads(text)


def write_profile(profile: dict[str, Any]) -> None:
    PROFILE_PATH.write_text(json.dumps(profile, indent=2, ensure_ascii=False) + "\n")


def _json_clone(value: Any) -> Any:
    return json.loads(json.dumps(value))


def _assign_profile_value(target: dict[str, Any], key: str, value: Any) -> None:
    """Assign profile update keys, with minimal dotted/list-index support."""
    parts = [part for part in str(key).split(".") if part]
    if not parts:
        return
    cursor: Any = target
    for part in parts[:-1]:
        if "[" in part and part.endswith("]"):
            name, index_text = part[:-1].split("[", 1)
            cursor = cursor.setdefault(name, [])
            cursor = cursor[int(index_text)]
        else:
            cursor = cursor.setdefault(part, {})
    leaf = parts[-1]
    if "[" in leaf and leaf.endswith("]"):
        name, index_text = leaf[:-1].split("[", 1)
        index = int(index_text)
        values = cursor.setdefault(name, [])
        while len(values) <= index:
            values.append(0.0)
        values[index] = _json_clone(value)
    else:
        cursor[leaf] = _json_clone(value)


def current_profile_with_candidate(original_profile_text: str, candidate: Candidate) -> dict[str, Any]:
    profile = load_profile(original_profile_text)
    current = _json_clone(profile.get("current", {}))
    if not isinstance(current, dict):
        current = {}
    for key, value in candidate.profile_updates.items():
        _assign_profile_value(current, key, value)
    return current


def apply_profile_candidate(original_profile_text: str, candidate: Candidate) -> None:
    profile = load_profile(original_profile_text)
    current = profile.setdefault("current", {})
    for key, value in candidate.profile_updates.items():
        _assign_profile_value(current, key, value)
    write_profile(profile)


def _replace_geom_fluidcoef(scene_text: str, geom_name: str, candidate: Candidate) -> str:
    pattern = re.compile(
        rf'(<geom\b(?:(?!/>).)*?name="{re.escape(geom_name)}"(?:(?!/>).)*?fluidcoef=")([^"]+)("(?:(?!/>).)*?/>)',
        re.DOTALL,
    )

    def repl(match: re.Match[str]) -> str:
        coeff = [float(part) for part in match.group(2).split()]
        if len(coeff) < 5:
            raise ValueError(f"{geom_name} fluidcoef must have at least 5 values")
        coeff[0] *= candidate.fluid_linear_scale * candidate.fluid_blunt_scale
        coeff[1] *= candidate.fluid_linear_scale * candidate.fluid_slender_scale
        coeff[2] *= candidate.fluid_rot_scale * candidate.fluid_angular_scale
        if candidate.fluid_kutta is not None:
            coeff[3] = float(candidate.fluid_kutta)
        if candidate.fluid_magnus is not None:
            coeff[4] = float(candidate.fluid_magnus)
        return match.group(1) + " ".join(f"{value:.3f}" for value in coeff) + match.group(3)

    updated, count = pattern.subn(repl, scene_text, count=1)
    if count != 1:
        raise ValueError(f"Could not locate geom {geom_name} in {SCENE_PATH}")
    return updated


def apply_scene_candidate(original_scene_text: str, candidate: Candidate) -> None:
    scene_text = original_scene_text
    for geom_name in FLUID_GEOMS:
        scene_text = _replace_geom_fluidcoef(
            scene_text,
            geom_name,
            candidate,
        )
    SCENE_PATH.write_text(scene_text)


def apply_candidate(original_profile_text: str, original_scene_text: str, candidate: Candidate) -> None:
    apply_profile_candidate(original_profile_text, candidate)
    apply_scene_candidate(original_scene_text, candidate)


def restore_original(original_profile_text: str, original_scene_text: str) -> None:
    PROFILE_PATH.write_text(original_profile_text)
    SCENE_PATH.write_text(original_scene_text)


def metric_score(summary: dict[str, Any], include_depth: bool) -> tuple[float, dict[str, Any]]:
    axes = [
        ("dvl_x", summary.get("dvl_velocity_metrics", {}).get("x", {}), 1.4),
        ("dvl_z", summary.get("dvl_velocity_metrics", {}).get("z", {}), 1.2),
        ("dvl_y", summary.get("dvl_velocity_metrics", {}).get("y", {}), 0.4),
        ("gyro_z", summary.get("imu_gyro_metrics", {}).get("z", {}), 1.6),
        ("gyro_x", summary.get("imu_gyro_metrics", {}).get("x", {}), 0.8),
        ("gyro_y", summary.get("imu_gyro_metrics", {}).get("y", {}), 0.8),
    ]
    if include_depth:
        axes.append(("depth", summary.get("depth_metrics", {}), 0.5))

    rows: dict[str, Any] = {}
    weighted = 0.0
    weight_sum = 0.0
    for name, metrics, weight in axes:
        count = int(metrics.get("count", 0) or 0)
        rmse = float(metrics.get("rmse", math.inf) or math.inf)
        real = metrics.get("real", {}) if isinstance(metrics.get("real"), dict) else {}
        sim = metrics.get("sim", {}) if isinstance(metrics.get("sim"), dict) else {}
        real_rms = float(real.get("rms", 0.0) or 0.0)
        sim_rms = float(sim.get("rms", 0.0) or 0.0)
        corr_raw = metrics.get("correlation")
        corr = 0.0 if corr_raw is None else float(corr_raw)
        if count <= 0 or not math.isfinite(rmse):
            normalized = 10.0
            corr_penalty = 1.0
            amp_penalty = 1.0
        else:
            normalized = rmse / max(real_rms, 1.0e-6)
            corr_penalty = 0.25 * (1.0 - max(-1.0, min(1.0, corr)))
            amp_ratio = sim_rms / max(real_rms, 1.0e-6)
            amp_penalty = 0.15 * abs(math.log(max(amp_ratio, 1.0e-6)))
        axis_score = normalized + corr_penalty + amp_penalty
        rows[name] = {
            "count": count,
            "rmse": rmse,
            "real_rms": real_rms,
            "sim_rms": sim_rms,
            "normalized_rmse": normalized,
            "correlation": corr_raw,
            "axis_score": axis_score,
        }
        weighted += weight * axis_score
        weight_sum += weight
    return weighted / max(weight_sum, 1.0e-9), rows


def run_process(cmd: list[str], log_path: Path) -> int:
    global CURRENT_PROC
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    with log_path.open("w") as log:
        proc = subprocess.Popen(
            cmd,
            cwd=str(ROOT_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
            start_new_session=True,
        )
        CURRENT_PROC = proc
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                print(line, end="", flush=True)
            return int(proc.wait())
        finally:
            CURRENT_PROC = None


def stop_current_process() -> None:
    proc = CURRENT_PROC
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass


def handle_signal(_signum, _frame) -> None:
    stop_current_process()
    raise KeyboardInterrupt


def crop_real_series(real: Any, start_s: float, duration_s: float) -> Any:
    start_s = max(0.0, float(start_s))
    end_s = start_s + max(0.0, float(duration_s))

    def crop_pair(t: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        t = np.asarray(t, dtype=float).reshape(-1)
        values = np.asarray(values)
        if t.size == 0 or values.size == 0:
            return t[:0], values[:0]
        n = min(t.shape[0], values.shape[0])
        t = t[:n]
        values = values[:n]
        mask = (t >= start_s) & (t <= end_s)
        return t[mask] - start_s, values[mask]

    rc_override_t, rc_override = crop_pair(real.rc_override_t, real.rc_override)
    rc_out_t, rc_out = crop_pair(real.rc_out_t, real.rc_out)
    joy_t, joy_axes = crop_pair(real.joy_t, real.joy_axes)
    _, joy_buttons = crop_pair(real.joy_t, real.joy_buttons)
    joy_rc_override_t, joy_rc_override = crop_pair(real.joy_rc_override_t, real.joy_rc_override)
    dvl_t, dvl_vel = crop_pair(real.dvl_t, real.dvl_vel)
    imu_t, imu_rpy = crop_pair(real.imu_t, real.imu_rpy)
    _, imu_gyro = crop_pair(real.imu_t, real.imu_gyro)
    _, imu_accel = crop_pair(real.imu_t, real.imu_accel)
    depth_t, depth = crop_pair(real.depth_t, real.depth)
    odom_t, odom_xyz = crop_pair(real.odom_t, real.odom_xyz)
    _, odom_rpy = crop_pair(real.odom_t, real.odom_rpy)
    _, odom_vel = crop_pair(real.odom_t, real.odom_vel)
    static_pressure_t, static_pressure = crop_pair(real.static_pressure_t, real.static_pressure)
    atm_pressure_t, atm_pressure = crop_pair(real.atm_pressure_t, real.atm_pressure)
    return replace(
        real,
        duration_s=max(0.0, end_s - start_s),
        rc_override_t=rc_override_t,
        rc_override=rc_override,
        rc_out_t=rc_out_t,
        rc_out=rc_out,
        joy_t=joy_t,
        joy_axes=joy_axes,
        joy_buttons=joy_buttons,
        joy_rc_override_t=joy_rc_override_t,
        joy_rc_override=joy_rc_override,
        dvl_t=dvl_t,
        dvl_vel=dvl_vel,
        imu_t=imu_t,
        imu_rpy=imu_rpy,
        imu_gyro=imu_gyro,
        imu_accel=imu_accel,
        depth_t=depth_t,
        depth=depth.reshape(-1) if depth.size else depth,
        odom_t=odom_t,
        odom_xyz=odom_xyz,
        odom_rpy=odom_rpy,
        odom_vel=odom_vel,
        static_pressure_t=static_pressure_t,
        static_pressure=static_pressure.reshape(-1) if static_pressure.size else static_pressure,
        atm_pressure_t=atm_pressure_t,
        atm_pressure=atm_pressure.reshape(-1) if atm_pressure.size else atm_pressure,
    )


def plant_command_source(tune_mode: str) -> str:
    if tune_mode == "plant-rc-override":
        return "rc_override"
    if tune_mode == "plant-joy":
        return "joy_node"
    return "rc_out"


def load_plant_replay_tools() -> tuple[Any, Any, Any, Any, Path]:
    os.environ.setdefault("MPLBACKEND", "Agg")
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    from replay_april1_real_commands_in_mujoco import (  # noqa: WPS433
        CURRENT_SCENE,
        OfflineUuvReplay,
        extract_real_series,
        plot_overlay,
        summarize_replay,
    )

    return extract_real_series, OfflineUuvReplay, summarize_replay, plot_overlay, CURRENT_SCENE


def plant_scene_for_candidate(original_scene_text: str, candidate: Candidate, candidate_dir: Path) -> Path:
    scene_text = original_scene_text
    for geom_name in FLUID_GEOMS:
        scene_text = _replace_geom_fluidcoef(
            scene_text,
            geom_name,
            candidate,
        )
    scene_dir = SCENE_PATH.parent

    def absolutize_meshdir(match: re.Match[str]) -> str:
        meshdir = match.group(2)
        mesh_path = Path(meshdir)
        if not mesh_path.is_absolute():
            mesh_path = (scene_dir / mesh_path).resolve()
        return match.group(1) + str(mesh_path) + match.group(3)

    scene_text = re.sub(r'(<compiler\b[^>]*\bmeshdir=")([^"]+)(")', absolutize_meshdir, scene_text, count=1)
    scene_path = candidate_dir / "scene_candidate.xml"
    scene_path.write_text(scene_text)
    return scene_path


def run_plant_candidate(
    args: argparse.Namespace,
    candidate: Candidate,
    out_root: Path,
    original_profile_text: str,
    original_scene_text: str,
) -> dict[str, Any]:
    candidate_dir = out_root / candidate.name
    comparison_dir = candidate_dir / "comparison"
    candidate_dir.mkdir(parents=True, exist_ok=True)
    comparison_dir.mkdir(parents=True, exist_ok=True)
    print_flush(f"[autotune] candidate {candidate.name}: start")
    print_flush(f"[autotune] candidate {candidate.name}: plant mode {args.tune_mode}")

    extract_real_series, OfflineUuvReplay, summarize_replay, plot_overlay, _current_scene = load_plant_replay_tools()
    real = extract_real_series(args.bag)
    real = crop_real_series(real, args.start_offset_s, args.duration_s)
    command_source = plant_command_source(args.tune_mode)
    scene_path = plant_scene_for_candidate(original_scene_text, candidate, candidate_dir)
    profile_overrides = current_profile_with_candidate(original_profile_text, candidate)
    cfg = {
        "name": candidate.name,
        "scene": scene_path,
        "profile": "current",
        "fluid_model": "current",
        "command_source": command_source,
        "thruster_dt_mode": "current-code",
        "profile_updates": candidate.profile_updates,
        "fluid_linear_scale": candidate.fluid_linear_scale,
        "fluid_rot_scale": candidate.fluid_rot_scale,
        "fluid_blunt_scale": candidate.fluid_blunt_scale,
        "fluid_slender_scale": candidate.fluid_slender_scale,
        "fluid_angular_scale": candidate.fluid_angular_scale,
        "fluid_kutta": candidate.fluid_kutta,
        "fluid_magnus": candidate.fluid_magnus,
    }

    replay = OfflineUuvReplay(
        scene=scene_path,
        profile_name="current",
        fluid_model="current",
        profile_overrides=profile_overrides,
        rc_out_scale=args.rc_out_scale,
    )

    sim = replay.run(
        real,
        command_source=command_source,
        max_duration_s=None,
        record_dt_s=float(args.record_dt_s),
    )
    np.savez_compressed(candidate_dir / "plant_timeseries.npz", **sim)
    summary = summarize_replay(real, sim, cfg)
    summary_path = comparison_dir / "closed_loop_real_vs_sim_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    plot_overlay(real, sim, f"{real.name} {candidate.name} {command_source}", comparison_dir / "closed_loop_real_vs_sim_overlay.png")

    score, axes = metric_score(summary, bool(args.include_depth))
    result: dict[str, Any] = {
        "candidate": candidate.name,
        "returncode": 0,
        "out_dir": str(candidate_dir),
        "profile_updates": candidate.profile_updates,
        "fluid_linear_scale": candidate.fluid_linear_scale,
        "fluid_rot_scale": candidate.fluid_rot_scale,
        "fluid_blunt_scale": candidate.fluid_blunt_scale,
        "fluid_slender_scale": candidate.fluid_slender_scale,
        "fluid_angular_scale": candidate.fluid_angular_scale,
        "fluid_kutta": candidate.fluid_kutta,
        "fluid_magnus": candidate.fluid_magnus,
        "tune_mode": args.tune_mode,
        "command_source": command_source,
        "score": score,
        "status": "ok",
        "axis_scores": axes,
        "summary_json": str(summary_path),
    }
    print_flush(f"[autotune] candidate {candidate.name}: score={score:.4f}")
    return result


def run_closed_loop_candidate(args: argparse.Namespace, candidate: Candidate, out_root: Path) -> dict[str, Any]:
    candidate_dir = out_root / candidate.name
    cmd = [
        str(RUN_REPLAY),
        "--bag",
        str(args.bag),
        "--out-dir",
        str(candidate_dir),
        "--start-offset-s",
        f"{args.start_offset_s:g}",
        "--duration-s",
        f"{args.duration_s:g}",
        "--mode-from-bag",
        "1",
        "--sitl-servo-scale",
        f"{args.sitl_servo_scale:g}",
    ]
    if args.initial_depth_m is not None:
        cmd.extend(["--initial-depth-m", f"{args.initial_depth_m:g}"])
    if args.calibration_depth_m is not None:
        cmd.extend(["--calibration-depth-m", f"{args.calibration_depth_m:g}", "--hold-initial-depth", "1"])

    print_flush(f"[autotune] candidate {candidate.name}: start")
    rc = run_process(cmd, candidate_dir / "autotune_candidate.log")
    result: dict[str, Any] = {
        "candidate": candidate.name,
        "returncode": rc,
        "out_dir": str(candidate_dir),
        "profile_updates": candidate.profile_updates,
        "fluid_linear_scale": candidate.fluid_linear_scale,
        "fluid_rot_scale": candidate.fluid_rot_scale,
        "fluid_blunt_scale": candidate.fluid_blunt_scale,
        "fluid_slender_scale": candidate.fluid_slender_scale,
        "fluid_angular_scale": candidate.fluid_angular_scale,
        "fluid_kutta": candidate.fluid_kutta,
        "fluid_magnus": candidate.fluid_magnus,
        "tune_mode": args.tune_mode,
    }
    summary_path = candidate_dir / "comparison" / "closed_loop_real_vs_sim_summary.json"
    if rc != 0 or not summary_path.exists():
        result.update({"score": math.inf, "status": "failed"})
        print_flush(f"[autotune] candidate {candidate.name}: failed rc={rc}")
        return result

    summary = json.loads(summary_path.read_text())
    score, axes = metric_score(summary, bool(args.include_depth))
    result.update(
        {
            "score": score,
            "status": "ok",
            "axis_scores": axes,
            "summary_json": str(summary_path),
        }
    )
    print_flush(f"[autotune] candidate {candidate.name}: score={score:.4f}")
    return result


def write_outputs(out_root: Path, results: list[dict[str, Any]], best: dict[str, Any] | None, args: argparse.Namespace) -> None:
    payload = {
        "best": best,
        "results": results,
        "tune_mode": args.tune_mode,
        "candidate_set": args.candidate_set,
        "score_definition": (
            "Weighted normalized RMSE plus correlation/amplitude penalties over "
            "DVL x/y/z and IMU gyro x/y/z. Lower is better."
        ),
    }
    (out_root / "autotune_summary.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    with (out_root / "autotune_scores.csv").open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "candidate",
                "status",
                "score",
                "fluid_linear_scale",
                "fluid_rot_scale",
                "fluid_blunt_scale",
                "fluid_slender_scale",
                "fluid_angular_scale",
                "fluid_kutta",
                "fluid_magnus",
                "profile_updates",
                "tune_mode",
                "command_source",
                "out_dir",
            ],
        )
        writer.writeheader()
        for row in sorted(results, key=lambda item: float(item.get("score", math.inf))):
            writer.writerow(
                {
                    "candidate": row.get("candidate"),
                    "status": row.get("status"),
                    "score": row.get("score"),
                    "fluid_linear_scale": row.get("fluid_linear_scale"),
                    "fluid_rot_scale": row.get("fluid_rot_scale"),
                    "fluid_blunt_scale": row.get("fluid_blunt_scale"),
                    "fluid_slender_scale": row.get("fluid_slender_scale"),
                    "fluid_angular_scale": row.get("fluid_angular_scale"),
                    "fluid_kutta": row.get("fluid_kutta"),
                    "fluid_magnus": row.get("fluid_magnus"),
                    "profile_updates": json.dumps(row.get("profile_updates", {}), ensure_ascii=False),
                    "tune_mode": row.get("tune_mode"),
                    "command_source": row.get("command_source"),
                    "out_dir": row.get("out_dir"),
                }
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--start-offset-s", type=float, default=60.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    # Legacy polynomial/gain tuned mode used default=0.58.
    parser.add_argument("--sitl-servo-scale", type=float, default=1.0)
    parser.add_argument("--max-candidates", type=int, default=9)
    parser.add_argument("--candidate", action="append", default=[], help="Candidate name to run; repeatable.")
    parser.add_argument(
        "--candidate-set",
        choices=(
            "micro",
            "fluid-focus",
            "ellipsoid5",
            "ellipsoid5-refine",
            "ellipsoid5-final",
            "rosbag-yaw",
            "closed-loop-refine",
            "lift-refine",
            "broad",
            "rosbag-axis",
            "physics-ls-current",
        ),
        default="micro",
        help="micro uses small axis-separated perturbations; ellipsoid5 tunes the five MuJoCo fluidcoef terms.",
    )
    parser.add_argument(
        "--tune-mode",
        choices=("plant-rc-out", "plant-rc-override", "plant-joy", "closed-loop"),
        default="plant-rc-out",
        help="plant-* replays commands directly through MuJoCo; closed-loop replays /rc/override through SITL.",
    )
    parser.add_argument("--record-dt-s", type=float, default=0.02)
    parser.add_argument("--rc-out-scale", type=float, default=1.0)
    parser.add_argument("--include-depth", action="store_true")
    parser.add_argument("--apply-best", action="store_true")
    parser.add_argument("--initial-depth-m", type=float)
    parser.add_argument("--calibration-depth-m", type=float)
    return parser.parse_args()


def main() -> int:
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    args = parse_args()
    args.bag = resolve_bag(args.bag)
    out_root = args.out_root.expanduser()
    out_root.mkdir(parents=True, exist_ok=True)

    original_profile_text = PROFILE_PATH.read_text()
    original_scene_text = SCENE_PATH.read_text()
    candidates = default_candidates(args.candidate_set)
    if args.candidate:
        wanted = set(args.candidate)
        candidates = [candidate for candidate in candidates if candidate.name in wanted]
    if args.max_candidates > 0:
        candidates = candidates[: args.max_candidates]
    if not candidates:
        raise RuntimeError("No candidates selected")

    print_flush(f"[autotune] bag={args.bag}")
    print_flush(f"[autotune] out={out_root}")
    print_flush(f"[autotune] tune_mode={args.tune_mode}")
    print_flush(f"[autotune] candidate_set={args.candidate_set}")
    print_flush(f"[autotune] candidates={', '.join(candidate.name for candidate in candidates)}")

    results: list[dict[str, Any]] = []
    best: dict[str, Any] | None = None
    try:
        for candidate in candidates:
            if args.tune_mode == "closed-loop":
                apply_candidate(original_profile_text, original_scene_text, candidate)
                result = run_closed_loop_candidate(args, candidate, out_root)
            else:
                result = run_plant_candidate(args, candidate, out_root, original_profile_text, original_scene_text)
            results.append(result)
            if result.get("status") == "ok" and (
                best is None or float(result["score"]) < float(best.get("score", math.inf))
            ):
                best = result
            write_outputs(out_root, results, best, args)
    except KeyboardInterrupt:
        print_flush("[autotune] interrupted")
        return 130
    finally:
        restore_original(original_profile_text, original_scene_text)

    if best is not None and args.apply_best:
        candidate = next(candidate for candidate in candidates if candidate.name == best["candidate"])
        apply_candidate(original_profile_text, original_scene_text, candidate)
        print_flush(f"[autotune] applied best candidate: {candidate.name}")
    elif best is not None:
        print_flush(f"[autotune] best={best['candidate']} score={float(best['score']):.4f}")
        print_flush("[autotune] original MuJoCo parameters restored; use --apply-best to keep the winner")
    else:
        print_flush("[autotune] no successful candidate")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
