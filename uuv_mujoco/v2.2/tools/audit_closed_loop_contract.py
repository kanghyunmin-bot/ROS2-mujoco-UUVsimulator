#!/usr/bin/env python3
"""Emit the closed-loop validation contract used by SITL/MuJoCo replay runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WATCH_PARAMS = (
    "FRAME_CONFIG",
    "MOT_1_DIRECTION",
    "MOT_2_DIRECTION",
    "MOT_3_DIRECTION",
    "MOT_4_DIRECTION",
    "MOT_5_DIRECTION",
    "MOT_6_DIRECTION",
    "MOT_7_DIRECTION",
    "MOT_8_DIRECTION",
    "MOT_PWM_MIN",
    "MOT_PWM_MAX",
    "MOT_SPIN_ARM",
    "MOT_SPIN_MIN",
    "MOT_SPIN_MAX",
    "MOT_SPOOL_TIME",
    "MOT_THST_EXPO",
    "MOT_THST_HOVER",
    "MOT_YAW_HEADROOM",
    "RC1_MIN",
    "RC1_MAX",
    "RC1_TRIM",
    "RC1_DZ",
    "RC1_REVERSED",
    "RC2_MIN",
    "RC2_MAX",
    "RC2_TRIM",
    "RC2_DZ",
    "RC2_REVERSED",
    "RC3_MIN",
    "RC3_MAX",
    "RC3_TRIM",
    "RC3_DZ",
    "RC3_REVERSED",
    "RC4_MIN",
    "RC4_MAX",
    "RC4_TRIM",
    "RC4_DZ",
    "RC4_REVERSED",
    "RC5_MIN",
    "RC5_MAX",
    "RC5_TRIM",
    "RC5_DZ",
    "RC5_REVERSED",
    "RC6_MIN",
    "RC6_MAX",
    "RC6_TRIM",
    "RC6_DZ",
    "RC6_REVERSED",
    "RC_OPTIONS",
    "RC_OVERRIDE_TIME",
    "THR_DZ",
    "JS_GAIN_DEFAULT",
    "JS_GAIN_MAX",
    "JS_GAIN_MIN",
    "JS_GAIN_STEPS",
    "JS_THR_GAIN",
    "FS_PILOT_INPUT",
    "FS_PILOT_TIMEOUT",
    "PILOT_SPEED_UP",
    "PILOT_SPEED_DN",
    "PILOT_ACCEL_Z",
    "ATC_SLEW_YAW",
    "PSC_POSZ_P",
    "PSC_VELZ_P",
    "PSC_ACCZ_P",
    "PSC_ACCZ_I",
    "PSC_ACCZ_D",
    "BARO_PRIMARY",
    "BARO_SPEC_GRAV",
    "SURFACE_DEPTH",
    "AHRS_EKF_TYPE",
    "EK3_GBIAS_P_NSE",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "EK3_SRC2_YAW",
    "EK3_SRC_OPTIONS",
    "EK3_YAW_M_NSE",
    "INS_POS1_X",
    "INS_POS1_Y",
    "INS_POS1_Z",
    "VISO_TYPE",
    "VISO_DELAY_MS",
    "VISO_POS_X",
    "VISO_POS_Y",
    "VISO_POS_Z",
    "VISO_POS_M_NSE",
    "VISO_VEL_M_NSE",
    "VISO_YAW_M_NSE",
)

PROFILE_KEYS = (
    "buoyancy_scale",
    "surface_heave_damping",
    "heave_damping_scale",
    "buoyancy_slope_scale",
    "cob_x_offset",
    "cob_z_offset",
    "cob_torque_scale",
    "body_inertia_scale_xyz",
    "yaw_torque_scale",
    "thruster_voltage",
    "thruster_force_max",
    "mujoco_fluidcoef_scale",
    "mujoco_fluidcoef_geom_scales",
)

CURRENT_INACTIVE_KEYS = (
    "thruster_force_max",
    "linear_drag",
    "angular_drag",
    "ellipsoid_model.effective_cd_linear",
    "ellipsoid_model.effective_cd_angular",
    "linear_damping_linear",
    "linear_damping_angular",
    "quadratic_damping_linear",
    "quadratic_damping_angular",
    "ellipsoid_model.added_mass_scale_linear",
    "ellipsoid_model.added_mass_scale_angular",
    "ellipsoid_model.linear_damping_ratio_linear",
    "ellipsoid_model.linear_damping_ratio_angular",
    "ellipsoid_model.reference_speed_linear",
    "ellipsoid_model.reference_speed_angular",
)


def parse_param_file(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    if not path.exists():
        return params
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        line = line.replace(",", " ")
        parts = [part for part in line.split() if part]
        if len(parts) < 2:
            continue
        params[parts[0]] = parts[1]
    return params


def parse_start_sitl_enforced_params(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    if not path.exists():
        return params
    in_block = False
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "[start-sitl] enforcing params via" in raw_line:
            in_block = True
            continue
        if not in_block:
            continue
        if not raw_line.startswith("  "):
            break
        line = raw_line.strip()
        parts = [part for part in line.split() if part]
        if len(parts) >= 2:
            params[parts[0]] = parts[1]
    return params


def same_param_value(left: str | None, right: str | None) -> bool:
    if left is None or right is None:
        return left == right
    try:
        return abs(float(left) - float(right)) <= 1.0e-9
    except ValueError:
        return left == right


def nested_get(mapping: dict[str, Any], dotted_key: str) -> Any:
    current: Any = mapping
    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def load_profile(path: Path, name: str) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    profiles = payload.get("profiles")
    if isinstance(profiles, dict) and isinstance(profiles.get(name), dict):
        return profiles[name]
    profile = payload.get(name)
    if isinstance(profile, dict):
        return profile
    raise KeyError(f"profile '{name}' not found in {path}")


def selected_thruster_curve(path: Path, requested_voltage: float) -> dict[str, Any]:
    if not path.exists():
        return {"active": False, "reason": f"missing file: {path}"}
    payload = json.loads(path.read_text(encoding="utf-8"))
    curves = payload.get("curves", {})
    candidates: list[tuple[float, dict[str, Any]]] = []
    if isinstance(curves, dict):
        for key, curve in curves.items():
            try:
                voltage = float(key)
            except ValueError:
                voltage = float(curve.get("voltage", "nan")) if isinstance(curve, dict) else float("nan")
            if voltage == voltage and isinstance(curve, dict):
                candidates.append((voltage, curve))
    elif isinstance(curves, list):
        for curve in curves:
            if not isinstance(curve, dict):
                continue
            try:
                voltage = float(curve.get("voltage_v", curve.get("voltage")))
            except (TypeError, ValueError):
                continue
            candidates.append((voltage, curve))
    if not candidates:
        return {"active": False, "reason": "no usable curves"}
    voltage, curve = min(candidates, key=lambda item: abs(item[0] - requested_voltage))
    force_values: list[float] = []
    if isinstance(curve.get("force_n"), list):
        force_values.extend(float(value) for value in curve.get("force_n", []))
    for row in curve.get("samples", []):
        if isinstance(row, dict) and "force_n" in row:
            force_values.append(float(row["force_n"]))
        elif isinstance(row, (list, tuple)) and len(row) >= 2:
            force_values.append(float(row[-1]))
    return {
        "active": bool(force_values),
        "requested_voltage": requested_voltage,
        "selected_voltage": voltage,
        "force_min_n": min(force_values) if force_values else None,
        "force_max_n": max(force_values) if force_values else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--profile", default="current")
    parser.add_argument(
        "--sitl-log",
        type=Path,
        default=None,
        help="Optional start_ardusub_sitl log; used to capture temp enforced params.",
    )
    parser.add_argument("--json-out", type=Path, required=True)
    args = parser.parse_args()

    workspace = args.workspace
    sim_dir = workspace / "uuv_mujoco" / "v2.2"
    real_params = parse_param_file(workspace / "real_robot.param")
    sitl_param_sources: list[str] = []
    sitl_defaults_path = workspace / "ardupilot" / "Tools" / "autotest" / "default_params" / "sub-6dof.parm"
    sitl_params = parse_param_file(sitl_defaults_path)
    if sitl_defaults_path.exists():
        sitl_param_sources.append(str(sitl_defaults_path))
    mav_param_path = workspace / "ardupilot" / "mav.parm"
    mav_params = parse_param_file(mav_param_path)
    if mav_params:
        sitl_params.update(mav_params)
        sitl_param_sources.append(str(mav_param_path))
    if args.sitl_log is not None:
        logged_params = parse_start_sitl_enforced_params(args.sitl_log)
        if logged_params:
            sitl_params.update(logged_params)
            sitl_param_sources.append(str(args.sitl_log))
    profile = load_profile(sim_dir / "config" / "sim_profiles.json", args.profile)
    requested_voltage = float(profile.get("thruster_voltage", 22.2))
    curve = selected_thruster_curve(sim_dir / "config" / "thruster_performance.json", requested_voltage)

    watched_real = {key: real_params.get(key) for key in WATCH_PARAMS}
    watched_sitl = {key: sitl_params.get(key) for key in WATCH_PARAMS}
    mismatches = {
        key: {"real": watched_real.get(key), "sitl": watched_sitl.get(key)}
        for key in WATCH_PARAMS
        if watched_real.get(key) is not None
        and watched_sitl.get(key) is not None
        and not same_param_value(watched_real.get(key), watched_sitl.get(key))
    }
    missing_sitl = [
        key
        for key in WATCH_PARAMS
        if watched_real.get(key) is not None and watched_sitl.get(key) is None
    ]
    current_profile = {key: profile.get(key) for key in PROFILE_KEYS}
    active_keys = [
        key
        for key in PROFILE_KEYS
        if key not in CURRENT_INACTIVE_KEYS and profile.get(key) is not None
    ]

    payload = {
        "workspace": str(workspace),
        "profile": args.profile,
        "real_robot_param": str(workspace / "real_robot.param"),
        "sitl_param": str(mav_param_path),
        "sitl_param_sources": sitl_param_sources,
        "watched_real_params": watched_real,
        "watched_sitl_params": watched_sitl,
        "real_vs_sitl_mismatches": mismatches,
        "missing_sitl_params": missing_sitl,
        "current_mode": {
            "active_profile_keys": active_keys,
            "inactive_profile_keys": list(CURRENT_INACTIVE_KEYS),
            "profile_values": current_profile,
        },
        "thruster_performance_curve": curve,
        "notes": [
            "Current MuJoCo mode uses built-in geom fluidcoef plus hydrostatic/CoB terms.",
            "Legacy/custom added_mass, linear_damping, and quadratic_damping are inactive in current mode.",
            "When the T200 performance curve is active, profile thruster_force_max is not the actuator force limit.",
            "real_vs_sitl_mismatches only compares parameters observed in both sources; missing_sitl_params must be resolved before claiming full QGC/SITL parity.",
        ],
    }
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[contract-audit] wrote {args.json_out}")
    if mismatches:
        print(f"[contract-audit] real/SITL watched param mismatches: {len(mismatches)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
