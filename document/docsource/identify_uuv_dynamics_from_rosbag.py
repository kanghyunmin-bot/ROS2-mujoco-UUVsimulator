#!/usr/bin/env python3
"""Estimate UUV hydrodynamic coefficients from a real ROS2 bag.

This is an inverse-dynamics identification tool, not a closed-loop replay.
It constructs one linear least-squares problem per 6-DOF body axis:

    M_rb * nu_dot - tau_thruster = Y(nu, nu_dot, eta) * theta

The fitted parameters are intended to diagnose whether the current MuJoCo
profile is over/under damping or over/under restoring.  It deliberately keeps
nonlinear geometry parameters outside the solve; use those as an outer-loop
search after this linear fit is stable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
UUV_DIR = REPO_ROOT / "uuv_mujoco" / "v2.2"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(UUV_DIR) not in sys.path:
    sys.path.insert(0, str(UUV_DIR))

from replay_april1_real_commands_in_mujoco import (  # noqa: E402
    CURRENT_SCENE,
    DEFAULT_ROOT,
    OfflineUuvReplay,
    RealSeries,
    THRUSTER_PARAMS_PATH,
    euler_to_quat_wxyz,
    extract_real_series,
    normalize_rc_out,
    normalize_rc_override,
    resample_matrix,
)
from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    PHYSICAL_VERTICAL_THRUSTERS,
    PHYSICAL_YAW_THRUSTERS,
)


DEFAULT_REAL_BAG = (
    DEFAULT_ROOT
    / "bag_2026-04-01_20-08-11"
    / "bag_2026-04-01_20-08-11_0.db3"
)
DEFAULT_OUT = Path("document/docsource/inverse_dynamics_identification")

AXIS_NAMES = ("surge_x", "sway_y", "heave_z", "roll_k", "pitch_m", "yaw_n")
AXIS_SHORT = ("x", "y", "z", "roll", "pitch", "yaw")


def resolve_db3(path: Path) -> Path:
    path = Path(path).expanduser()
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.db3")) or sorted(path.glob("**/*.db3"))
        if candidates:
            return candidates[0]
    raise FileNotFoundError(f"No .db3 file found at {path}")


def parse_vec3(text: str, *, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    parts = [item.strip() for item in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected comma-separated triple, e.g. 1,-1,1")
    try:
        out = np.asarray([float(item) for item in parts], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    if not np.all(np.isfinite(out)):
        raise argparse.ArgumentTypeError("all signs/scales must be finite")
    return out


def moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.size == 0 or samples <= 1:
        return values.copy()
    samples = min(samples, max(1, values.shape[0] // 2))
    if values.ndim == 1:
        padded = np.pad(values, (samples // 2, samples - 1 - samples // 2), mode="edge")
        kernel = np.ones(samples, dtype=float) / float(samples)
        return np.convolve(padded, kernel, mode="valid")
    return np.column_stack([moving_average(values[:, idx], samples) for idx in range(values.shape[1])])


def unwrap_rpy(rpy: np.ndarray) -> np.ndarray:
    rpy = np.asarray(rpy, dtype=float)
    if rpy.size == 0:
        return rpy.reshape((0, 3))
    out = rpy.reshape((-1, 3)).copy()
    out[:, 2] = np.unwrap(out[:, 2])
    return out


def derivative(t: np.ndarray, y: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float)
    if t.size < 3 or y.size == 0:
        return np.zeros_like(y, dtype=float)
    dt = np.gradient(t)
    dt = np.where(np.abs(dt) < 1e-6, np.nan, dt)
    if y.ndim == 1:
        return np.nan_to_num(np.gradient(y) / dt)
    return np.column_stack([np.nan_to_num(np.gradient(y[:, idx]) / dt) for idx in range(y.shape[1])])


def regular_grid(real: RealSeries, start_s: float, duration_s: float | None, dt_s: float) -> np.ndarray:
    starts = []
    ends = []
    for t in (real.dvl_t, real.imu_t, real.rc_out_t):
        if t.size:
            starts.append(float(t[0]))
            ends.append(float(t[-1]))
    if not starts:
        raise RuntimeError("bag has no usable DVL/IMU/RCOut time series")
    t0 = max(float(start_s), min(starts))
    t1_data = min(ends)
    t1 = t1_data if duration_s is None else min(t1_data, t0 + float(duration_s))
    if t1 <= t0 + 2.0 * dt_s:
        raise RuntimeError(f"identification window too short: start={t0:.3f}, end={t1:.3f}")
    return np.arange(t0, t1 + 0.5 * dt_s, float(dt_s), dtype=float)


def interp_matrix(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray, dims: int) -> np.ndarray:
    src_t = np.asarray(src_t, dtype=float).reshape(-1)
    src_v = np.asarray(src_v, dtype=float)
    if src_t.size == 0 or src_v.size == 0:
        return np.full((dst_t.size, dims), np.nan, dtype=float)
    src_v = src_v.reshape((-1, dims))
    count = min(src_t.size, src_v.shape[0])
    src_t = src_t[:count]
    src_v = src_v[:count]
    out = np.column_stack([np.interp(dst_t, src_t, src_v[:, idx]) for idx in range(dims)])
    outside = (dst_t < src_t[0]) | (dst_t > src_t[-1])
    out[outside, :] = np.nan
    return out


def interp_scalar(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    values = interp_matrix(src_t, np.asarray(src_v).reshape(-1, 1), dst_t, 1)
    return values[:, 0]


@dataclass
class ThrusterWrenchSeries:
    wrench_body: np.ndarray
    command8_norm: np.ndarray
    thruster_forces: dict[str, np.ndarray]


def compute_thruster_wrench(
    replay: OfflineUuvReplay,
    real: RealSeries,
    t: np.ndarray,
    rpy: np.ndarray,
    *,
    command_source: str,
) -> ThrusterWrenchSeries:
    wrench = np.zeros((t.size, 6), dtype=float)
    command8 = np.zeros((t.size, 8), dtype=float)
    forces = {name: np.zeros(t.size, dtype=float) for name in replay.all_thruster_names}

    rc_out_norm = normalize_rc_out(real.rc_out) if real.rc_out.size else np.empty((0, 8), dtype=float)
    rc_override_norm = normalize_rc_override(real.rc_override) if real.rc_override.size else np.empty((0, 8), dtype=float)
    joy_override_norm = normalize_rc_override(real.joy_rc_override) if real.joy_rc_override.size else np.empty((0, 8), dtype=float)

    replay.data.qpos[:3] = 0.0
    replay.data.qvel[:] = 0.0
    replay.next_thruster_sim_time = -1.0
    for name in replay.all_thruster_names:
        replay.thr_state[name] = 0.0
        replay.thr_target[name] = 0.0
        replay.thruster_force_cmd[name] = 0.0

    prev_t = float(t[0])
    com_body = replay.model.body_ipos[replay.base_id].copy()
    site_ids = {
        name: mujoco.mj_name2id(replay.model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
        for name in replay.all_thruster_names
    }

    for i, ti in enumerate(t):
        dt = max(float(ti - prev_t), 1.0e-3)
        prev_t = float(ti)
        replay.data.qpos[3:7] = euler_to_quat_wxyz(float(rpy[i, 0]), float(rpy[i, 1]), float(rpy[i, 2]))
        mujoco.mj_forward(replay.model, replay.data)

        if command_source == "rc_out" and rc_out_norm.size:
            cmd = resample_matrix(real.rc_out_t, rc_out_norm, np.array([ti], dtype=float))[0]
            replay.apply_rc_out_targets(cmd)
        elif command_source == "joy_node" and joy_override_norm.size:
            cmd = resample_matrix(real.joy_rc_override_t, joy_override_norm, np.array([ti], dtype=float))[0]
            direct = np.array([cmd[4], cmd[5], -cmd[3], -cmd[2]], dtype=float)
            replay.apply_direct_command_targets(direct)
        elif rc_override_norm.size:
            cmd = resample_matrix(real.rc_override_t, rc_override_norm, np.array([ti], dtype=float))[0]
            direct = np.array([cmd[4], cmd[5], -cmd[3], -cmd[2]], dtype=float)
            replay.apply_direct_command_targets(direct)
        else:
            cmd = np.zeros(8, dtype=float)
            for name in replay.all_thruster_names:
                replay.thr_target[name] = 0.0

        command8[i, :] = cmd[:8]
        replay.update_thruster_forces(dt)

        f_total = np.zeros(3, dtype=float)
        tau_total = np.zeros(3, dtype=float)
        for name in replay.all_thruster_names:
            aid = replay.act[name]
            sid = site_ids[name]
            fdir = replay.normalize(replay.model.actuator_gear[aid, :3].copy())
            force = float(replay.data.ctrl[aid])
            f_body = fdir * force
            f_total += f_body
            if sid >= 0:
                r_body = replay.model.site_pos[sid].copy() - com_body
                tau_body = np.cross(r_body, f_body)
                if replay.yaw_torque_scale > 1.0 and name in PHYSICAL_YAW_THRUSTERS:
                    tau_body[2] *= replay.yaw_torque_scale
                tau_total += tau_body
            forces[name][i] = force
        wrench[i, :3] = f_total
        wrench[i, 3:] = tau_total

    return ThrusterWrenchSeries(wrench_body=wrench, command8_norm=command8, thruster_forces=forces)


def compute_mujoco_fluid_wrench(
    replay: OfflineUuvReplay,
    nu_body: np.ndarray,
    rpy: np.ndarray,
    *,
    sample_depth_m: float = 0.45,
) -> np.ndarray:
    """Sample MuJoCo's built-in ellipsoid qfrc_fluid along the bag velocities."""
    nu_body = np.asarray(nu_body, dtype=float).reshape((-1, 6))
    rpy = np.asarray(rpy, dtype=float).reshape((-1, 3))
    count = min(nu_body.shape[0], rpy.shape[0])
    out = np.zeros((count, 6), dtype=float)
    data = replay.data
    model = replay.model
    for idx in range(count):
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[:3] = np.array([0.0, 0.0, -abs(float(sample_depth_m))], dtype=float)
        data.qpos[3:7] = euler_to_quat_wxyz(float(rpy[idx, 0]), float(rpy[idx, 1]), float(rpy[idx, 2]))
        data.qvel[:3] = nu_body[idx, :3]
        data.qvel[3:6] = nu_body[idx, 3:]
        mujoco.mj_forward(model, data)
        out[idx, :] = data.qfrc_fluid[:6]
    return out


def fit_linear_model(
    t: np.ndarray,
    b: np.ndarray,
    columns: dict[str, np.ndarray],
    *,
    ridge: float,
    min_column_rms: float = 1.0e-8,
) -> dict[str, Any]:
    names = list(columns)
    A = np.column_stack([np.asarray(columns[name], dtype=float).reshape(-1) for name in names])
    b = np.asarray(b, dtype=float).reshape(-1)
    mask = np.isfinite(b)
    for idx in range(A.shape[1]):
        mask &= np.isfinite(A[:, idx])
    A = A[mask]
    b = b[mask]
    tt = t[mask]
    if A.shape[0] < max(12, A.shape[1] + 4):
        return {"count": int(A.shape[0]), "status": "insufficient_data", "coefficients": {}}

    rms = np.sqrt(np.mean(A * A, axis=0))
    keep = rms > float(min_column_rms)
    if not np.any(keep):
        return {"count": int(A.shape[0]), "status": "no_excited_columns", "coefficients": {}}
    A_use = A[:, keep]
    names_use = [name for name, ok in zip(names, keep) if ok]
    scale = np.sqrt(np.mean(A_use * A_use, axis=0))
    scale = np.where(scale < 1.0e-12, 1.0, scale)
    As = A_use / scale
    if ridge > 0.0:
        lhs = As.T @ As + float(ridge) * np.eye(As.shape[1])
        rhs = As.T @ b
        coef_scaled = np.linalg.solve(lhs, rhs)
    else:
        coef_scaled, *_ = np.linalg.lstsq(As, b, rcond=None)
    coef = coef_scaled / scale
    pred = A_use @ coef
    residual = b - pred
    ss_res = float(np.sum(residual * residual))
    ss_tot = float(np.sum((b - np.mean(b)) ** 2))
    singular = np.linalg.svd(As, compute_uv=False)
    coeffs = {name: float(value) for name, value in zip(names_use, coef)}
    for name, ok in zip(names, keep):
        if not ok:
            coeffs[name] = None
    return {
        "count": int(A.shape[0]),
        "status": "ok",
        "coefficients": coeffs,
        "rmse": float(np.sqrt(np.mean(residual * residual))),
        "mae": float(np.mean(np.abs(residual))),
        "bias": float(np.mean(residual)),
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1.0e-12 else None,
        "target_rms": float(np.sqrt(np.mean(b * b))),
        "target_std": float(np.std(b)),
        "condition_number_scaled": float(singular[0] / max(singular[-1], 1.0e-12)),
        "time_start_s": float(tt[0]),
        "time_end_s": float(tt[-1]),
        "prediction": pred,
        "target": b,
        "residual": residual,
        "fit_time": tt,
        "kept_columns": names_use,
    }


def fit_bounded_linear_model(
    t: np.ndarray,
    b: np.ndarray,
    columns: dict[str, np.ndarray],
    *,
    ridge: float,
    lower_bounds: dict[str, float],
    min_column_rms: float = 1.0e-8,
) -> dict[str, Any]:
    """Small bounded least-squares solver by active-set enumeration.

    The per-axis UUV problems have only a few coefficients, so exhaustive
    active-set enumeration is more predictable than adding a scipy dependency.
    Bounds are currently lower-only, which is enough to prevent nonphysical
    negative damping/added-mass/restoring terms while leaving bias terms free.
    """
    names = list(columns)
    A = np.column_stack([np.asarray(columns[name], dtype=float).reshape(-1) for name in names])
    b = np.asarray(b, dtype=float).reshape(-1)
    mask = np.isfinite(b)
    for idx in range(A.shape[1]):
        mask &= np.isfinite(A[:, idx])
    A = A[mask]
    b = b[mask]
    tt = t[mask]
    if A.shape[0] < max(12, A.shape[1] + 4):
        return {"count": int(A.shape[0]), "status": "insufficient_data", "coefficients": {}}

    rms = np.sqrt(np.mean(A * A, axis=0))
    keep = rms > float(min_column_rms)
    if not np.any(keep):
        return {"count": int(A.shape[0]), "status": "no_excited_columns", "coefficients": {}}
    A_use = A[:, keep]
    names_use = [name for name, ok in zip(names, keep) if ok]
    scale = np.sqrt(np.mean(A_use * A_use, axis=0))
    scale = np.where(scale < 1.0e-12, 1.0, scale)
    As = A_use / scale
    lower = np.array([float(lower_bounds.get(name, -np.inf)) for name in names_use], dtype=float)
    lower_scaled = np.where(np.isfinite(lower), lower * scale, -np.inf)
    bounded_indices = [idx for idx, value in enumerate(lower_scaled) if np.isfinite(value)]

    if len(bounded_indices) > 14:
        return {"count": int(A.shape[0]), "status": "too_many_bounded_columns", "coefficients": {}}

    best: dict[str, Any] | None = None
    total_masks = 1 << len(bounded_indices)
    for active_bits in range(total_masks):
        fixed = np.zeros(As.shape[1], dtype=bool)
        y = np.zeros(As.shape[1], dtype=float)
        for bit_idx, col_idx in enumerate(bounded_indices):
            if active_bits & (1 << bit_idx):
                fixed[col_idx] = True
                y[col_idx] = lower_scaled[col_idx]
        free = ~fixed
        rhs = b - As[:, fixed] @ y[fixed] if np.any(fixed) else b
        if np.any(free):
            Af = As[:, free]
            if ridge > 0.0:
                lhs = Af.T @ Af + float(ridge) * np.eye(Af.shape[1])
                sol = np.linalg.solve(lhs, Af.T @ rhs)
            else:
                sol, *_ = np.linalg.lstsq(Af, rhs, rcond=None)
            y[free] = sol
        for col_idx in bounded_indices:
            if y[col_idx] < lower_scaled[col_idx] - 1.0e-8:
                break
        else:
            pred = As @ y
            residual = b - pred
            ss_res = float(np.sum(residual * residual))
            if best is None or ss_res < best["ss_res"]:
                best = {
                    "ss_res": ss_res,
                    "y": y.copy(),
                    "pred": pred.copy(),
                    "residual": residual.copy(),
                    "active_lower_bounds": [
                        names_use[idx]
                        for idx in bounded_indices
                        if abs(y[idx] - lower_scaled[idx]) <= 1.0e-7 * max(1.0, abs(lower_scaled[idx]))
                    ],
                }

    if best is None:
        return {"count": int(A.shape[0]), "status": "no_feasible_solution", "coefficients": {}}

    coef = best["y"] / scale
    pred = best["pred"]
    residual = best["residual"]
    ss_res = best["ss_res"]
    ss_tot = float(np.sum((b - np.mean(b)) ** 2))
    singular = np.linalg.svd(As, compute_uv=False)
    coeffs = {name: float(value) for name, value in zip(names_use, coef)}
    for name, ok in zip(names, keep):
        if not ok:
            coeffs[name] = None
    return {
        "count": int(A.shape[0]),
        "status": "ok",
        "coefficients": coeffs,
        "rmse": float(np.sqrt(np.mean(residual * residual))),
        "mae": float(np.mean(np.abs(residual))),
        "bias": float(np.mean(residual)),
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1.0e-12 else None,
        "target_rms": float(np.sqrt(np.mean(b * b))),
        "target_std": float(np.std(b)),
        "condition_number_scaled": float(singular[0] / max(singular[-1], 1.0e-12)),
        "time_start_s": float(tt[0]),
        "time_end_s": float(tt[-1]),
        "active_lower_bounds": best["active_lower_bounds"],
        "kept_columns": names_use,
    }


def axis_regressor_columns(
    t: np.ndarray,
    nu_smooth: np.ndarray,
    nudot: np.ndarray,
    rpy_smooth: np.ndarray,
    depth_smooth: np.ndarray,
) -> list[dict[str, np.ndarray]]:
    return [
        {
            "added_mass_u_dot": -nudot[:, 0],
            "linear_damping_u": -nu_smooth[:, 0],
            "quadratic_damping_u": -np.abs(nu_smooth[:, 0]) * nu_smooth[:, 0],
            "bias_force_x": np.ones_like(t),
        },
        {
            "added_mass_v_dot": -nudot[:, 1],
            "linear_damping_v": -nu_smooth[:, 1],
            "quadratic_damping_v": -np.abs(nu_smooth[:, 1]) * nu_smooth[:, 1],
            "bias_force_y": np.ones_like(t),
        },
        {
            "added_mass_w_dot": -nudot[:, 2],
            "linear_damping_w": -nu_smooth[:, 2],
            "quadratic_damping_w": -np.abs(nu_smooth[:, 2]) * nu_smooth[:, 2],
            "net_buoyancy_minus_weight": np.ones_like(t),
            "depth_stiffness_about_mean": -(depth_smooth - np.nanmedian(depth_smooth)),
        },
        {
            "added_inertia_p_dot": -nudot[:, 3],
            "linear_damping_p": -nu_smooth[:, 3],
            "quadratic_damping_p": -np.abs(nu_smooth[:, 3]) * nu_smooth[:, 3],
            "restoring_roll": -rpy_smooth[:, 0],
            "bias_torque_roll": np.ones_like(t),
        },
        {
            "added_inertia_q_dot": -nudot[:, 4],
            "linear_damping_q": -nu_smooth[:, 4],
            "quadratic_damping_q": -np.abs(nu_smooth[:, 4]) * nu_smooth[:, 4],
            "restoring_pitch": -rpy_smooth[:, 1],
            "bias_torque_pitch": np.ones_like(t),
        },
        {
            "added_inertia_r_dot": -nudot[:, 5],
            "linear_damping_r": -nu_smooth[:, 5],
            "quadratic_damping_r": -np.abs(nu_smooth[:, 5]) * nu_smooth[:, 5],
            "bias_torque_yaw": np.ones_like(t),
        },
    ]


def physical_lower_bounds(columns: dict[str, np.ndarray]) -> dict[str, float]:
    lower: dict[str, float] = {}
    for name in columns:
        if (
            name.startswith("added_")
            or name.startswith("linear_damping_")
            or name.startswith("quadratic_damping_")
            or name.startswith("restoring_")
        ):
            lower[name] = 0.0
    return lower


def current_profile_reference(replay: OfflineUuvReplay) -> dict[str, Any]:
    m = float(replay.vehicle_mass)
    g = float(replay.g)
    B = m * g * float(replay.buoyancy_scale)
    com = replay.model.body_ipos[replay.base_id].copy()
    center = None
    dz = None
    if replay.buoyancy_points:
        total_share = float(sum(point.share for point in replay.buoyancy_points))
        points = []
        for point in replay.buoyancy_points:
            p = point.pos.astype(float, copy=True)
            p[0] += replay.cob_longitudinal_offset
            p[2] += replay.cob_vertical_offset
            points.append((p, point.share / max(total_share, 1.0e-9)))
        center = sum(p * s for p, s in points)
        dz = float(center[2] - com[2])
    elif replay.cob_site_id >= 0:
        center = replay.model.site_pos[replay.cob_site_id].copy()
        dz = float(center[2] - com[2])
    restoring = None
    if dz is not None:
        restoring = float(replay.cob_torque_scale * B * dz)
    return {
        "mass_kg": m,
        "inertia_diag_kg_m2": [float(v) for v in replay.model.body_inertia[replay.base_id]],
        "buoyancy_scale": float(replay.buoyancy_scale),
        "net_buoyancy_minus_weight_n": float(m * g * (float(replay.buoyancy_scale) - 1.0)),
        "manual_full_heave_damping_n_per_mps": float(replay.full_heave_damping),
        "manual_surface_heave_damping_n_per_mps": float(replay.surface_heave_damping),
        "cob_torque_scale": float(replay.cob_torque_scale),
        "effective_cob_minus_com_z_m": dz,
        "small_angle_restoring_k_n_m_per_rad": restoring,
        "buoyancy_point_center_body_m": [float(v) for v in center] if center is not None else None,
        "note": "MuJoCo built-in ellipsoid fluid force is active but not exposed as an explicit coefficient here.",
    }


def build_thruster_param_overrides(horizontal_scale: float, vertical_scale: float) -> dict[str, Any] | None:
    horizontal_scale = float(horizontal_scale)
    vertical_scale = float(vertical_scale)
    if abs(horizontal_scale - 1.0) < 1.0e-12 and abs(vertical_scale - 1.0) < 1.0e-12:
        return None
    try:
        payload = json.loads(Path(THRUSTER_PARAMS_PATH).read_text())
    except (OSError, json.JSONDecodeError):
        payload = {}
    per = payload.get("per_thruster", {}) if isinstance(payload, dict) else {}
    overrides: dict[str, Any] = {"per_thruster": {}}
    for name in PHYSICAL_YAW_THRUSTERS:
        cfg = per.get(name, {}) if isinstance(per, dict) else {}
        base_gain = float(cfg.get("gain_scale", 1.0)) if isinstance(cfg, dict) else 1.0
        overrides["per_thruster"][name] = {"gain_scale": base_gain * horizontal_scale}
    for name in PHYSICAL_VERTICAL_THRUSTERS:
        cfg = per.get(name, {}) if isinstance(per, dict) else {}
        base_gain = float(cfg.get("gain_scale", 1.0)) if isinstance(cfg, dict) else 1.0
        overrides["per_thruster"][name] = {"gain_scale": base_gain * vertical_scale}
    return overrides


def build_identification(
    real: RealSeries,
    replay: OfflineUuvReplay,
    *,
    start_s: float,
    duration_s: float | None,
    dt_s: float,
    smooth_s: float,
    command_source: str,
    velocity_signs: np.ndarray,
    gyro_signs: np.ndarray,
    attitude_signs: np.ndarray,
    ridge: float,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    t = regular_grid(real, start_s, duration_s, dt_s)
    smooth_samples = max(1, int(round(float(smooth_s) / float(dt_s))))

    dvl = interp_matrix(real.dvl_t, real.dvl_vel, t, 3) * velocity_signs.reshape(1, 3)
    gyro = interp_matrix(real.imu_t, real.imu_gyro, t, 3) * gyro_signs.reshape(1, 3)
    rpy = unwrap_rpy(interp_matrix(real.imu_t, real.imu_rpy, t, 3)) * attitude_signs.reshape(1, 3)
    pressure_depth = interp_scalar(real.pressure_depth_t, real.pressure_depth, t)
    depth = pressure_depth
    if not np.any(np.isfinite(depth)) and real.depth.size:
        depth = interp_scalar(real.depth_t, real.depth, t)

    nu = np.column_stack([dvl, gyro])
    nu_smooth = moving_average(nu, smooth_samples)
    rpy_smooth = moving_average(rpy, smooth_samples)
    depth_smooth = moving_average(depth, smooth_samples)
    nudot = derivative(t, nu_smooth)

    thr = compute_thruster_wrench(replay, real, t, rpy_smooth, command_source=command_source)

    mass = float(replay.vehicle_mass)
    inertia = replay.model.body_inertia[replay.base_id].astype(float, copy=True)
    rigid_diag = np.array([mass, mass, mass, *inertia], dtype=float)
    target = rigid_diag.reshape(1, 6) * nudot - thr.wrench_body
    mujoco_fluid_wrench = compute_mujoco_fluid_wrench(replay, nu_smooth, rpy_smooth)

    axis_columns = axis_regressor_columns(t, nu_smooth, nudot, rpy_smooth, depth_smooth)

    fits: dict[str, Any] = {}
    physical_fits: dict[str, Any] = {}
    mujoco_fluid_fits: dict[str, Any] = {}
    fit_arrays: dict[str, np.ndarray] = {}
    for idx, axis in enumerate(AXIS_NAMES):
        fit = fit_linear_model(t, target[:, idx], axis_columns[idx], ridge=ridge)
        physical_fit = fit_bounded_linear_model(
            t,
            target[:, idx],
            axis_columns[idx],
            ridge=ridge,
            lower_bounds=physical_lower_bounds(axis_columns[idx]),
        )
        fluid_fit = fit_bounded_linear_model(
            t,
            mujoco_fluid_wrench[:, idx],
            axis_columns[idx],
            ridge=ridge,
            lower_bounds=physical_lower_bounds(axis_columns[idx]),
        )
        arrays = {}
        for key in ("prediction", "target", "residual", "fit_time"):
            value = fit.pop(key, None)
            if value is not None:
                arrays[key] = np.asarray(value, dtype=float)
                fit_arrays[f"{axis}_{key}"] = arrays[key]
        fits[axis] = fit
        physical_fits[axis] = physical_fit
        mujoco_fluid_fits[axis] = fluid_fit

    summary = {
        "method": (
            "Per-axis inverse dynamics least squares: "
            "M_rb*nu_dot - tau_thr = Y(theta). Positive damping/restoring signs mean force/torque opposes velocity/angle."
        ),
        "assumptions": {
            "body_frame": "MuJoCo/base_link FLU convention assumed: x forward, y left, z up.",
            "linear_velocity_source": "/dvl/twist linear velocity, interpolated and smoothed",
            "angular_velocity_source": "/mavros/imu/data angular_velocity",
            "attitude_source": "/mavros/imu/data orientation",
            "thruster_source": command_source,
            "rigid_body_coriolis": "not included in this first linear diagnostic fit",
            "ellipsoid_mujoco_drag": "not directly observable as a separate regressor from the real bag; residual captures missing/nonlinear terms",
        },
        "bag": str(real.name),
        "window": {
            "start_s": float(t[0]),
            "end_s": float(t[-1]),
            "duration_s": float(t[-1] - t[0]),
            "dt_s": float(dt_s),
            "smooth_s": float(smooth_s),
            "samples": int(t.size),
        },
        "source_counts": {
            "dvl": int(real.dvl_t.size),
            "imu": int(real.imu_t.size),
            "rc_out": int(real.rc_out_t.size),
            "rc_override": int(real.rc_override_t.size),
            "joy": int(real.joy_t.size),
            "pressure_depth": int(real.pressure_depth_t.size),
        },
        "thruster_force_scale": {
            "note": "Applied by scaling per-thruster gain_scale before computing tau_thruster.",
            "horizontal": float(getattr(replay, "ident_horizontal_force_scale", 1.0)),
            "vertical": float(getattr(replay, "ident_vertical_force_scale", 1.0)),
        },
        "velocity_signs": [float(v) for v in velocity_signs],
        "gyro_signs": [float(v) for v in gyro_signs],
        "attitude_signs": [float(v) for v in attitude_signs],
        "current_profile_reference": current_profile_reference(replay),
        "fits": fits,
        "physical_fits": physical_fits,
        "mujoco_fluid_fits": mujoco_fluid_fits,
    }

    series = {
        "t": t,
        "nu": nu,
        "nu_smooth": nu_smooth,
        "nudot": nudot,
        "rpy": rpy,
        "rpy_smooth": rpy_smooth,
        "depth": depth,
        "depth_smooth": depth_smooth,
        "thruster_wrench_body": thr.wrench_body,
        "mujoco_fluid_wrench_body": mujoco_fluid_wrench,
        "target_hydro_wrench_body": target,
        "command8_norm": thr.command8_norm,
    }
    for key, value in fit_arrays.items():
        series[key] = value
    for name, value in thr.thruster_forces.items():
        series[f"thr_{name}_force"] = value
    return summary, series


def json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_ready(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_ready(v) for v in value]
    if isinstance(value, tuple):
        return [json_ready(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_ready(value.tolist())
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def write_coeff_csv(summary: dict[str, Any], path: Path) -> None:
    rows = []
    for axis, fit in summary["fits"].items():
        coeffs = fit.get("coefficients", {})
        for name, value in coeffs.items():
            rows.append(
                {
                    "axis": axis,
                    "coefficient": name,
                    "value": "" if value is None else f"{float(value):.12g}",
                    "rmse": fit.get("rmse", ""),
                    "r2": fit.get("r2", ""),
                    "condition_number_scaled": fit.get("condition_number_scaled", ""),
                }
            )
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=("axis", "coefficient", "value", "rmse", "r2", "condition_number_scaled"),
        )
        writer.writeheader()
        writer.writerows(rows)


def write_physical_coeff_csv(summary: dict[str, Any], path: Path) -> None:
    rows = []
    for axis, fit in summary.get("physical_fits", {}).items():
        coeffs = fit.get("coefficients", {})
        for name, value in coeffs.items():
            rows.append(
                {
                    "axis": axis,
                    "coefficient": name,
                    "value": "" if value is None else f"{float(value):.12g}",
                    "rmse": fit.get("rmse", ""),
                    "r2": fit.get("r2", ""),
                    "active_lower_bounds": ";".join(fit.get("active_lower_bounds", [])),
                    "condition_number_scaled": fit.get("condition_number_scaled", ""),
                }
            )
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "axis",
                "coefficient",
                "value",
                "rmse",
                "r2",
                "active_lower_bounds",
                "condition_number_scaled",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)


def write_group_coeff_csv(summary: dict[str, Any], group: str, path: Path) -> None:
    rows = []
    for axis, fit in summary.get(group, {}).items():
        coeffs = fit.get("coefficients", {})
        for name, value in coeffs.items():
            rows.append(
                {
                    "axis": axis,
                    "coefficient": name,
                    "value": "" if value is None else f"{float(value):.12g}",
                    "rmse": fit.get("rmse", ""),
                    "r2": fit.get("r2", ""),
                    "active_lower_bounds": ";".join(fit.get("active_lower_bounds", [])),
                    "condition_number_scaled": fit.get("condition_number_scaled", ""),
                }
            )
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "axis",
                "coefficient",
                "value",
                "rmse",
                "r2",
                "active_lower_bounds",
                "condition_number_scaled",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)


def fit_coeff(summary: dict[str, Any], group: str, axis: str, name: str) -> float | None:
    value = summary.get(group, {}).get(axis, {}).get("coefficients", {}).get(name)
    return None if value is None else float(value)


def fit_r2(summary: dict[str, Any], group: str, axis: str) -> float | None:
    value = summary.get(group, {}).get(axis, {}).get("r2")
    return None if value is None else float(value)


def confidence_from_r2(value: float | None) -> str:
    if value is None:
        return "unknown"
    if value >= 0.70:
        return "high"
    if value >= 0.45:
        return "medium"
    if value >= 0.20:
        return "low"
    return "very_low"


def blend(current: float, direct: float | None, gain: float) -> float | None:
    if direct is None or not np.isfinite(direct):
        return None
    return float(current + float(gain) * (direct - current))


def build_parameter_recommendation(summary: dict[str, Any]) -> dict[str, Any]:
    ref = summary["current_profile_reference"]
    mass = float(ref["mass_kg"])
    g = 9.80665
    current_buoyancy_scale = float(ref["buoyancy_scale"])
    current_surface_heave = float(ref["manual_surface_heave_damping_n_per_mps"])
    current_full_heave = float(ref["manual_full_heave_damping_n_per_mps"])
    current_heave_scale = current_full_heave / max(current_surface_heave, 1.0e-9)
    dz = ref.get("effective_cob_minus_com_z_m")

    net_b = fit_coeff(summary, "physical_fits", "heave_z", "net_buoyancy_minus_weight")
    heave_linear = fit_coeff(summary, "physical_fits", "heave_z", "linear_damping_w")
    heave_quad = fit_coeff(summary, "physical_fits", "heave_z", "quadratic_damping_w")
    surge_linear = fit_coeff(summary, "physical_fits", "surge_x", "linear_damping_u")
    surge_quad = fit_coeff(summary, "physical_fits", "surge_x", "quadratic_damping_u")
    yaw_linear = fit_coeff(summary, "physical_fits", "yaw_n", "linear_damping_r")
    yaw_quad = fit_coeff(summary, "physical_fits", "yaw_n", "quadratic_damping_r")
    roll_restore = fit_coeff(summary, "physical_fits", "roll_k", "restoring_roll")
    pitch_restore = fit_coeff(summary, "physical_fits", "pitch_m", "restoring_pitch")
    fluid_surge_linear = fit_coeff(summary, "mujoco_fluid_fits", "surge_x", "linear_damping_u")
    fluid_surge_quad = fit_coeff(summary, "mujoco_fluid_fits", "surge_x", "quadratic_damping_u")
    fluid_heave_linear = fit_coeff(summary, "mujoco_fluid_fits", "heave_z", "linear_damping_w")
    fluid_heave_quad = fit_coeff(summary, "mujoco_fluid_fits", "heave_z", "quadratic_damping_w")
    fluid_yaw_linear = fit_coeff(summary, "mujoco_fluid_fits", "yaw_n", "linear_damping_r")
    fluid_yaw_quad = fit_coeff(summary, "mujoco_fluid_fits", "yaw_n", "quadratic_damping_r")

    direct_buoyancy_scale = None
    if net_b is not None:
        direct_buoyancy_scale = float(1.0 + net_b / max(mass * g, 1.0e-9))
    direct_heave_scale = None
    if heave_linear is not None:
        direct_heave_scale = float(heave_linear / max(current_surface_heave, 1.0e-9))
    split_manual_heave_linear = None
    split_heave_scale = None
    if heave_linear is not None and fluid_heave_linear is not None:
        split_manual_heave_linear = float(max(0.0, heave_linear - fluid_heave_linear))
        split_heave_scale = float(split_manual_heave_linear / max(current_surface_heave, 1.0e-9))

    direct_roll_cob_scale = None
    direct_pitch_cob_scale = None
    direct_buoyancy = mass * g * (direct_buoyancy_scale or current_buoyancy_scale)
    if dz is not None and abs(float(dz)) > 1.0e-9 and direct_buoyancy > 1.0e-9:
        if roll_restore is not None:
            direct_roll_cob_scale = float(roll_restore / (direct_buoyancy * float(dz)))
        if pitch_restore is not None:
            direct_pitch_cob_scale = float(pitch_restore / (direct_buoyancy * float(dz)))

    direct_equivalent = {
        "buoyancy_scale": direct_buoyancy_scale,
        "heave_damping_scale": direct_heave_scale,
        "full_heave_damping_n_per_mps": heave_linear,
        "heave_quadratic_damping_n_per_mps2": heave_quad,
        "surge_linear_damping_n_per_mps": surge_linear,
        "surge_quadratic_damping_n_per_mps2": surge_quad,
        "yaw_linear_damping_n_m_per_radps": yaw_linear,
        "yaw_quadratic_damping_n_m_per_radps2": yaw_quad,
        "cob_torque_scale_from_roll": direct_roll_cob_scale,
        "cob_torque_scale_from_pitch": direct_pitch_cob_scale,
    }
    mujoco_ellipsoid_equivalent = {
        "surge_linear_damping_n_per_mps": fluid_surge_linear,
        "surge_quadratic_damping_n_per_mps2": fluid_surge_quad,
        "heave_linear_damping_n_per_mps": fluid_heave_linear,
        "heave_quadratic_damping_n_per_mps2": fluid_heave_quad,
        "yaw_linear_damping_n_m_per_radps": fluid_yaw_linear,
        "yaw_quadratic_damping_n_m_per_radps2": fluid_yaw_quad,
    }
    ellipsoid_scale_estimate = {
        "surge_linear_scale": None if fluid_surge_linear in (None, 0.0) or surge_linear is None else float(surge_linear / fluid_surge_linear),
        "surge_quadratic_scale": None if fluid_surge_quad in (None, 0.0) or surge_quad is None else float(surge_quad / fluid_surge_quad),
        "heave_linear_scale": None if fluid_heave_linear in (None, 0.0) or heave_linear is None else float(heave_linear / fluid_heave_linear),
        "heave_quadratic_scale": None if fluid_heave_quad in (None, 0.0) or heave_quad is None else float(heave_quad / fluid_heave_quad),
        "yaw_linear_scale": None if fluid_yaw_linear in (None, 0.0) or yaw_linear is None else float(yaw_linear / fluid_yaw_linear),
        "yaw_quadratic_scale": None if fluid_yaw_quad in (None, 0.0) or yaw_quad is None else float(yaw_quad / fluid_yaw_quad),
    }

    conservative_candidate = {
        "buoyancy_scale": blend(current_buoyancy_scale, direct_buoyancy_scale, 0.35),
        "heave_damping_scale": blend(current_heave_scale, split_heave_scale or direct_heave_scale, 0.50),
        "surface_heave_damping": current_surface_heave,
        "split_manual_full_heave_damping_n_per_mps": blend(current_full_heave, split_manual_heave_linear, 0.50),
        "comment": (
            "Candidate is intentionally blended toward the inverse-dynamics solution. "
            "The direct equivalent ignores MuJoCo built-in ellipsoid drag separation, so applying it all at once can over-damp."
        ),
    }
    if direct_roll_cob_scale is not None and confidence_from_r2(fit_r2(summary, "physical_fits", "roll_k")) != "very_low":
        conservative_candidate["cob_torque_scale_from_roll"] = blend(
            float(ref["cob_torque_scale"]),
            direct_roll_cob_scale,
            0.25,
        )

    return {
        "method": (
            "Physical constrained least-squares. Added mass, damping, and restoring coefficients are constrained >= 0. "
            "The MuJoCo current profile uses built-in ellipsoid fluid plus custom hydrostatic/heave damping, so direct coefficients are aggregate equivalents."
        ),
        "axis_confidence": {
            axis: {
                "r2": fit_r2(summary, "physical_fits", axis),
                "confidence": confidence_from_r2(fit_r2(summary, "physical_fits", axis)),
                "active_lower_bounds": summary.get("physical_fits", {}).get(axis, {}).get("active_lower_bounds", []),
            }
            for axis in AXIS_NAMES
        },
        "direct_equivalent_parameters": direct_equivalent,
        "current_mujoco_ellipsoid_equivalent": mujoco_ellipsoid_equivalent,
        "ellipsoid_fluidcoef_scale_estimate": ellipsoid_scale_estimate,
        "split_heave_after_subtracting_mujoco_ellipsoid": {
            "manual_full_heave_damping_n_per_mps": split_manual_heave_linear,
            "heave_damping_scale": split_heave_scale,
        },
        "conservative_next_test_profile_values": conservative_candidate,
        "current_reference": {
            "buoyancy_scale": current_buoyancy_scale,
            "surface_heave_damping": current_surface_heave,
            "heave_damping_scale": current_heave_scale,
            "full_heave_damping_n_per_mps": current_full_heave,
            "cob_torque_scale": float(ref["cob_torque_scale"]),
            "small_angle_restoring_k_n_m_per_rad": ref.get("small_angle_restoring_k_n_m_per_rad"),
        },
        "cautions": [
            "Do not use sway_y from this bag; dominant sway excitation is insufficient.",
            "Roll/pitch restoring from this closed-loop bag is weaker than a proper static/free-decay test; treat it as diagnostic only.",
            "Heave coefficients are the most relevant to the ALT_HOLD depth drift issue, but still include controller and pressure-depth effects.",
            "Surge and yaw are the best candidates for ellipsoid drag tuning from this bag.",
        ],
    }


def plot_wrench_fit(summary: dict[str, Any], series: dict[str, np.ndarray], path: Path) -> None:
    fig, axes = plt.subplots(6, 1, figsize=(13, 15), sharex=False)
    fig.suptitle("Inverse Dynamics Fit: required hydro wrench vs fitted model", fontsize=14, fontweight="bold")
    for idx, axis in enumerate(AXIS_NAMES):
        ax = axes[idx]
        tt = series.get(f"{axis}_fit_time", np.array([]))
        target = series.get(f"{axis}_target", np.array([]))
        pred = series.get(f"{axis}_prediction", np.array([]))
        residual = series.get(f"{axis}_residual", np.array([]))
        if tt.size:
            ax.plot(tt, target, color="black", linewidth=1.0, label="bag inverse target")
            ax.plot(tt, pred, color="crimson", linewidth=1.0, label="least-squares fit")
            ax.plot(tt, residual, color="tab:blue", linewidth=0.7, alpha=0.6, label="residual")
        fit = summary["fits"].get(axis, {})
        rmse = fit.get("rmse")
        r2 = fit.get("r2")
        unit = "N" if idx < 3 else "N*m"
        ax.set_ylabel(f"{AXIS_SHORT[idx]}\n{unit}")
        ax.grid(True, alpha=0.25)
        ax.set_title(f"{axis}: RMSE={rmse:.3g} {unit}" if rmse is not None else axis, loc="left")
        if r2 is not None:
            ax.text(0.99, 0.85, f"R2={r2:.3f}", transform=ax.transAxes, ha="right", va="center")
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[-1].set_xlabel("bag time [s]")
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_coefficients(summary: dict[str, Any], path: Path) -> None:
    selected = [
        ("heave_z", "linear_damping_w", "fit c_w"),
        ("heave_z", "net_buoyancy_minus_weight", "fit B-W"),
        ("roll_k", "restoring_roll", "fit K_roll"),
        ("pitch_m", "restoring_pitch", "fit K_pitch"),
        ("yaw_n", "linear_damping_r", "fit c_r"),
    ]
    labels = []
    values = []
    for axis, key, label in selected:
        coeff = summary["fits"].get(axis, {}).get("coefficients", {}).get(key)
        if coeff is not None:
            labels.append(label)
            values.append(float(coeff))
    ref = summary["current_profile_reference"]
    ref_labels = []
    ref_values = []
    if ref.get("manual_full_heave_damping_n_per_mps") is not None:
        ref_labels.append("profile c_w manual")
        ref_values.append(float(ref["manual_full_heave_damping_n_per_mps"]))
    if ref.get("net_buoyancy_minus_weight_n") is not None:
        ref_labels.append("profile B-W")
        ref_values.append(float(ref["net_buoyancy_minus_weight_n"]))
    if ref.get("small_angle_restoring_k_n_m_per_rad") is not None:
        ref_labels.append("profile K_restore")
        ref_values.append(float(ref["small_angle_restoring_k_n_m_per_rad"]))

    all_labels = labels + ref_labels
    all_values = values + ref_values
    colors = ["#111111"] * len(labels) + ["#d62728"] * len(ref_labels)
    fig, ax = plt.subplots(figsize=(12, 5))
    x = np.arange(len(all_labels))
    ax.bar(x, all_values, color=colors)
    ax.axhline(0.0, color="0.3", linewidth=0.8)
    ax.set_xticks(x, all_labels, rotation=25, ha="right")
    ax.set_ylabel("coefficient value [mixed units]")
    ax.set_title("Key fitted coefficients vs current profile reference")
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_signals(series: dict[str, np.ndarray], path: Path) -> None:
    t = series["t"]
    nu = series["nu_smooth"]
    nudot = series["nudot"]
    rpy = series["rpy_smooth"]
    cmd = series["command8_norm"]
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    axes[0].plot(t, nu[:, 0], label="u")
    axes[0].plot(t, nu[:, 1], label="v")
    axes[0].plot(t, nu[:, 2], label="w")
    axes[0].set_ylabel("linear vel [m/s]")
    axes[1].plot(t, nu[:, 3], label="p")
    axes[1].plot(t, nu[:, 4], label="q")
    axes[1].plot(t, nu[:, 5], label="r")
    axes[1].set_ylabel("angular vel [rad/s]")
    axes[2].plot(t, np.rad2deg(rpy[:, 0]), label="roll")
    axes[2].plot(t, np.rad2deg(rpy[:, 1]), label="pitch")
    axes[2].plot(t, np.rad2deg(rpy[:, 2]), label="yaw")
    axes[2].set_ylabel("attitude [deg]")
    for idx in range(min(8, cmd.shape[1])):
        axes[3].plot(t, cmd[:, idx], label=f"ch{idx+1}", linewidth=0.8)
    axes[3].set_ylabel("command norm")
    axes[3].set_xlabel("bag time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=4, fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def contiguous_windows(t: np.ndarray, mask: np.ndarray, min_duration_s: float) -> list[tuple[int, int]]:
    mask = np.asarray(mask, dtype=bool)
    if t.size == 0 or mask.size != t.size:
        return []
    windows: list[tuple[int, int]] = []
    start: int | None = None
    for idx, active in enumerate(mask):
        if active and start is None:
            start = idx
        elif not active and start is not None:
            end = idx
            if float(t[end - 1] - t[start]) >= min_duration_s:
                windows.append((start, end))
            start = None
    if start is not None:
        end = mask.size
        if float(t[end - 1] - t[start]) >= min_duration_s:
            windows.append((start, end))
    return windows


def fit_axis_subset(
    t: np.ndarray,
    target: np.ndarray,
    columns: dict[str, np.ndarray],
    mask: np.ndarray,
    *,
    ridge: float,
) -> dict[str, Any]:
    return fit_linear_model(t[mask], target[mask], {key: value[mask] for key, value in columns.items()}, ridge=ridge)


def auto_segment_report(
    series: dict[str, np.ndarray],
    *,
    ridge: float,
    min_duration_s: float = 0.75,
    dominance_ratio: float = 0.45,
) -> dict[str, Any]:
    t = series["t"]
    target = series["target_hydro_wrench_body"]
    wrench = series["thruster_wrench_body"]
    columns = axis_regressor_columns(
        t,
        series["nu_smooth"],
        series["nudot"],
        series["rpy_smooth"],
        series["depth_smooth"],
    )
    groups = [slice(0, 3), slice(0, 3), slice(0, 3), slice(3, 6), slice(3, 6), slice(3, 6)]
    report: dict[str, Any] = {
        "method": (
            "Dominant-axis windows are detected from applied thruster wrench. "
            "A window is kept when that axis is both large relative to its own p95 and dominant inside force/torque group."
        ),
        "min_duration_s": float(min_duration_s),
        "dominance_ratio": float(dominance_ratio),
        "axes": {},
    }
    for axis_idx, axis in enumerate(AXIS_NAMES):
        mag = np.abs(wrench[:, axis_idx])
        finite = np.isfinite(mag)
        if not np.any(finite):
            report["axes"][axis] = {"status": "no_finite_wrench", "segments": []}
            continue
        p95 = float(np.nanpercentile(mag[finite], 95.0))
        p70 = float(np.nanpercentile(mag[finite], 70.0))
        group_mag = np.nansum(np.abs(wrench[:, groups[axis_idx]]), axis=1)
        ratio = mag / np.maximum(group_mag, 1.0e-9)
        active = finite & (mag >= max(0.25 * p95, p70)) & (ratio >= dominance_ratio)
        active &= np.all(np.isfinite(target), axis=1)
        active &= np.isfinite(series["nu_smooth"][:, axis_idx])
        windows = contiguous_windows(t, active, min_duration_s)
        axis_entry: dict[str, Any] = {
            "wrench_p95_abs": p95,
            "wrench_p70_abs": p70,
            "active_fraction": float(np.mean(active)),
            "segments": [],
        }
        if np.count_nonzero(active) >= 24:
            combined = fit_axis_subset(t, target[:, axis_idx], columns[axis_idx], active, ridge=ridge)
            axis_entry["combined_fit"] = {
                key: value for key, value in combined.items() if key not in {"prediction", "target", "residual", "fit_time"}
            }
        else:
            axis_entry["combined_fit"] = {"status": "insufficient_dominant_samples", "count": int(np.count_nonzero(active))}
        for start, end in windows:
            seg_mask = np.zeros_like(active)
            seg_mask[start:end] = True
            fit = fit_axis_subset(t, target[:, axis_idx], columns[axis_idx], seg_mask, ridge=ridge)
            axis_entry["segments"].append(
                {
                    "start_s": float(t[start]),
                    "end_s": float(t[end - 1]),
                    "duration_s": float(t[end - 1] - t[start]),
                    "count": int(end - start),
                    "mean_wrench": float(np.nanmean(wrench[start:end, axis_idx])),
                    "max_abs_wrench": float(np.nanmax(mag[start:end])),
                    "fit": {
                        key: value
                        for key, value in fit.items()
                        if key not in {"prediction", "target", "residual", "fit_time"}
                    },
                }
            )
        report["axes"][axis] = axis_entry
    return report


def write_segment_csv(report: dict[str, Any], path: Path) -> None:
    fields = (
        "axis",
        "kind",
        "start_s",
        "end_s",
        "duration_s",
        "count",
        "status",
        "r2",
        "rmse",
        "coefficient",
        "value",
    )
    rows: list[dict[str, Any]] = []
    for axis, axis_entry in report.get("axes", {}).items():
        fits: list[tuple[str, dict[str, Any], dict[str, Any]]] = []
        combined = axis_entry.get("combined_fit")
        if isinstance(combined, dict):
            fits.append(("combined", {"start_s": "", "end_s": "", "duration_s": "", "count": combined.get("count", "")}, combined))
        for segment in axis_entry.get("segments", []):
            fits.append(("segment", segment, segment.get("fit", {})))
        for kind, meta, fit in fits:
            coeffs = fit.get("coefficients", {}) if isinstance(fit, dict) else {}
            if not coeffs:
                rows.append(
                    {
                        "axis": axis,
                        "kind": kind,
                        "start_s": meta.get("start_s", ""),
                        "end_s": meta.get("end_s", ""),
                        "duration_s": meta.get("duration_s", ""),
                        "count": meta.get("count", fit.get("count", "")),
                        "status": fit.get("status", ""),
                        "r2": fit.get("r2", ""),
                        "rmse": fit.get("rmse", ""),
                        "coefficient": "",
                        "value": "",
                    }
                )
                continue
            for coefficient, value in coeffs.items():
                rows.append(
                    {
                        "axis": axis,
                        "kind": kind,
                        "start_s": meta.get("start_s", ""),
                        "end_s": meta.get("end_s", ""),
                        "duration_s": meta.get("duration_s", ""),
                        "count": meta.get("count", fit.get("count", "")),
                        "status": fit.get("status", ""),
                        "r2": fit.get("r2", ""),
                        "rmse": fit.get("rmse", ""),
                        "coefficient": coefficient,
                        "value": "" if value is None else f"{float(value):.12g}",
                    }
                )
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_segment_scores(report: dict[str, Any], path: Path) -> None:
    labels = []
    r2_values = []
    active = []
    for axis in AXIS_NAMES:
        entry = report.get("axes", {}).get(axis, {})
        fit = entry.get("combined_fit", {}) if isinstance(entry, dict) else {}
        r2 = fit.get("r2") if isinstance(fit, dict) else None
        labels.append(axis.replace("_", "\n"))
        r2_values.append(0.0 if r2 is None else max(0.0, float(r2)))
        active.append(float(entry.get("active_fraction", 0.0)) if isinstance(entry, dict) else 0.0)
    x = np.arange(len(labels))
    fig, ax1 = plt.subplots(figsize=(11, 4.8))
    ax1.bar(x - 0.18, r2_values, width=0.36, color="#1f77b4", label="dominant-window R2")
    ax1.set_ylim(0.0, 1.0)
    ax1.set_ylabel("R2")
    ax1.grid(True, axis="y", alpha=0.25)
    ax2 = ax1.twinx()
    ax2.bar(x + 0.18, active, width=0.36, color="#ff7f0e", alpha=0.75, label="active fraction")
    ax2.set_ylim(0.0, 1.0)
    ax2.set_ylabel("fraction of window")
    ax1.set_xticks(x, labels)
    ax1.set_title("Dominant-axis identification quality")
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_timeseries_csv(series: dict[str, np.ndarray], path: Path) -> None:
    t = series["t"]
    fields = ["t"]
    columns = [t]
    for prefix, arr, names in (
        ("nu", series["nu_smooth"], ("u", "v", "w", "p", "q", "r")),
        ("nudot", series["nudot"], ("udot", "vdot", "wdot", "pdot", "qdot", "rdot")),
        ("thr", series["thruster_wrench_body"], ("fx", "fy", "fz", "tx", "ty", "tz")),
        ("mujoco_fluid", series["mujoco_fluid_wrench_body"], ("fx", "fy", "fz", "tx", "ty", "tz")),
        ("target", series["target_hydro_wrench_body"], ("fx", "fy", "fz", "tx", "ty", "tz")),
    ):
        for idx, name in enumerate(names):
            fields.append(f"{prefix}_{name}")
            columns.append(arr[:, idx])
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for row in zip(*columns):
            writer.writerow([f"{float(v):.12g}" for v in row])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=DEFAULT_REAL_BAG)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--scene", type=Path, default=CURRENT_SCENE)
    parser.add_argument("--profile", default="current")
    parser.add_argument("--fluid-model", default="current")
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--smooth", type=float, default=0.55)
    parser.add_argument("--command-source", choices=("rc_out", "rc_override", "joy_node"), default="rc_out")
    parser.add_argument("--velocity-signs", type=lambda s: parse_vec3(s, default=(1.0, 1.0, 1.0)), default=np.ones(3))
    parser.add_argument("--gyro-signs", type=lambda s: parse_vec3(s, default=(1.0, 1.0, 1.0)), default=np.ones(3))
    parser.add_argument("--attitude-signs", type=lambda s: parse_vec3(s, default=(1.0, 1.0, 1.0)), default=np.ones(3))
    parser.add_argument("--horizontal-force-scale", type=float, default=1.0)
    parser.add_argument("--vertical-force-scale", type=float, default=1.0)
    parser.add_argument("--ridge", type=float, default=1.0e-6)
    parser.add_argument("--auto-segments", action="store_true", help="write dominant-axis segment fit report")
    args = parser.parse_args()

    db_path = resolve_db3(args.bag)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[identify] reading real bag: {db_path}", flush=True)
    real = extract_real_series(db_path)
    print(
        "[identify] series counts: "
        f"dvl={real.dvl_t.size}, imu={real.imu_t.size}, rc_out={real.rc_out_t.size}, "
        f"rc_override={real.rc_override_t.size}, pressure_depth={real.pressure_depth_t.size}",
        flush=True,
    )

    thruster_param_overrides = build_thruster_param_overrides(
        float(args.horizontal_force_scale),
        float(args.vertical_force_scale),
    )
    replay = OfflineUuvReplay(
        scene=args.scene,
        profile_name=args.profile,
        fluid_model=args.fluid_model,
        thruster_dt_mode="current-code",
        thruster_param_overrides=thruster_param_overrides,
    )
    replay.ident_horizontal_force_scale = float(args.horizontal_force_scale)
    replay.ident_vertical_force_scale = float(args.vertical_force_scale)
    summary, series = build_identification(
        real,
        replay,
        start_s=args.start,
        duration_s=args.duration,
        dt_s=args.dt,
        smooth_s=args.smooth,
        command_source=args.command_source,
        velocity_signs=np.asarray(args.velocity_signs, dtype=float),
        gyro_signs=np.asarray(args.gyro_signs, dtype=float),
        attitude_signs=np.asarray(args.attitude_signs, dtype=float),
        ridge=float(args.ridge),
    )

    summary_path = out_dir / "inverse_dynamics_summary.json"
    coeff_path = out_dir / "inverse_dynamics_coefficients.csv"
    physical_coeff_path = out_dir / "inverse_dynamics_physical_coefficients.csv"
    fluid_coeff_path = out_dir / "inverse_dynamics_mujoco_fluid_coefficients.csv"
    recommendation_path = out_dir / "inverse_dynamics_parameter_recommendation.json"
    ts_path = out_dir / "inverse_dynamics_timeseries.csv"
    fit_plot_path = out_dir / "inverse_dynamics_wrench_fit.png"
    coeff_plot_path = out_dir / "inverse_dynamics_coefficients.png"
    signals_plot_path = out_dir / "inverse_dynamics_signals.png"

    summary_path.write_text(json.dumps(json_ready(summary), ensure_ascii=False, indent=2))
    write_coeff_csv(summary, coeff_path)
    write_physical_coeff_csv(summary, physical_coeff_path)
    write_group_coeff_csv(summary, "mujoco_fluid_fits", fluid_coeff_path)
    recommendation_path.write_text(
        json.dumps(json_ready(build_parameter_recommendation(summary)), ensure_ascii=False, indent=2)
    )
    write_timeseries_csv(series, ts_path)
    plot_wrench_fit(summary, series, fit_plot_path)
    plot_coefficients(summary, coeff_plot_path)
    plot_signals(series, signals_plot_path)
    if args.auto_segments:
        segment_report = auto_segment_report(series, ridge=float(args.ridge))
        segment_json_path = out_dir / "inverse_dynamics_segment_report.json"
        segment_csv_path = out_dir / "inverse_dynamics_segment_report.csv"
        segment_plot_path = out_dir / "inverse_dynamics_segment_quality.png"
        segment_json_path.write_text(json.dumps(json_ready(segment_report), ensure_ascii=False, indent=2))
        write_segment_csv(segment_report, segment_csv_path)
        plot_segment_scores(segment_report, segment_plot_path)

    print(f"[identify] wrote {summary_path}", flush=True)
    print(f"[identify] wrote {coeff_path}", flush=True)
    print(f"[identify] wrote {physical_coeff_path}", flush=True)
    print(f"[identify] wrote {fluid_coeff_path}", flush=True)
    print(f"[identify] wrote {recommendation_path}", flush=True)
    print(f"[identify] wrote {fit_plot_path}", flush=True)
    if args.auto_segments:
        print(f"[identify] wrote {out_dir / 'inverse_dynamics_segment_report.json'}", flush=True)

    ref = summary["current_profile_reference"]
    fits = summary["physical_fits"]
    def coeff(axis: str, name: str) -> float | None:
        value = fits.get(axis, {}).get("coefficients", {}).get(name)
        return None if value is None else float(value)

    print("[identify] key comparison", flush=True)
    print(
        "  heave damping fit/profile: "
        f"{coeff('heave_z', 'linear_damping_w')} / {ref['manual_full_heave_damping_n_per_mps']}",
        flush=True,
    )
    print(
        "  net buoyancy fit/profile: "
        f"{coeff('heave_z', 'net_buoyancy_minus_weight')} / {ref['net_buoyancy_minus_weight_n']}",
        flush=True,
    )
    print(
        "  roll restoring fit/profile: "
        f"{coeff('roll_k', 'restoring_roll')} / {ref['small_angle_restoring_k_n_m_per_rad']}",
        flush=True,
    )
    print(
        "  pitch restoring fit/profile: "
        f"{coeff('pitch_m', 'restoring_pitch')} / {ref['small_angle_restoring_k_n_m_per_rad']}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
