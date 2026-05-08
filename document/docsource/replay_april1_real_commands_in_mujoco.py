from __future__ import annotations

import argparse
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

from analyze_april1_real_bags import read_bag, scalar_stats, vector_stats  # noqa: E402
from physics.hydrodynamics_helpers import (  # noqa: E402
    added_mass_coriolis,
    first_order_response,
    scaled_polynomial_force,
    shape_thruster_command,
    submerged_fraction,
)
from physics.sim_profile_helpers import (  # noqa: E402
    build_hydrodynamics_config,
    build_sim_profile,
    canonical_profile_name,
    load_sim_profiles,
)
from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER,
    PHYSICAL_VERTICAL_THRUSTERS,
    PHYSICAL_YAW_THRUSTERS,
)
from physics.thruster_performance import load_thruster_performance  # noqa: E402


DEFAULT_ROOT = Path("real_robot_ros_bag/extracted_2026_04_01")
DEFAULT_OUT = Path("document/docsource/real_bag_2026_04_01_replay")
CURRENT_SCENE = UUV_DIR / "scenes" / "tank_current_scene.xml"
LEGACY_SCENE = UUV_DIR / "scenes" / "tank_legacy_scene.xml"
PROFILE_PATH = UUV_DIR / "config" / "sim_profiles.json"
THRUSTER_PERF_PATH = UUV_DIR / "config" / "thruster_performance.json"
THRUSTER_PARAMS_PATH = UUV_DIR / "config" / "thruster_params.json"


CONTROL_CHANNELS = (3, 4, 5, 6)
PWM_CENTER = 1500.0
RC_OVERRIDE_SPAN = 300.0
RC_OUT_SPAN = 400.0
COMMAND_MODE_NONE = 0
COMMAND_MODE_RC_OVERRIDE = 1
COMMAND_MODE_RC_OUT = 2


def discover_bags(root: Path) -> list[Path]:
    if root.is_file() and root.suffix == ".db3":
        return [root]
    bags: list[Path] = []
    for db_path in sorted(root.glob("bag_*/**/*.db3")):
        try:
            with db_path.open("rb"):
                pass
            import sqlite3

            conn = sqlite3.connect(str(db_path))
            try:
                has_topics = conn.execute(
                    "select 1 from sqlite_master where type='table' and name='topics'"
                ).fetchone()
            finally:
                conn.close()
            if has_topics:
                bags.append(db_path)
        except Exception:
            continue
    return bags


def normalize_rc_override(rc: np.ndarray) -> np.ndarray:
    channels = np.asarray(rc[:, :8], dtype=float)
    return np.clip((channels - PWM_CENTER) / RC_OVERRIDE_SPAN, -1.0, 1.0)


def normalize_rc_out(rc: np.ndarray) -> np.ndarray:
    channels = np.asarray(rc[:, :8], dtype=float)
    valid = (channels >= 800.0) & (channels <= 2200.0)
    out = np.zeros_like(channels, dtype=float)
    out[valid] = (channels[valid] - PWM_CENTER) / RC_OUT_SPAN
    return np.clip(out, -1.0, 1.0)


def axis_column(axes: np.ndarray, index: int) -> np.ndarray:
    if axes.size == 0:
        return np.empty(0, dtype=float)
    return axes[:, index] if index < axes.shape[1] else np.zeros(axes.shape[0], dtype=float)


def button_column(buttons: np.ndarray, index: int) -> np.ndarray:
    if buttons.size == 0:
        return np.empty(0, dtype=float)
    return buttons[:, index] if index < buttons.shape[1] else np.zeros(buttons.shape[0], dtype=float)


def joy_node_rc_override(joy_t: np.ndarray, joy_axes: np.ndarray, joy_buttons: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rebuild rospkg/kmu26_auv/src/joy2mavros.cpp output from recorded /joy."""
    if joy_t.size == 0 or joy_axes.size == 0:
        return np.empty(0, dtype=float), np.empty((0, 18), dtype=float)
    axes = np.asarray(joy_axes, dtype=float)
    if axes.ndim == 1:
        axes = axes.reshape((-1, 1))
    count = min(joy_t.size, axes.shape[0])
    if count <= 1:
        return np.empty(0, dtype=float), np.empty((0, 18), dtype=float)
    axes = axes[:count]
    buttons = np.asarray(joy_buttons, dtype=float)
    if buttons.size:
        if buttons.ndim == 1:
            buttons = buttons.reshape((-1, 1))
        buttons = buttons[: min(count, buttons.shape[0])]

    channels = np.full((count, 18), PWM_CENTER, dtype=float)
    channels[:, 3] = PWM_CENTER + (-axis_column(axes, 2)) * RC_OVERRIDE_SPAN
    channels[:, 2] = PWM_CENTER + axis_column(axes, 3) * RC_OVERRIDE_SPAN
    channels[:, 5] = PWM_CENTER + (-axis_column(axes, 0)) * RC_OVERRIDE_SPAN
    channels[:, 4] = PWM_CENTER + axis_column(axes, 1) * RC_OVERRIDE_SPAN

    led_pwm = PWM_CENTER
    channels[:, 8] = led_pwm
    if buttons.size and buttons.shape[0] >= count:
        b5 = button_column(buttons, 5)
        b6 = button_column(buttons, 6)
        for i in range(1, count):
            if b6[i] == 1.0 and b6[i - 1] != 1.0:
                if b5[i] == 1.0:
                    led_pwm = max(1100.0, led_pwm - 100.0)
                else:
                    led_pwm = min(1800.0, led_pwm + 100.0)
            channels[i, 8] = led_pwm

    # The C++ node stores the first message and returns without publishing.
    return np.asarray(joy_t[:count], dtype=float)[1:], channels[1:]


def euler_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def quat_to_yaw_wxyz(quat: np.ndarray) -> float:
    w, x, y, z = np.asarray(quat, dtype=float)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def quat_to_rpy_wxyz(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quat, dtype=float)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=np.float64)


def wrap_angle(angle: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    if values.size == 0 or samples <= 1:
        return values.copy()
    samples = min(samples, max(1, values.size // 2))
    kernel = np.ones(samples, dtype=float) / float(samples)
    padded = np.pad(values.astype(float), (samples // 2, samples - 1 - samples // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def derivative(t: np.ndarray, y: np.ndarray) -> np.ndarray:
    if t.size < 3:
        return np.zeros_like(y)
    return np.gradient(y.reshape(-1)) / np.maximum(np.gradient(t.reshape(-1)), 1e-6)


def resample_matrix(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    if src_t.size == 0 or src_v.size == 0:
        return np.empty((0, src_v.shape[1] if src_v.ndim == 2 else 1), dtype=float)
    values = np.asarray(src_v, dtype=float)
    if values.ndim == 1:
        values = values.reshape((-1, 1))
    return np.column_stack([np.interp(dst_t, src_t, values[:, idx]) for idx in range(values.shape[1])])


def fit_gain_offset(x: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    finite = np.isfinite(x) & np.isfinite(y)
    x = x[finite]
    y = y[finite]
    if x.size < 12 or np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return {"count": int(x.size), "gain": None, "offset": None, "r2": None}
    A = np.column_stack([np.ones_like(x), x])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ beta
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return {
        "count": int(x.size),
        "offset": float(beta[0]),
        "gain": float(beta[1]),
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None,
        "residual_std": float(np.std(y - pred)),
    }


def compare_scalar_with_lag(
    real_t: np.ndarray,
    real_y: np.ndarray,
    sim_t: np.ndarray,
    sim_y: np.ndarray,
    *,
    crop_start_s: float = 5.0,
    lag_min_s: float = -2.0,
    lag_max_s: float = 2.0,
    lag_step_s: float = 0.05,
    remove_median_offset: bool = False,
    sim_valid_t: np.ndarray | None = None,
    sim_valid: np.ndarray | None = None,
) -> dict[str, Any]:
    real_t = np.asarray(real_t, dtype=float).reshape(-1)
    real_y = np.asarray(real_y, dtype=float).reshape(-1)
    sim_t = np.asarray(sim_t, dtype=float).reshape(-1)
    sim_y = np.asarray(sim_y, dtype=float).reshape(-1)
    if real_t.size < 12 or sim_t.size < 12:
        return {"count": 0}

    best: dict[str, Any] | None = None
    for lag in np.arange(lag_min_s, lag_max_s + 1e-9, lag_step_s):
        query_t = real_t - float(lag)
        mask = (
            (real_t >= crop_start_s)
            & (query_t >= sim_t[0])
            & (query_t <= sim_t[-1])
            & np.isfinite(real_y)
        )
        if sim_valid_t is not None and sim_valid is not None:
            valid_t = np.asarray(sim_valid_t, dtype=float).reshape(-1)
            valid_v = np.asarray(sim_valid, dtype=float).reshape(-1)
            if valid_t.size >= 2 and valid_v.size == valid_t.size:
                valid_at_query = np.interp(query_t, valid_t, valid_v, left=0.0, right=0.0) >= 0.5
                mask &= valid_at_query
        if np.sum(mask) < 12:
            continue
        r = real_y[mask]
        s = np.interp(query_t[mask], sim_t, sim_y)
        finite = np.isfinite(r) & np.isfinite(s)
        r = r[finite]
        s = s[finite]
        if r.size < 12:
            continue
        residual = r - s
        offset = 0.0
        if remove_median_offset:
            offset = float(np.median(residual))
            residual = residual - offset
        rmse = float(np.sqrt(np.mean(residual * residual)))
        corr = None
        if np.std(r) > 1e-12 and np.std(s) > 1e-12:
            corr = float(np.corrcoef(r, s)[0, 1])
        candidate = {
            "count": int(r.size),
            "lag_s": float(lag),
            "rmse": rmse,
            "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)),
            "residual_std": float(np.std(residual)),
            "correlation": corr,
            "removed_median_offset": offset,
            "real": scalar_stats(real_t[mask][finite], r),
            "sim": scalar_stats(real_t[mask][finite], s),
            "gain_fit_real_from_sim": fit_gain_offset(s, r),
        }
        if best is None:
            best = candidate
        else:
            best_corr = -1.0 if best["correlation"] is None else abs(float(best["correlation"]))
            cand_corr = -1.0 if candidate["correlation"] is None else abs(float(candidate["correlation"]))
            if cand_corr > best_corr + 1e-9 or (
                abs(cand_corr - best_corr) <= 1e-9 and candidate["rmse"] < best["rmse"]
            ):
                best = candidate
    return best or {"count": 0}


def compare_vector(
    real_t: np.ndarray,
    real_v: np.ndarray,
    sim_t: np.ndarray,
    sim_v: np.ndarray,
    names: tuple[str, str, str] = ("x", "y", "z"),
    *,
    crop_start_s: float = 5.0,
    remove_median_offset: bool = False,
    sim_valid_t: np.ndarray | None = None,
    sim_valid: np.ndarray | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    if real_v.size == 0 or sim_v.size == 0:
        return {"count": 0}
    real_v = np.asarray(real_v, dtype=float).reshape((-1, 3))
    sim_v = np.asarray(sim_v, dtype=float).reshape((-1, 3))
    for idx, name in enumerate(names):
        out[name] = compare_scalar_with_lag(
            real_t,
            real_v[:, idx],
            sim_t,
            sim_v[:, idx],
            crop_start_s=crop_start_s,
            remove_median_offset=remove_median_offset,
            sim_valid_t=sim_valid_t,
            sim_valid=sim_valid,
        )
    return out


def compare_same_time_scalar(t: np.ndarray, truth: np.ndarray, measured: np.ndarray) -> dict[str, Any]:
    t = np.asarray(t, dtype=float).reshape(-1)
    truth = np.asarray(truth, dtype=float).reshape(-1)
    measured = np.asarray(measured, dtype=float).reshape(-1)
    n = min(t.size, truth.size, measured.size)
    if n < 12:
        return {"count": 0}
    truth = truth[:n]
    measured = measured[:n]
    finite = np.isfinite(truth) & np.isfinite(measured)
    truth = truth[finite]
    measured = measured[finite]
    if truth.size < 12:
        return {"count": 0}
    residual = measured - truth
    corr = None
    if np.std(truth) > 1e-12 and np.std(measured) > 1e-12:
        corr = float(np.corrcoef(truth, measured)[0, 1])
    return {
        "count": int(truth.size),
        "rmse": float(np.sqrt(np.mean(residual * residual))),
        "mae": float(np.mean(np.abs(residual))),
        "bias": float(np.mean(residual)),
        "residual_std": float(np.std(residual)),
        "correlation": corr,
        "truth": scalar_stats(t[:n][finite], truth),
        "measured": scalar_stats(t[:n][finite], measured),
        "gain_fit_measured_from_truth": fit_gain_offset(truth, measured),
    }


def compare_same_time_vector(
    t: np.ndarray,
    truth: np.ndarray,
    measured: np.ndarray,
    names: tuple[str, str, str] = ("x", "y", "z"),
) -> dict[str, Any]:
    truth = np.asarray(truth, dtype=float)
    measured = np.asarray(measured, dtype=float)
    if truth.size == 0 or measured.size == 0:
        return {"count": 0}
    truth = truth.reshape((-1, 3))
    measured = measured.reshape((-1, 3))
    return {name: compare_same_time_scalar(t, truth[:, idx], measured[:, idx]) for idx, name in enumerate(names)}


@dataclass
class RealSeries:
    name: str
    duration_s: float
    rc_override_t: np.ndarray
    rc_override: np.ndarray
    rc_out_t: np.ndarray
    rc_out: np.ndarray
    joy_t: np.ndarray
    joy_axes: np.ndarray
    joy_buttons: np.ndarray
    joy_rc_override_t: np.ndarray
    joy_rc_override: np.ndarray
    dvl_t: np.ndarray
    dvl_vel: np.ndarray
    imu_t: np.ndarray
    imu_rpy: np.ndarray
    imu_gyro: np.ndarray
    imu_accel: np.ndarray
    depth_t: np.ndarray
    depth: np.ndarray
    odom_t: np.ndarray
    odom_xyz: np.ndarray
    odom_rpy: np.ndarray
    odom_vel: np.ndarray
    static_pressure_t: np.ndarray
    static_pressure: np.ndarray
    atm_pressure_t: np.ndarray
    atm_pressure: np.ndarray


def extract_real_series(db_path: Path) -> RealSeries:
    data = read_bag(db_path)
    rc_override_t, rc_override = data.array("/mavros/rc/override:channels")
    rc_out_t, rc_out = data.array("/mavros/rc/out:channels")
    joy_t, joy_axes = data.array("/joy:axes")
    _, joy_buttons = data.array("/joy:buttons")
    joy_rc_override_t, joy_rc_override = joy_node_rc_override(joy_t, joy_axes, joy_buttons)
    dvl_t, dvl_vel = data.array("/dvl/twist:linear_m_s", 3)
    imu_t, imu_rpy = data.array("/mavros/imu/data:rpy_rad", 3)
    _, imu_gyro = data.array("/mavros/imu/data:gyro_rad_s", 3)
    _, imu_accel = data.array("/mavros/imu/data:accel_m_s2", 3)
    depth_t, depth = data.array("/depth/pose:depth_positive_m")
    odom_t, odom_xyz = data.array("/odometry/filtered:xyz_m", 3)
    _, odom_rpy = data.array("/odometry/filtered:rpy_rad", 3)
    _, odom_vel = data.array("/odometry/filtered:linear_m_s", 3)
    static_pressure_t, static_pressure = data.array("/mavros/imu/static_pressure:pressure_pa")
    atm_pressure_t, atm_pressure = data.array("/mavros/imu/atm_pressure:pressure_pa")
    return RealSeries(
        name=data.name,
        duration_s=data.duration_s,
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


class OfflineUuvReplay:
    def __init__(
        self,
        *,
        scene: Path,
        profile_name: str,
        fluid_model: str,
        thruster_dt_mode: str = "current-code",
        rc_out_scale: float = 1.0,
        disable_thruster_perf: bool = False,
        profile_overrides: dict[str, Any] | None = None,
        thruster_param_overrides: dict[str, Any] | None = None,
    ) -> None:
        self.scene = Path(scene)
        self.profile_name = canonical_profile_name(profile_name)
        self.fluid_model = str(fluid_model)
        self.thruster_dt_mode = thruster_dt_mode
        self.rc_out_scale = float(rc_out_scale)
        self.disable_thruster_perf = bool(disable_thruster_perf)
        self.thruster_param_overrides = thruster_param_overrides or {}

        profiles, warning = load_sim_profiles(PROFILE_PATH)
        if warning:
            print(warning, flush=True)
        self.sim_profile = build_sim_profile(profiles, self.profile_name)
        if profile_overrides:
            self.sim_profile.update(profile_overrides)
        self.active_thruster_voltage = float(self.sim_profile.get("thruster_voltage", 16.0))
        self.perf_cfg, perf_msg = load_thruster_performance(THRUSTER_PERF_PATH, self.active_thruster_voltage)
        if self.disable_thruster_perf:
            self.perf_cfg = type(self.perf_cfg)(
                False,
                self.active_thruster_voltage,
                None,
                np.array([], dtype=np.float64),
                np.array([], dtype=np.float64),
            )
            print("[thruster perf] disabled for offline replay", flush=True)
        elif perf_msg:
            print(perf_msg, flush=True)

        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        self.scene_fluid_density = float(self.model.opt.density)
        self.scene_fluid_viscosity = float(self.model.opt.viscosity)
        self.applied_mujoco_fluidcoef_scale = None
        fluidcoef_scale = self.sim_profile.get("mujoco_fluidcoef_scale")
        if isinstance(fluidcoef_scale, list) and len(fluidcoef_scale) == 5:
            try:
                scale = np.clip(np.asarray([float(v) for v in fluidcoef_scale], dtype=np.float64), 0.0, 10.0)
            except (TypeError, ValueError):
                scale = None
            if scale is not None and np.all(np.isfinite(scale)):
                fluid_geom_mask = (self.model.geom_fluid[:, 0] > 0.5) & np.any(
                    np.abs(self.model.geom_fluid[:, 1:6]) > 1e-12,
                    axis=1,
                )
                if np.any(fluid_geom_mask):
                    self.model.geom_fluid[fluid_geom_mask, 1:6] *= scale.reshape(1, 5)
                    self.applied_mujoco_fluidcoef_scale = scale.copy()
                    print(
                        "[physics] MuJoCo fluidcoef scale applied offline: "
                        f"count={int(np.sum(fluid_geom_mask))}, "
                        f"scale={np.array2string(scale, precision=3)}",
                        flush=True,
                    )
        mujoco.mj_forward(self.model, self.data)

        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if self.base_id < 0:
            raise RuntimeError("base_link body not found")

        self.act = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.model.nu)
        }
        self.ctrlrange = self.model.actuator_ctrlrange.copy()
        self.hydro_cfg = build_hydrodynamics_config(self.sim_profile, perf_force_max=self.perf_cfg.force_max)
        self.use_custom_hydrodynamics = self.fluid_model == "legacy"
        if self.use_custom_hydrodynamics:
            self.model.opt.density = 0.0
            self.model.opt.viscosity = 0.0

        self.body_children = [[] for _ in range(self.model.nbody)]
        for body_idx in range(1, self.model.nbody):
            parent_idx = int(self.model.body_parentid[body_idx])
            if 0 <= parent_idx < self.model.nbody:
                self.body_children[parent_idx].append(body_idx)

        self.apply_body_component_distribution()
        self.cob_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "cob_site")
        self.cob_longitudinal_offset = float(self.sim_profile.get("cob_x_offset", 0.0))
        self.cob_vertical_offset = float(self.sim_profile.get("cob_z_offset", 0.0))
        self.align_cob_to_com_with_offset()
        mujoco.mj_forward(self.model, self.data)

        self.sensor_ids = {}
        for name in ("imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude", "depth_pos"):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid >= 0:
                self.sensor_ids[name] = sid
        self.site_ids = {
            "bar30": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site"),
            "dvl": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site"),
            "imu": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_site"),
            "water_surface": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "water_surface_ref"),
            "pool_floor": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "pool_floor_ref"),
        }
        self.bmj_to_flu = np.eye(3, dtype=np.float64)
        self.last_command_mode = COMMAND_MODE_NONE
        self.last_command8 = np.zeros(8, dtype=np.float64)
        self.last_direct_command = np.zeros(4, dtype=np.float64)
        command_axis_gain_cfg = self.sim_profile.get("command_axis_gain", {})
        if not isinstance(command_axis_gain_cfg, dict):
            command_axis_gain_cfg = {}
        self.command_axis_gain = {
            "surge": float(np.clip(float(command_axis_gain_cfg.get("surge", 1.0)), 0.05, 5.0)),
            "sway": float(np.clip(float(command_axis_gain_cfg.get("sway", 1.0)), 0.05, 5.0)),
            "yaw": float(np.clip(float(command_axis_gain_cfg.get("yaw", 1.0)), 0.05, 5.0)),
            "heave": float(np.clip(float(command_axis_gain_cfg.get("heave", 1.0)), 0.05, 5.0)),
        }
        self.water_surface_z = 0.0
        if self.site_ids["water_surface"] >= 0:
            self.water_surface_z = float(self.data.site_xpos[self.site_ids["water_surface"]][2])
        self.pool_floor_z = float("nan")
        if self.site_ids["pool_floor"] >= 0:
            self.pool_floor_z = float(self.data.site_xpos[self.site_ids["pool_floor"]][2])

        self.rho = self.scene_fluid_density
        self.g = abs(float(self.model.opt.gravity[2]))
        self.vehicle_mass = self.body_subtree_mass(self.base_id)
        if self.vehicle_mass <= 1e-9:
            self.vehicle_mass = float(self.model.body_mass[self.base_id])
        self.neutral_volume = self.vehicle_mass / max(self.rho, 1e-6)

        self.ver_names = list(PHYSICAL_VERTICAL_THRUSTERS)
        self.yaw_names = list(PHYSICAL_YAW_THRUSTERS)
        self.horiz_order = list(ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER)
        self.all_thruster_names = list(PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS)
        self.thr_state = {name: 0.0 for name in self.all_thruster_names}
        self.thr_target = {name: 0.0 for name in self.all_thruster_names}
        self.thruster_scale = {name: 1.0 for name in self.all_thruster_names}
        self.thruster_signed_scale = {
            name: {"positive": 1.0, "negative": 1.0}
            for name in self.all_thruster_names
        }
        self.thruster_force_cmd = {name: 0.0 for name in self.all_thruster_names}
        self.thruster_global = {
            "deadzone": 0.05,
            "tau_up": 0.12,
            "tau_down": 0.18,
            "reverse_asymmetry": 0.75,
            "command_limit": 0.65,
            "gain_scale_all": 1.0,
            "positive_gain_scale": 1.0,
            "negative_gain_scale": 1.0,
            "pwm_center_us": 1500.0,
            "pwm_span_us": 400.0,
            "forward_poly": [0.0, 3.5, 7.0, 12.0],
            "reverse_poly": [0.0, 2.8, 5.5, 9.5],
        }
        self.load_thruster_params()

        self.horiz_alloc = np.zeros((3, len(self.horiz_order)), dtype=np.float64)
        self.horiz_pinv = np.zeros((len(self.horiz_order), 3), dtype=np.float64)
        self.build_horizontal_allocator()

        self.half_height = self.hydro_cfg.half_height
        self.buoyancy_model = self.hydro_cfg.buoyancy_model
        self.buoyancy_scale = self.hydro_cfg.buoyancy_scale
        self.buoyancy_slope_scale = self.hydro_cfg.buoyancy_slope_scale
        self.surface_heave_damping = self.hydro_cfg.surface_heave_damping
        self.heave_damping_scale = self.hydro_cfg.heave_damping_scale
        self.full_heave_damping = self.surface_heave_damping * self.heave_damping_scale
        self.cob_torque_scale = self.hydro_cfg.cob_torque_scale
        self.buoyancy_point_blend = self.hydro_cfg.buoyancy_point_blend
        self.thruster_force_max = self.hydro_cfg.thruster_force_max
        self.vertical_thruster_gain_scale = float(getattr(self.hydro_cfg, "vertical_thruster_gain_scale", 1.0))
        self.spin_gain = self.hydro_cfg.spin_gain
        self.yaw_torque_scale = float(max(self.hydro_cfg.yaw_torque_scale, 0.0))
        self.added_mass_diag = self.hydro_cfg.added_mass_diag.astype(np.float64, copy=True)
        self.linear_damping_diag = self.hydro_cfg.linear_damping_diag.astype(np.float64, copy=True)
        self.quadratic_damping_diag = self.hydro_cfg.quadratic_damping_diag.astype(np.float64, copy=True)
        self.air_linear_damping_diag = self.hydro_cfg.air_linear_damping_diag.astype(np.float64, copy=True)
        self.water_current_world = self.hydro_cfg.water_current_world.astype(np.float64, copy=True)
        self.body_components = self.hydro_cfg.body_components
        self.buoyancy_points = self.hydro_cfg.buoyancy_points
        if not self.use_custom_hydrodynamics:
            self.added_mass_diag[:] = 0.0
            self.linear_damping_diag[:] = 0.0
            self.quadratic_damping_diag[:] = 0.0
            self.air_linear_damping_diag[:] = 0.0

        self.thruster_loop_hz = 100.0
        self.thruster_loop_dt = 1.0 / self.thruster_loop_hz
        self.next_thruster_sim_time = -1.0
        self.prev_rel_nu_body = np.zeros(6, dtype=np.float64)
        self.thruster_extra_torque_world = np.zeros(3, dtype=np.float64)
        self.last_terms: dict[str, Any] = {}

    @staticmethod
    def normalize(vec: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(vec))
        if n < 1e-9:
            return vec
        return vec / n

    def body_subtree_mass(self, root_body_id: int) -> float:
        total = 0.0
        stack = [int(root_body_id)]
        while stack:
            bid = stack.pop()
            total += float(self.model.body_mass[bid])
            stack.extend(self.body_children[bid])
        return total

    def component_self_inertia_diag(self, component) -> np.ndarray:
        a, b, c = component.size.astype(np.float64, copy=False)
        if component.shape == "box":
            return np.array(
                [
                    component.mass * (b * b + c * c) / 3.0,
                    component.mass * (a * a + c * c) / 3.0,
                    component.mass * (a * a + b * b) / 3.0,
                ],
                dtype=np.float64,
            )
        return np.array(
            [
                component.mass * (b * b + c * c) / 5.0,
                component.mass * (a * a + c * c) / 5.0,
                component.mass * (a * a + b * b) / 5.0,
            ],
            dtype=np.float64,
        )

    def apply_body_component_distribution(self) -> None:
        components = self.hydro_cfg.body_components
        if not components:
            return
        total_mass = float(sum(component.mass for component in components))
        if total_mass <= 1e-9:
            return
        inertia_scale = np.array(self.sim_profile.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=np.float64)
        if inertia_scale.shape != (3,) or not np.all(np.isfinite(inertia_scale)):
            inertia_scale = np.ones(3, dtype=np.float64)
        inertia_scale = np.clip(inertia_scale, 1e-6, 100.0)
        composite_com = sum(component.mass * component.mass_pos for component in components) / total_mass
        composite_inertia = np.zeros(3, dtype=np.float64)
        for component in components:
            offset = component.mass_pos - composite_com
            parallel_axis = component.mass * np.array(
                [
                    offset[1] * offset[1] + offset[2] * offset[2],
                    offset[0] * offset[0] + offset[2] * offset[2],
                    offset[0] * offset[0] + offset[1] * offset[1],
                ],
                dtype=np.float64,
            )
            composite_inertia += self.component_self_inertia_diag(component) + parallel_axis
        composite_inertia *= inertia_scale
        self.model.body_mass[self.base_id] = total_mass
        self.model.body_ipos[self.base_id, :] = composite_com
        self.model.body_inertia[self.base_id, :] = np.maximum(composite_inertia, 1e-6)
        if hasattr(mujoco, "mj_setConst"):
            mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def align_cob_to_com_with_offset(self) -> None:
        if self.cob_site_id < 0:
            return
        self.model.site_pos[self.cob_site_id][0] = float(self.model.body_ipos[self.base_id][0] + self.cob_longitudinal_offset)
        self.model.site_pos[self.cob_site_id][2] = float(self.model.body_ipos[self.base_id][2] + self.cob_vertical_offset)

    def load_thruster_params(self) -> None:
        if not THRUSTER_PARAMS_PATH.exists():
            return
        try:
            payload = json.loads(THRUSTER_PARAMS_PATH.read_text())
        except (OSError, json.JSONDecodeError):
            return
        global_cfg = payload.get("global", {})
        if isinstance(global_cfg, dict):
            global_cfg = dict(global_cfg)
            override_global = self.thruster_param_overrides.get("global")
            if isinstance(override_global, dict):
                global_cfg.update(override_global)
        global_gain_scale = 1.0
        global_positive_scale = 1.0
        global_negative_scale = 1.0
        if isinstance(global_cfg, dict):
            for key in (
                "deadzone",
                "tau_up",
                "tau_down",
                "reverse_asymmetry",
                "command_limit",
                "pwm_center_us",
                "pwm_span_us",
            ):
                value = global_cfg.get(key)
                if isinstance(value, (int, float)):
                    self.thruster_global[key] = float(value)
            scale_all = global_cfg.get("gain_scale_all")
            if isinstance(scale_all, (int, float)):
                global_gain_scale = float(np.clip(float(scale_all), 0.1, 20.0))
                self.thruster_global["gain_scale_all"] = global_gain_scale
            positive_scale = global_cfg.get("positive_gain_scale")
            if isinstance(positive_scale, (int, float)):
                global_positive_scale = float(np.clip(float(positive_scale), 0.0, 20.0))
                self.thruster_global["positive_gain_scale"] = global_positive_scale
            negative_scale = global_cfg.get("negative_gain_scale")
            if isinstance(negative_scale, (int, float)):
                global_negative_scale = float(np.clip(float(negative_scale), 0.0, 20.0))
                self.thruster_global["negative_gain_scale"] = global_negative_scale
            for key in ("forward_poly", "reverse_poly"):
                coeffs = global_cfg.get(key)
                if isinstance(coeffs, list) and coeffs:
                    try:
                        self.thruster_global[key] = [float(item) for item in coeffs]
                    except (TypeError, ValueError):
                        pass
        per_thruster = payload.get("per_thruster", {})
        override_per_thruster = self.thruster_param_overrides.get("per_thruster")
        if isinstance(per_thruster, dict) and isinstance(override_per_thruster, dict):
            per_thruster = {
                name: ({**cfg, **override_per_thruster.get(name, {})} if isinstance(cfg, dict) else cfg)
                for name, cfg in per_thruster.items()
            }
        if not isinstance(per_thruster, dict):
            return
        for name in self.all_thruster_names:
            cfg = per_thruster.get(name, {})
            if not isinstance(cfg, dict):
                cfg = {}
            gain = cfg.get("gain_scale", 1.0)
            if not isinstance(gain, (int, float)):
                gain = 1.0
            self.thruster_scale[name] = float(np.clip(float(gain) * global_gain_scale, 0.1, 20.0))
            positive_gain = cfg.get("positive_gain_scale", 1.0)
            negative_gain = cfg.get("negative_gain_scale", 1.0)
            if not isinstance(positive_gain, (int, float)):
                positive_gain = 1.0
            if not isinstance(negative_gain, (int, float)):
                negative_gain = 1.0
            self.thruster_signed_scale[name]["positive"] = float(
                np.clip(float(positive_gain) * global_positive_scale, 0.0, 20.0)
            )
            self.thruster_signed_scale[name]["negative"] = float(
                np.clip(float(negative_gain) * global_negative_scale, 0.0, 20.0)
            )

    def build_horizontal_allocator(self) -> None:
        com_body = self.model.body_ipos[self.base_id].copy()
        alloc = np.zeros((3, len(self.horiz_order)), dtype=np.float64)
        for i, name in enumerate(self.horiz_order):
            aid = self.act[name]
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            fdir = self.normalize(self.model.actuator_gear[aid, :3].copy())
            r = self.model.site_pos[sid].copy() - com_body
            tau = np.cross(r, fdir)
            alloc[:, i] = np.array([fdir[0], fdir[1], tau[2]], dtype=np.float64)
        row_scale = np.sum(np.abs(alloc), axis=1)
        row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
        self.horiz_pinv = np.linalg.pinv(alloc / row_scale[:, None])
        self.horiz_alloc = alloc

    def mix_horizontal_thrusters(self, fwd_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
        wrench_cmd = np.array([fwd_cmd, sway_cmd, yaw_cmd], dtype=np.float64)
        u = self.horiz_pinv @ wrench_cmd
        max_abs = float(np.max(np.abs(u)))
        if max_abs > 1.0:
            u /= max_abs
        return np.clip(u, -1.0, 1.0)

    def sensor_value(self, name: str) -> np.ndarray | None:
        sid = self.sensor_ids.get(name, -1)
        if sid < 0:
            return None
        adr = int(self.model.sensor_adr[sid])
        dim = int(self.model.sensor_dim[sid])
        return self.data.sensordata[adr : adr + dim].copy()

    def body_velocity_local(self) -> tuple[np.ndarray, np.ndarray]:
        vel6 = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, int(self.base_id), vel6, 1)
        return vel6[3:].copy(), vel6[:3].copy()

    def body_velocity_world(self) -> tuple[np.ndarray, np.ndarray]:
        vel6 = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, int(self.base_id), vel6, 0)
        return vel6[3:].copy(), vel6[:3].copy()

    def vector_from_site_to_body(self, site_id: int, value_site: np.ndarray | None) -> np.ndarray | None:
        if value_site is None:
            return None
        value = np.asarray(value_site, dtype=np.float64)
        if site_id < 0:
            return value.copy()
        try:
            base_rot = self.data.xmat[self.base_id].reshape(3, 3).copy()
            site_rot = self.data.site_xmat[site_id].reshape(3, 3).copy()
            return (base_rot.T @ site_rot) @ value
        except Exception:
            return value.copy()

    def dvl_velocity_body_from_sensor(
        self,
        dvl_vel_sensor: np.ndarray | None,
        gyro_body: np.ndarray | None,
    ) -> np.ndarray | None:
        if dvl_vel_sensor is None:
            return None
        vel_body = self.vector_from_site_to_body(self.site_ids["dvl"], dvl_vel_sensor)
        if vel_body is None:
            return None
        if gyro_body is not None and self.site_ids["dvl"] >= 0:
            try:
                base_rot = self.data.xmat[self.base_id].reshape(3, 3).copy()
                r_world = self.data.site_xpos[self.site_ids["dvl"]] - self.data.xpos[self.base_id]
                r_body = base_rot.T @ r_world
                vel_body = vel_body - np.cross(gyro_body, r_body)
            except Exception:
                pass
        return np.nan_to_num(vel_body, nan=0.0, posinf=0.0, neginf=0.0)

    def set_initial_state(self, real: RealSeries) -> None:
        depth0 = 0.45
        if real.depth.size:
            depth0 = float(real.depth[0])
        roll0 = pitch0 = yaw0 = 0.0
        if real.odom_rpy.size:
            roll0, pitch0, yaw0 = [float(v) for v in real.odom_rpy[0]]
        elif real.imu_rpy.size:
            roll0, pitch0, yaw0 = [float(v) for v in real.imu_rpy[0]]
        quat = euler_to_quat_wxyz(roll0, pitch0, yaw0)
        bar30_local_z = 0.0
        if self.site_ids["bar30"] >= 0:
            bar30_local_z = float(self.model.site_pos[self.site_ids["bar30"]][2])
        self.data.qpos[:7] = np.array([0.0, 0.0, self.water_surface_z - depth0 - bar30_local_z, *quat], dtype=np.float64)
        self.data.qvel[:] = 0.0
        self.prev_rel_nu_body[:] = 0.0
        for name in self.all_thruster_names:
            self.thr_state[name] = 0.0
            self.thr_target[name] = 0.0
            self.thruster_force_cmd[name] = 0.0
        self.next_thruster_sim_time = -1.0
        mujoco.mj_forward(self.model, self.data)

    def thruster_update_due(self) -> bool:
        sim_t = float(self.data.time)
        if self.next_thruster_sim_time < 0.0:
            self.next_thruster_sim_time = sim_t
        if sim_t + 1e-9 < self.next_thruster_sim_time:
            return False
        while sim_t + 1e-9 >= self.next_thruster_sim_time:
            self.next_thruster_sim_time += self.thruster_loop_dt
        return True

    def force_from_shaped_command(self, command_shaped: float, gain: float) -> float:
        if abs(command_shaped) <= 1e-9:
            return 0.0
        if self.perf_cfg.active:
            return float(self.perf_cfg.force_from_norm(command_shaped) * gain)
        magnitude = abs(command_shaped)
        if command_shaped >= 0.0:
            force_mag = scaled_polynomial_force(magnitude, self.thruster_global["forward_poly"], self.thruster_force_max)
            return float(force_mag * gain)
        reverse_force_max = self.thruster_force_max * float(np.clip(self.thruster_global["reverse_asymmetry"], 0.1, 1.5))
        force_mag = scaled_polynomial_force(magnitude, self.thruster_global["reverse_poly"], reverse_force_max)
        return float(-force_mag * gain)

    def update_thruster_forces(self, dt: float) -> None:
        self.thruster_extra_torque_world = np.zeros(3, dtype=np.float64)
        base_rot = self.data.xmat[self.base_id].reshape(3, 3)
        com_body = self.model.body_ipos[self.base_id].copy()
        deadzone = float(np.clip(self.thruster_global["deadzone"], 0.0, 0.95))
        tau_up = float(max(self.thruster_global["tau_up"], 1e-4))
        tau_down = float(max(self.thruster_global["tau_down"], 1e-4))
        command_limit = float(np.clip(self.thruster_global["command_limit"], deadzone + 1e-3, 1.0))
        for name in self.all_thruster_names:
            aid = self.act[name]
            lo, hi = self.ctrlrange[aid]
            gain = float(self.thruster_scale.get(name, 1.0))
            if name in self.ver_names:
                gain *= self.vertical_thruster_gain_scale
            target_norm = float(np.clip(self.thr_target[name], -1.0, 1.0))
            self.thr_state[name] = first_order_response(self.thr_state[name], target_norm, dt, tau_up, tau_down)
            shaped_cmd = shape_thruster_command(self.thr_state[name], deadzone, command_limit)
            signed = self.thruster_signed_scale.get(name, {"positive": 1.0, "negative": 1.0})
            direction_gain = signed["positive"] if shaped_cmd >= 0.0 else signed["negative"]
            force = float(np.clip(self.force_from_shaped_command(shaped_cmd, gain * float(direction_gain)), lo, hi))
            self.data.ctrl[aid] = force
            self.thruster_force_cmd[name] = force
            fdir = self.normalize(self.model.actuator_gear[aid, :3].copy())
            if self.yaw_torque_scale > 1.0 and name in PHYSICAL_YAW_THRUSTERS:
                sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
                if sid >= 0:
                    r_body = self.model.site_pos[sid].copy() - com_body
                    tau_body = np.cross(r_body, fdir * force)
                    extra_tau_body = np.array([0.0, 0.0, tau_body[2] * (self.yaw_torque_scale - 1.0)], dtype=np.float64)
                    self.thruster_extra_torque_world += base_rot @ extra_tau_body

    def apply_direct_command_targets(self, command: np.ndarray) -> None:
        raw_fwd, raw_sway, raw_yaw, raw_heave = [float(v) for v in command]
        fwd_cmd = float(np.clip(raw_fwd * self.command_axis_gain["surge"], -1.0, 1.0))
        sway_cmd = float(np.clip(raw_sway * self.command_axis_gain["sway"], -1.0, 1.0))
        yaw_cmd = float(np.clip(raw_yaw * self.command_axis_gain["yaw"], -1.0, 1.0))
        heave_cmd = float(np.clip(raw_heave * self.command_axis_gain["heave"], -1.0, 1.0))
        horiz_cmd = self.mix_horizontal_thrusters(fwd_cmd, sway_cmd, yaw_cmd)
        for name in self.all_thruster_names:
            self.thr_target[name] = 0.0
        for name in self.ver_names:
            self.thr_target[name] = heave_cmd
        for i, name in enumerate(self.horiz_order):
            self.thr_target[name] = float(horiz_cmd[i])

    def apply_rc_out_targets(self, rc_out_norm8: np.ndarray) -> None:
        for name in self.all_thruster_names:
            self.thr_target[name] = 0.0
        for idx, thr_name in enumerate(ARDUSUB_VECTORED_6DOF_SERVO_MAP):
            if idx >= rc_out_norm8.size:
                break
            sign = float(ARDUSUB_VECTORED_6DOF_SERVO_SIGNS[idx])
            self.thr_target[thr_name] = float(np.clip(rc_out_norm8[idx] * sign * self.rc_out_scale, -1.0, 1.0))

    def apply_underwater_wrench(self, dt: float) -> None:
        self.data.xfrc_applied[self.base_id, :] = 0.0
        com = self.data.xipos[self.base_id].copy()
        base_rot = self.data.xmat[self.base_id].reshape(3, 3)
        base_origin = self.data.xpos[self.base_id].copy()
        lin_vel_body, ang_vel_body = self.body_velocity_local()
        current_body = base_rot.T @ self.water_current_world
        rel_lin_vel_body = lin_vel_body - current_body
        rel_lin_vel_world = base_rot @ rel_lin_vel_body
        cob = self.data.site_xpos[self.cob_site_id].copy() if self.cob_site_id >= 0 else com
        depth = self.water_surface_z - float(base_origin[2])
        submerged = submerged_fraction(depth, self.half_height, self.buoyancy_model)
        buoyancy_submerged = submerged_fraction(depth * self.buoyancy_slope_scale, self.half_height, self.buoyancy_model)
        buoy_tau_world = np.zeros(3, dtype=np.float64)
        buoy_force_world = np.zeros(3, dtype=np.float64)
        buoy_point = cob.copy()

        if self.buoyancy_points:
            total_share = float(sum(point.share for point in self.buoyancy_points)) or float(len(self.buoyancy_points))
            weighted_submerged = 0.0
            weighted_buoyancy_submerged = 0.0
            weighted_point = np.zeros(3, dtype=np.float64)
            for point in self.buoyancy_points:
                share = point.share / max(total_share, 1e-9)
                point_local = point.pos.copy()
                point_local[0] += self.cob_longitudinal_offset
                point_local[2] += self.cob_vertical_offset
                point_world = base_origin + base_rot @ point_local
                point_depth = self.water_surface_z - float(point_world[2])
                point_submerged = submerged_fraction(point_depth, point.half_height, self.buoyancy_model)
                point_buoyancy_submerged = submerged_fraction(
                    point_depth * self.buoyancy_slope_scale,
                    point.half_height,
                    self.buoyancy_model,
                )
                point_buoyancy = self.rho * self.g * self.neutral_volume * share * point_buoyancy_submerged * self.buoyancy_scale
                point_force_world = np.array([0.0, 0.0, point_buoyancy], dtype=np.float64)
                buoy_force_world += point_force_world
                weighted_submerged += share * point_submerged
                weighted_buoyancy_submerged += share * point_buoyancy_submerged
                weighted_point += point_buoyancy * point_world
                if abs(self.cob_torque_scale) > 1e-9:
                    buoy_tau_world += np.cross(point_world - com, point_force_world) * self.cob_torque_scale
            submerged = float(np.clip(weighted_submerged, 0.0, 1.0))
            buoyancy_submerged = float(np.clip(weighted_buoyancy_submerged, 0.0, 1.0))
            total_buoyancy = float(np.linalg.norm(buoy_force_world))
            buoy_point = weighted_point / total_buoyancy if total_buoyancy > 1e-9 else cob.copy()
        elif self.body_components:
            total_share = float(sum(component.buoyancy_share for component in self.body_components))
            if total_share <= 1e-9:
                total_share = float(sum(component.mass for component in self.body_components))
            weighted_submerged = 0.0
            weighted_buoyancy_submerged = 0.0
            weighted_point = np.zeros(3, dtype=np.float64)
            for component in self.body_components:
                share = component.buoyancy_share / max(total_share, 1e-9)
                point_local = component.buoyancy_pos.copy()
                point_local[0] += self.cob_longitudinal_offset
                point_local[2] += self.cob_vertical_offset
                point_world = base_origin + base_rot @ point_local
                component_half_height = float(max(component.size[2], 1e-4))
                component_depth = self.water_surface_z - float(point_world[2])
                component_submerged = submerged_fraction(component_depth, component_half_height, self.buoyancy_model)
                component_buoyancy_submerged = submerged_fraction(
                    component_depth * self.buoyancy_slope_scale,
                    component_half_height,
                    self.buoyancy_model,
                )
                component_buoyancy = self.rho * self.g * self.neutral_volume * share * component_buoyancy_submerged * self.buoyancy_scale
                component_force_world = np.array([0.0, 0.0, component_buoyancy], dtype=np.float64)
                buoy_force_world += component_force_world
                weighted_submerged += share * component_submerged
                weighted_buoyancy_submerged += share * component_buoyancy_submerged
                weighted_point += component_buoyancy * point_world
                if abs(self.cob_torque_scale) > 1e-9:
                    buoy_tau_world += np.cross(point_world - com, component_force_world) * self.cob_torque_scale
            submerged = float(np.clip(weighted_submerged, 0.0, 1.0))
            buoyancy_submerged = float(np.clip(weighted_buoyancy_submerged, 0.0, 1.0))
            total_buoyancy = float(np.linalg.norm(buoy_force_world))
            buoy_point = weighted_point / total_buoyancy if total_buoyancy > 1e-9 else cob.copy()
        else:
            buoyancy_blend = self.buoyancy_point_blend * buoyancy_submerged
            buoy_point = ((1.0 - buoyancy_blend) * com) + (buoyancy_blend * cob)
            buoy = self.rho * self.g * self.neutral_volume * buoyancy_submerged * self.buoyancy_scale
            buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)
            if abs(self.cob_torque_scale) > 1e-9:
                buoy_tau_world = np.cross(buoy_point - com, buoy_force_world) * self.cob_torque_scale

        self.data.xfrc_applied[self.base_id, 0:3] += buoy_force_world
        self.data.xfrc_applied[self.base_id, 3:6] += buoy_tau_world
        surface_force_world = np.zeros(3, dtype=np.float64)
        surface_weight = max(0.0, 4.0 * submerged * (1.0 - submerged))
        if self.surface_heave_damping > 1e-9 and surface_weight > 1e-9:
            surface_force_world = np.array(
                [0.0, 0.0, -self.surface_heave_damping * surface_weight * float(rel_lin_vel_world[2])],
                dtype=np.float64,
            )
            self.data.xfrc_applied[self.base_id, 0:3] += surface_force_world

        if (not self.use_custom_hydrodynamics) and self.full_heave_damping > 1e-9 and submerged > 1e-9:
            self.data.xfrc_applied[self.base_id, 0:3] += np.array(
                [0.0, 0.0, -self.full_heave_damping * submerged * float(rel_lin_vel_world[2])],
                dtype=np.float64,
            )

        hydro_wrench_body = np.zeros(6, dtype=np.float64)
        if self.use_custom_hydrodynamics:
            nu_rel_body = np.concatenate((rel_lin_vel_body, ang_vel_body))
            rel_acc_body = (nu_rel_body - self.prev_rel_nu_body) / max(dt, 1e-6) if dt > 0.0 else np.zeros(6, dtype=np.float64)
            self.prev_rel_nu_body = nu_rel_body.copy()
            immersed_added_mass = self.added_mass_diag * submerged
            immersed_linear_damping = self.air_linear_damping_diag + submerged * (
                self.linear_damping_diag - self.air_linear_damping_diag
            )
            immersed_quadratic_damping = self.quadratic_damping_diag * submerged
            immersed_linear_damping[2] *= self.heave_damping_scale
            immersed_quadratic_damping[2] *= self.heave_damping_scale
            hydro_wrench_body -= immersed_added_mass * rel_acc_body
            hydro_wrench_body -= added_mass_coriolis(immersed_added_mass, nu_rel_body) @ nu_rel_body
            hydro_wrench_body -= immersed_linear_damping * nu_rel_body
            hydro_wrench_body -= immersed_quadratic_damping * np.abs(nu_rel_body) * nu_rel_body
            self.data.xfrc_applied[self.base_id, 0:3] += base_rot @ hydro_wrench_body[:3]
            self.data.xfrc_applied[self.base_id, 3:6] += base_rot @ hydro_wrench_body[3:]
        else:
            self.prev_rel_nu_body[:] = 0.0
        self.data.xfrc_applied[self.base_id, 3:6] += self.thruster_extra_torque_world

        thruster_force_world = np.zeros(3, dtype=np.float64)
        thruster_tau_world = np.zeros(3, dtype=np.float64)
        for name in self.all_thruster_names:
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            aid = self.act[name]
            fdir = self.normalize(self.model.actuator_gear[aid, :3].copy())
            force_world = base_rot @ (fdir * float(self.data.ctrl[aid]))
            thruster_force_world += force_world
            if sid >= 0:
                thruster_tau_world += np.cross(self.data.site_xpos[sid].copy() - com, force_world)
        thruster_tau_world += self.thruster_extra_torque_world

        self.last_terms = {
            "submerged": float(submerged),
            "buoyancy_submerged": float(buoyancy_submerged),
            "buoy_force_world": buoy_force_world.tolist(),
            "buoy_tau_world": buoy_tau_world.tolist(),
            "surface_force_world": surface_force_world.tolist(),
            "hydro_wrench_body": hydro_wrench_body.tolist(),
            "thruster_force_world": thruster_force_world.tolist(),
            "thruster_tau_world": thruster_tau_world.tolist(),
            "xfrc_world": self.data.xfrc_applied[self.base_id, :].copy().tolist(),
            "thruster_ctrl": {name: float(self.data.ctrl[self.act[name]]) for name in self.all_thruster_names},
        }

    def record_state(self, out: dict[str, list]) -> None:
        t = float(self.data.time)
        base_pos = self.data.xpos[self.base_id].copy()
        base_rot = self.data.xmat[self.base_id].reshape(3, 3).copy()
        quat = self.data.xquat[self.base_id].copy()
        lin_world, ang_world = self.body_velocity_world()
        lin_body, ang_body = self.body_velocity_local()
        dvl_raw = self.sensor_value("dvl_vel_body")
        gyro_raw = self.sensor_value("imu_gyro")
        accel_raw = self.sensor_value("imu_acc")
        gyro = self.vector_from_site_to_body(self.site_ids["imu"], gyro_raw)
        accel = self.vector_from_site_to_body(self.site_ids["imu"], accel_raw)
        dvl = self.dvl_velocity_body_from_sensor(dvl_raw, gyro)
        altitude = self.sensor_value("dvl_altitude")
        bar30_id = self.site_ids["bar30"]
        bar30_z = float(self.data.site_xpos[bar30_id][2]) if bar30_id >= 0 else float(base_pos[2])
        base_depth = max(0.0, self.water_surface_z - float(base_pos[2]))
        depth = max(0.0, self.water_surface_z - bar30_z)
        pressure = 101325.0 + self.rho * 9.80665 * depth
        out["t"].append(t)
        out["base_xyz"].append(base_pos.tolist())
        out["base_rpy"].append(quat_to_rpy_wxyz(quat).tolist())
        out["base_yaw"].append(quat_to_yaw_wxyz(quat))
        out["vel_world"].append(lin_world.tolist())
        out["ang_world"].append(ang_world.tolist())
        out["vel_body"].append(lin_body.tolist())
        out["ang_body"].append(ang_body.tolist())
        out["dvl_vel_raw_sensor"].append((dvl_raw if dvl_raw is not None else np.zeros(3)).tolist())
        out["dvl_vel"].append((dvl if dvl is not None else np.zeros(3)).tolist())
        out["imu_gyro_raw_sensor"].append((gyro_raw if gyro_raw is not None else np.zeros(3)).tolist())
        out["imu_gyro"].append((gyro if gyro is not None else np.zeros(3)).tolist())
        out["imu_accel_raw_sensor"].append((accel_raw if accel_raw is not None else np.zeros(3)).tolist())
        out["imu_accel"].append((accel if accel is not None else np.zeros(3)).tolist())
        out["base_depth_truth"].append(base_depth)
        out["depth"].append(depth)
        out["pressure"].append(pressure)
        out["dvl_altitude"].append(float(altitude[0]) if altitude is not None and altitude.size else float("nan"))
        out["command_mode"].append(float(self.last_command_mode))
        out["command8_norm"].append(self.last_command8.tolist())
        out["command_direct4"].append(self.last_direct_command.tolist())
        out["contact_count"].append(int(self.data.ncon))
        terms = self.last_terms
        for key in (
            "submerged",
            "buoyancy_submerged",
            "buoy_force_world",
            "buoy_tau_world",
            "surface_force_world",
            "hydro_wrench_body",
            "thruster_force_world",
            "thruster_tau_world",
            "xfrc_world",
        ):
            out[key].append(terms.get(key, 0.0 if key in {"submerged", "buoyancy_submerged"} else [0.0, 0.0, 0.0]))
        out["qfrc_passive_norm"].append(float(np.linalg.norm(self.data.qfrc_passive)))
        out["qfrc_actuator_norm"].append(float(np.linalg.norm(self.data.qfrc_actuator)))
        out["qfrc_applied_norm"].append(float(np.linalg.norm(self.data.qfrc_applied)))
        out["qfrc_bias_norm"].append(float(np.linalg.norm(self.data.qfrc_bias)))
        out["qacc_norm"].append(float(np.linalg.norm(self.data.qacc)))
        for name in self.all_thruster_names:
            out[f"thr_{name}_target"].append(float(self.thr_target[name]))
            out[f"thr_{name}_state"].append(float(self.thr_state[name]))
            out[f"thr_{name}_force"].append(float(self.thruster_force_cmd[name]))

    def run(self, real: RealSeries, *, command_source: str, max_duration_s: float | None, record_dt_s: float) -> dict[str, np.ndarray]:
        self.set_initial_state(real)
        duration = real.duration_s
        if max_duration_s is not None:
            duration = min(duration, float(max_duration_s))
        dt = float(self.model.opt.timestep)
        record_next_t = 0.0
        out: dict[str, list] = {
            "t": [],
            "base_xyz": [],
            "base_rpy": [],
            "base_yaw": [],
            "vel_world": [],
            "ang_world": [],
            "vel_body": [],
            "ang_body": [],
            "dvl_vel_raw_sensor": [],
            "dvl_vel": [],
            "imu_gyro_raw_sensor": [],
            "imu_gyro": [],
            "imu_accel_raw_sensor": [],
            "imu_accel": [],
            "base_depth_truth": [],
            "depth": [],
            "pressure": [],
            "dvl_altitude": [],
            "command_mode": [],
            "command8_norm": [],
            "command_direct4": [],
            "contact_count": [],
            "submerged": [],
            "buoyancy_submerged": [],
            "buoy_force_world": [],
            "buoy_tau_world": [],
            "surface_force_world": [],
            "hydro_wrench_body": [],
            "thruster_force_world": [],
            "thruster_tau_world": [],
            "xfrc_world": [],
            "qfrc_passive_norm": [],
            "qfrc_actuator_norm": [],
            "qfrc_applied_norm": [],
            "qfrc_bias_norm": [],
            "qacc_norm": [],
        }
        for name in self.all_thruster_names:
            out[f"thr_{name}_target"] = []
            out[f"thr_{name}_state"] = []
            out[f"thr_{name}_force"] = []

        rc_override_norm = normalize_rc_override(real.rc_override) if real.rc_override.size else np.empty((0, 8))
        rc_out_norm = normalize_rc_out(real.rc_out) if real.rc_out.size else np.empty((0, 8))
        joy_rc_override_norm = (
            normalize_rc_override(real.joy_rc_override) if real.joy_rc_override.size else np.empty((0, 8))
        )
        steps = int(math.ceil(duration / dt))
        for _ in range(steps):
            t = float(self.data.time)
            if command_source == "joy_node" and joy_rc_override_norm.size:
                command8 = resample_matrix(real.joy_rc_override_t, joy_rc_override_norm, np.array([t], dtype=float))[0]
                command = np.array([command8[4], command8[5], -command8[3], -command8[2]], dtype=np.float64)
                self.last_command_mode = COMMAND_MODE_RC_OVERRIDE
                self.last_command8 = command8.copy()
                self.last_direct_command = command.copy()
                self.apply_direct_command_targets(command)
            elif command_source == "rc_out" and rc_out_norm.size:
                command8 = resample_matrix(real.rc_out_t, rc_out_norm, np.array([t], dtype=float))[0]
                self.last_command_mode = COMMAND_MODE_RC_OUT
                self.last_command8 = command8.copy()
                self.last_direct_command = np.zeros(4, dtype=np.float64)
                self.apply_rc_out_targets(command8)
            elif rc_override_norm.size:
                command8 = resample_matrix(real.rc_override_t, rc_override_norm, np.array([t], dtype=float))[0]
                # Same default mapping used by Ros2Bridge: ch5 forward, ch6 sway,
                # ch4 yaw inverted, ch3 heave inverted.
                command = np.array([command8[4], command8[5], -command8[3], -command8[2]], dtype=np.float64)
                self.last_command_mode = COMMAND_MODE_RC_OVERRIDE
                self.last_command8 = command8.copy()
                self.last_direct_command = command.copy()
                self.apply_direct_command_targets(command)
            else:
                self.last_command_mode = COMMAND_MODE_NONE
                self.last_command8 = np.zeros(8, dtype=np.float64)
                self.last_direct_command = np.zeros(4, dtype=np.float64)
                for name in self.all_thruster_names:
                    self.thr_target[name] = 0.0

            if self.thruster_update_due():
                thr_dt = dt if self.thruster_dt_mode == "current-code" else self.thruster_loop_dt
                self.update_thruster_forces(thr_dt)
            self.apply_underwater_wrench(dt)
            mujoco.mj_step(self.model, self.data)
            if float(self.data.time) + 1e-9 >= record_next_t:
                self.record_state(out)
                while record_next_t <= float(self.data.time) + 1e-9:
                    record_next_t += record_dt_s

        result = {key: np.asarray(value, dtype=float) for key, value in out.items()}
        result["runtime_body_mass"] = np.array([float(self.model.body_mass[self.base_id])], dtype=float)
        result["runtime_body_ipos"] = self.model.body_ipos[self.base_id].copy().astype(float)
        result["runtime_body_inertia"] = self.model.body_inertia[self.base_id].copy().astype(float)
        return result


def build_replay_configs(selected: str) -> list[dict[str, Any]]:
    all_configs = [
        {
            "name": "current_joy_node_rc_override",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "joy_node",
            "thruster_dt_mode": "current-code",
        },
        {
            "name": "current_rc_override",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "rc_override",
            "thruster_dt_mode": "current-code",
        },
        {
            "name": "legacy_rc_override",
            "scene": LEGACY_SCENE,
            "profile": "legacy",
            "fluid_model": "legacy",
            "command_source": "rc_override",
            "thruster_dt_mode": "current-code",
        },
        {
            "name": "current_rc_out",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "rc_out",
            "thruster_dt_mode": "current-code",
        },
        {
            "name": "current_inertia1_rc_override",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "rc_override",
            "thruster_dt_mode": "current-code",
            "profile_overrides": {"body_inertia_scale_xyz": [1.0, 1.0, 1.0]},
        },
        {
            "name": "current_thruster_gain1_rc_override",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "rc_override",
            "thruster_dt_mode": "current-code",
            "thruster_param_overrides": {"global": {"gain_scale_all": 1.0}},
        },
        {
            "name": "current_inertia1_gain1_rc_override",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "rc_override",
            "thruster_dt_mode": "current-code",
            "profile_overrides": {"body_inertia_scale_xyz": [1.0, 1.0, 1.0]},
            "thruster_param_overrides": {"global": {"gain_scale_all": 1.0}},
        },
        {
            "name": "current_contact_free_tuned_rc_override",
            "scene": CURRENT_SCENE,
            "profile": "current",
            "fluid_model": "current",
            "command_source": "rc_override",
            "thruster_dt_mode": "current-code",
            "profile_overrides": {
                "command_axis_gain": {"surge": 0.46, "sway": 0.26, "yaw": 0.63, "heave": 0.45},
                "yaw_torque_scale": 1.20,
            },
        },
    ]
    if selected == "all":
        return all_configs
    wanted = {item.strip() for item in selected.split(",") if item.strip()}
    return [cfg for cfg in all_configs if cfg["name"] in wanted]


def summarize_replay(real: RealSeries, sim: dict[str, np.ndarray], cfg: dict[str, Any]) -> dict[str, Any]:
    t = sim["t"]
    dvl_metrics = compare_vector(real.dvl_t, real.dvl_vel, t, sim["dvl_vel"], crop_start_s=5.0)
    gyro_metrics = compare_vector(real.imu_t, real.imu_gyro, t, sim["imu_gyro"], crop_start_s=5.0)
    contact_free_mask = sim["contact_count"] <= 0 if "contact_count" in sim else np.ones_like(t, dtype=bool)
    dvl_contact_free_metrics = compare_vector(
        real.dvl_t,
        real.dvl_vel,
        t,
        sim["dvl_vel"],
        crop_start_s=5.0,
        sim_valid_t=t,
        sim_valid=contact_free_mask,
    )
    gyro_contact_free_metrics = compare_vector(
        real.imu_t,
        real.imu_gyro,
        t,
        sim["imu_gyro"],
        crop_start_s=5.0,
        sim_valid_t=t,
        sim_valid=contact_free_mask,
    )
    accel_stats = {
        "real": vector_stats(real.imu_t, real.imu_accel, ("x", "y", "z")) if real.imu_accel.size else {},
        "sim": vector_stats(t, sim["imu_accel"], ("x", "y", "z")),
    }
    depth_metrics = compare_scalar_with_lag(real.depth_t, real.depth, t, sim["depth"], crop_start_s=5.0, remove_median_offset=False)
    depth_shape_metrics = compare_scalar_with_lag(real.depth_t, real.depth, t, sim["depth"], crop_start_s=5.0, remove_median_offset=True)
    depth_contact_free_metrics = compare_scalar_with_lag(
        real.depth_t,
        real.depth,
        t,
        sim["depth"],
        crop_start_s=5.0,
        remove_median_offset=False,
        sim_valid_t=t,
        sim_valid=contact_free_mask,
    )
    depth_contact_free_shape_metrics = compare_scalar_with_lag(
        real.depth_t,
        real.depth,
        t,
        sim["depth"],
        crop_start_s=5.0,
        remove_median_offset=True,
        sim_valid_t=t,
        sim_valid=contact_free_mask,
    )
    if real.depth_t.size >= 3:
        real_depth_rate = derivative(real.depth_t, real.depth)
    else:
        real_depth_rate = np.empty(0)
    sim_depth_rate = derivative(t, sim["depth"])
    depth_rate_metrics = compare_scalar_with_lag(real.depth_t, real_depth_rate, t, sim_depth_rate, crop_start_s=5.0)
    depth_rate_contact_free_metrics = compare_scalar_with_lag(
        real.depth_t,
        real_depth_rate,
        t,
        sim_depth_rate,
        crop_start_s=5.0,
        sim_valid_t=t,
        sim_valid=contact_free_mask,
    )
    odom_vel_metrics = compare_vector(real.odom_t, real.odom_vel, t, sim["vel_world"], crop_start_s=5.0)
    sensor_truth_consistency = {
        "dvl_published_body_vs_body_truth": compare_same_time_vector(t, sim["vel_body"], sim["dvl_vel"]),
        "dvl_raw_sensor_vs_body_truth": compare_same_time_vector(t, sim["vel_body"], sim["dvl_vel_raw_sensor"]),
        "imu_gyro_vs_body_truth": compare_same_time_vector(t, sim["ang_body"], sim["imu_gyro"]),
        "depth_sensor_vs_base_depth_truth": compare_same_time_scalar(t, sim["base_depth_truth"], sim["depth"]),
    }
    command_summary = {
        "mode_counts": {
            "none": int(np.sum(sim["command_mode"] == COMMAND_MODE_NONE)) if t.size else 0,
            "rc_override": int(np.sum(sim["command_mode"] == COMMAND_MODE_RC_OVERRIDE)) if t.size else 0,
            "rc_out": int(np.sum(sim["command_mode"] == COMMAND_MODE_RC_OUT)) if t.size else 0,
        },
        "command_direct4": {
            "forward": scalar_stats(t, sim["command_direct4"][:, 0]) if t.size else {"count": 0},
            "sway": scalar_stats(t, sim["command_direct4"][:, 1]) if t.size else {"count": 0},
            "yaw": scalar_stats(t, sim["command_direct4"][:, 2]) if t.size else {"count": 0},
            "heave": scalar_stats(t, sim["command_direct4"][:, 3]) if t.size else {"count": 0},
        },
        "command8_norm": {
            f"ch{i + 1}": scalar_stats(t, sim["command8_norm"][:, i]) if t.size else {"count": 0}
            for i in range(8)
        },
    }

    real_yaw_t = real.odom_t if real.odom_rpy.size else real.imu_t
    real_yaw = real.odom_rpy[:, 2] if real.odom_rpy.size else (real.imu_rpy[:, 2] if real.imu_rpy.size else np.empty(0))
    yaw_metrics = compare_scalar_with_lag(real_yaw_t, np.unwrap(real_yaw), t, np.unwrap(sim["base_yaw"]), crop_start_s=5.0, remove_median_offset=True)
    attitude_t = real.odom_t if real.odom_rpy.size else real.imu_t
    attitude_rpy = real.odom_rpy if real.odom_rpy.size else real.imu_rpy
    attitude_metrics: dict[str, Any] = {}
    if attitude_rpy.size and "base_rpy" in sim:
        attitude_metrics = {
            "roll": compare_scalar_with_lag(
                attitude_t,
                np.unwrap(attitude_rpy[:, 0]),
                t,
                np.unwrap(sim["base_rpy"][:, 0]),
                crop_start_s=5.0,
                remove_median_offset=True,
            ),
            "pitch": compare_scalar_with_lag(
                attitude_t,
                np.unwrap(attitude_rpy[:, 1]),
                t,
                np.unwrap(sim["base_rpy"][:, 1]),
                crop_start_s=5.0,
                remove_median_offset=True,
            ),
            "yaw": yaw_metrics,
        }

    pressure_static_metrics = compare_scalar_with_lag(
        real.static_pressure_t,
        real.static_pressure,
        t,
        sim["pressure"],
        crop_start_s=5.0,
        remove_median_offset=True,
    )
    pressure_atm_metrics = compare_scalar_with_lag(
        real.atm_pressure_t,
        real.atm_pressure,
        t,
        sim["pressure"],
        crop_start_s=5.0,
        remove_median_offset=True,
    )

    path_summary: dict[str, Any] = {}
    if real.odom_xyz.size and t.size:
        real_xyz0 = real.odom_xyz - real.odom_xyz[0]
        sim_xyz0 = sim["base_xyz"] - sim["base_xyz"][0]
        path_summary["real_final_xyz_delta"] = [float(v) for v in real_xyz0[-1]]
        path_summary["sim_final_xyz_delta"] = [float(v) for v in sim_xyz0[-1]]
        path_summary["real_path_length_m"] = float(np.sum(np.linalg.norm(np.diff(real_xyz0, axis=0), axis=1)))
        path_summary["sim_path_length_m"] = float(np.sum(np.linalg.norm(np.diff(sim_xyz0, axis=0), axis=1)))
        path_summary["sim_base_xyz_stats"] = vector_stats(t, sim["base_xyz"], ("x", "y", "z"))

    physics_summary = {
        "contact_fraction": float(np.mean(sim["contact_count"] > 0)) if t.size else 0.0,
        "contact_max": int(np.max(sim["contact_count"])) if t.size else 0,
        "submerged": scalar_stats(t, sim["submerged"]) if t.size else {"count": 0},
        "buoyancy_force_world": vector_stats(t, sim["buoy_force_world"], ("x", "y", "z")) if t.size else {},
        "buoyancy_torque_world": vector_stats(t, sim["buoy_tau_world"], ("x", "y", "z")) if t.size else {},
        "thruster_force_world": vector_stats(t, sim["thruster_force_world"], ("x", "y", "z")) if t.size else {},
        "thruster_torque_world": vector_stats(t, sim["thruster_tau_world"], ("x", "y", "z")) if t.size else {},
        "xfrc_applied_world": vector_stats(t, sim["xfrc_world"][:, :3], ("x", "y", "z")) if t.size else {},
        "xfrc_applied_torque_world": vector_stats(t, sim["xfrc_world"][:, 3:], ("x", "y", "z")) if t.size else {},
        "qfrc_passive_norm": scalar_stats(t, sim["qfrc_passive_norm"]) if t.size else {"count": 0},
        "qfrc_actuator_norm": scalar_stats(t, sim["qfrc_actuator_norm"]) if t.size else {"count": 0},
        "qfrc_bias_norm": scalar_stats(t, sim["qfrc_bias_norm"]) if t.size else {"count": 0},
        "qacc_norm": scalar_stats(t, sim["qacc_norm"]) if t.size else {"count": 0},
    }
    thruster_summary = {}
    for name in PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS:
        thruster_summary[name] = {
            "target": scalar_stats(t, sim[f"thr_{name}_target"]),
            "state": scalar_stats(t, sim[f"thr_{name}_state"]),
            "force_n": scalar_stats(t, sim[f"thr_{name}_force"]),
        }

    return {
        "config": {k: str(v) if isinstance(v, Path) else v for k, v in cfg.items()},
        "model_runtime": {
            "body_mass_kg": float(sim["runtime_body_mass"][0]) if "runtime_body_mass" in sim else math.nan,
            "body_ipos_m": sim["runtime_body_ipos"].reshape(-1)[:3].astype(float).tolist()
            if "runtime_body_ipos" in sim
            else [math.nan, math.nan, math.nan],
            "body_inertia_kg_m2": sim["runtime_body_inertia"].reshape(-1)[:3].astype(float).tolist()
            if "runtime_body_inertia" in sim
            else [math.nan, math.nan, math.nan],
        },
        "duration_s": float(t[-1] - t[0]) if t.size >= 2 else 0.0,
        "sample_count": int(t.size),
        "dvl_velocity_metrics": dvl_metrics,
        "imu_gyro_metrics": gyro_metrics,
        "contact_free_metrics": {
            "valid_fraction": float(np.mean(contact_free_mask)) if t.size else 0.0,
            "dvl_velocity_metrics": dvl_contact_free_metrics,
            "imu_gyro_metrics": gyro_contact_free_metrics,
            "depth_metrics": depth_contact_free_metrics,
            "depth_shape_metrics_offset_removed": depth_contact_free_shape_metrics,
            "depth_rate_metrics": depth_rate_contact_free_metrics,
        },
        "imu_accel_distribution": accel_stats,
        "depth_metrics": depth_metrics,
        "depth_shape_metrics_offset_removed": depth_shape_metrics,
        "depth_rate_metrics": depth_rate_metrics,
        "odometry_velocity_metrics": odom_vel_metrics,
        "sensor_truth_consistency": sensor_truth_consistency,
        "command_summary": command_summary,
        "yaw_angle_metrics_offset_removed": yaw_metrics,
        "attitude_metrics_offset_removed": attitude_metrics,
        "pressure_static_vs_sim_pressure_offset_removed": pressure_static_metrics,
        "pressure_atm_vs_sim_pressure_offset_removed": pressure_atm_metrics,
        "path_summary": path_summary,
        "physics_summary": physics_summary,
        "thruster_summary": thruster_summary,
    }


def metric_cell(metric: dict[str, Any]) -> str:
    if not metric or metric.get("count", 0) == 0:
        return "n/a"
    corr = metric.get("correlation")
    gain = metric.get("gain_fit_real_from_sim", {}).get("gain")
    return (
        f"rmse={metric.get('rmse', float('nan')):.3g}, "
        f"corr={corr:.3g}" if corr is not None else f"rmse={metric.get('rmse', float('nan')):.3g}, corr=n/a"
    ) + (f", lag={metric.get('lag_s', 0.0):+.2f}s" if "lag_s" in metric else "") + (
        f", gain={gain:.3g}" if gain is not None else ""
    )


def plot_overlay(real: RealSeries, sim: dict[str, np.ndarray], title: str, out_path: Path) -> None:
    t = sim["t"]
    fig, axes = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    rc_t = real.rc_override_t
    if real.rc_override.size:
        cmd = normalize_rc_override(real.rc_override)
        axes[0].plot(rc_t, cmd[:, 4], label="ch5 forward", lw=0.8)
        axes[0].plot(rc_t, cmd[:, 5], label="ch6 sway", lw=0.8)
        axes[0].plot(rc_t, -cmd[:, 3], label="-ch4 yaw", lw=0.8)
        axes[0].plot(rc_t, -cmd[:, 2], label="-ch3 heave", lw=0.8)
    axes[0].set_ylabel("cmd")
    axes[0].legend(loc="upper right", ncol=4, fontsize=8)
    if real.dvl_vel.size:
        axes[1].plot(real.dvl_t, real.dvl_vel[:, 0], "k", lw=0.8, label="real DVL x")
        axes[1].plot(real.dvl_t, real.dvl_vel[:, 1], color="0.5", lw=0.8, label="real DVL y")
    axes[1].plot(t, sim["dvl_vel"][:, 0], "tab:red", lw=0.8, label="sim DVL x")
    axes[1].plot(t, sim["dvl_vel"][:, 1], "tab:orange", lw=0.8, label="sim DVL y")
    axes[1].set_ylabel("DVL m/s")
    axes[1].legend(loc="upper right", ncol=4, fontsize=8)
    if real.imu_gyro.size:
        axes[2].plot(real.imu_t, real.imu_gyro[:, 2], "k", lw=0.8, label="real gyro z")
    axes[2].plot(t, sim["imu_gyro"][:, 2], "tab:red", lw=0.8, label="sim gyro z")
    axes[2].set_ylabel("yaw rate rad/s")
    axes[2].legend(loc="upper right", fontsize=8)
    if real.depth.size:
        axes[3].plot(real.depth_t, real.depth, "k", lw=0.8, label="real depth")
    axes[3].plot(t, sim["depth"], "tab:red", lw=0.8, label="sim depth")
    axes[3].set_ylabel("depth m")
    axes[3].legend(loc="upper right", fontsize=8)
    if real.odom_xyz.size:
        real_xyz = real.odom_xyz - real.odom_xyz[0]
        axes[4].plot(real.odom_t, real_xyz[:, 0], "k", lw=0.8, label="real odom x delta")
        axes[4].plot(real.odom_t, real_xyz[:, 1], color="0.5", lw=0.8, label="real odom y delta")
    sim_xyz = sim["base_xyz"] - sim["base_xyz"][0]
    axes[4].plot(t, sim_xyz[:, 0], "tab:red", lw=0.8, label="sim x delta")
    axes[4].plot(t, sim_xyz[:, 1], "tab:orange", lw=0.8, label="sim y delta")
    axes[4].set_ylabel("xy delta m")
    axes[4].set_xlabel("bag time s")
    axes[4].legend(loc="upper right", ncol=4, fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def write_report(out_dir: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# 2026-04-01 Real RC Replay vs MuJoCo",
        "",
        "This report replays real `/mavros/rc/override` or `/mavros/rc/out` commands through the local MuJoCo model without modifying ArduPilot.",
        "Metrics are computed after a 5 s warmup crop. Positive/negative best lag is selected by maximum absolute correlation.",
        "",
    ]
    for bag_name, bag_result in payload["bags"].items():
        lines.extend([f"## {bag_name}", ""])
        lines.append(
            f"Real duration: `{bag_result['real_duration_s']:.2f}s`, "
            f"RC override samples: `{bag_result['real_counts']['rc_override']}`, "
            f"RC out samples: `{bag_result['real_counts']['rc_out']}`, "
            f"DVL samples: `{bag_result['real_counts']['dvl']}`, "
            f"IMU samples: `{bag_result['real_counts']['imu']}`, "
            f"depth samples: `{bag_result['real_counts']['depth']}`."
        )
        lines.append("")
        lines.append("| replay config | DVL x | DVL y | DVL z | gyro z | depth | depth rate | contact frac |")
        lines.append("|---|---|---|---|---|---|---|---:|")
        for cfg_name, result in bag_result["replays"].items():
            dvl = result["dvl_velocity_metrics"]
            gyro = result["imu_gyro_metrics"]
            phys = result["physics_summary"]
            lines.append(
                "| "
                + " | ".join(
                    [
                        f"`{cfg_name}`",
                        metric_cell(dvl.get("x", {})),
                        metric_cell(dvl.get("y", {})),
                        metric_cell(dvl.get("z", {})),
                        metric_cell(gyro.get("z", {})),
                        metric_cell(result.get("depth_metrics", {})),
                        metric_cell(result.get("depth_rate_metrics", {})),
                        f"{phys.get('contact_fraction', 0.0):.3f}",
                    ]
                )
                + " |"
            )
        lines.append("")
        lines.append("### Main Per-Config Diagnostics")
        lines.append("")
        for cfg_name, result in bag_result["replays"].items():
            path = result.get("path_summary", {})
            phys = result.get("physics_summary", {})
            pressure_static = result.get("pressure_static_vs_sim_pressure_offset_removed", {})
            pressure_atm = result.get("pressure_atm_vs_sim_pressure_offset_removed", {})
            consistency = result.get("sensor_truth_consistency", {})
            dvl_consistency = consistency.get("dvl_published_body_vs_body_truth", {})
            raw_dvl_consistency = consistency.get("dvl_raw_sensor_vs_body_truth", {})
            depth_consistency = consistency.get("depth_sensor_vs_base_depth_truth", {})
            lines.append(f"#### `{cfg_name}`")
            lines.append("")
            lines.append(
                f"- path length real/sim: `{path.get('real_path_length_m', float('nan')):.3g}` / "
                f"`{path.get('sim_path_length_m', float('nan')):.3g}` m; "
                f"final delta real/sim: `{path.get('real_final_xyz_delta')}` / `{path.get('sim_final_xyz_delta')}`."
            )
            lines.append(
                f"- static pressure vs sim pressure: {metric_cell(pressure_static)}; "
                f"atm pressure vs sim pressure: {metric_cell(pressure_atm)}."
            )
            lines.append(
                f"- sim sensor-vs-truth check: DVL y corrected `{metric_cell(dvl_consistency.get('y', {}))}`, "
                f"DVL y raw `{metric_cell(raw_dvl_consistency.get('y', {}))}`, "
                f"Bar30 depth vs base depth `{metric_cell(depth_consistency)}`."
            )
            lines.append(
                f"- qfrc passive rms `{phys.get('qfrc_passive_norm', {}).get('rms', float('nan')):.3g}`, "
                f"actuator rms `{phys.get('qfrc_actuator_norm', {}).get('rms', float('nan')):.3g}`, "
                f"qacc rms `{phys.get('qacc_norm', {}).get('rms', float('nan')):.3g}`."
            )
            thr = result.get("thruster_summary", {})
            force_peaks = {
                name: cfg.get("force_n", {}).get("p95")
                for name, cfg in thr.items()
                if cfg.get("force_n", {}).get("count", 0)
            }
            lines.append(f"- thruster force p95 N by motor: `{force_peaks}`.")
            lines.append("")
    lines.extend(
        [
            "## Interpretation Checklist",
            "",
            "- `/mavros/rc/override` replay tests the simulator's current ROS2 bridge command path.",
            "- `/mavros/rc/out` replay is closer to motor-output replay, but the real topic is only about 2 Hz in these bags, so fast motor transients are lost.",
            "- DVL/gyro comparisons are the most physically meaningful in these bags. Heave/depth is confounded by ALT_HOLD and weak ch3 isolation.",
            "- MuJoCo built-in ellipsoid fluid force is not separately exposed as one named term; it is visible indirectly through passive/generalized dynamics and the resulting response.",
            "",
        ]
    )
    (out_dir / "replay_report.md").write_text("\n".join(lines))


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    bags = discover_bags(Path(args.root))
    configs = build_replay_configs(args.configs)
    payload: dict[str, Any] = {
        "root": str(args.root),
        "out": str(out_dir),
        "max_duration_s": args.max_duration,
        "record_dt_s": args.record_dt,
        "configs": [{k: str(v) if isinstance(v, Path) else v for k, v in cfg.items()} for cfg in configs],
        "bags": {},
    }
    for db_path in bags:
        real = extract_real_series(db_path)
        if real.rc_override.size == 0 and real.rc_out.size == 0 and real.joy_rc_override.size == 0:
            continue
        if args.bag and args.bag not in real.name:
            continue
        print(f"[replay] bag={real.name} duration={real.duration_s:.2f}s", flush=True)
        bag_out = out_dir / real.name
        bag_out.mkdir(parents=True, exist_ok=True)
        bag_result: dict[str, Any] = {
            "db_path": str(db_path),
            "real_duration_s": real.duration_s,
            "real_counts": {
                "rc_override": int(real.rc_override.shape[0]) if real.rc_override.size else 0,
                "rc_out": int(real.rc_out.shape[0]) if real.rc_out.size else 0,
                "joy": int(real.joy_axes.shape[0]) if real.joy_axes.size else 0,
                "joy_node_rc_override": int(real.joy_rc_override.shape[0]) if real.joy_rc_override.size else 0,
                "dvl": int(real.dvl_vel.shape[0]) if real.dvl_vel.size else 0,
                "imu": int(real.imu_gyro.shape[0]) if real.imu_gyro.size else 0,
                "depth": int(real.depth.shape[0]) if real.depth.size else 0,
                "odom": int(real.odom_xyz.shape[0]) if real.odom_xyz.size else 0,
            },
            "real_sensor_stats": {
                "dvl": vector_stats(real.dvl_t, real.dvl_vel, ("x", "y", "z")) if real.dvl_vel.size else {},
                "imu_gyro": vector_stats(real.imu_t, real.imu_gyro, ("x", "y", "z")) if real.imu_gyro.size else {},
                "imu_accel": vector_stats(real.imu_t, real.imu_accel, ("x", "y", "z")) if real.imu_accel.size else {},
                "depth": scalar_stats(real.depth_t, real.depth) if real.depth.size else {},
                "static_pressure": scalar_stats(real.static_pressure_t, real.static_pressure) if real.static_pressure.size else {},
                "atm_pressure": scalar_stats(real.atm_pressure_t, real.atm_pressure) if real.atm_pressure.size else {},
            },
            "replays": {},
        }
        for cfg in configs:
            if cfg["command_source"] == "joy_node" and real.joy_rc_override.size == 0:
                continue
            if cfg["command_source"] == "rc_out" and real.rc_out.size == 0:
                continue
            print(f"[replay]   config={cfg['name']}", flush=True)
            sim = OfflineUuvReplay(
                scene=Path(cfg["scene"]),
                profile_name=str(cfg["profile"]),
                fluid_model=str(cfg["fluid_model"]),
                thruster_dt_mode=str(cfg["thruster_dt_mode"]),
                rc_out_scale=float(args.rc_out_scale),
                profile_overrides=cfg.get("profile_overrides"),
                thruster_param_overrides=cfg.get("thruster_param_overrides"),
            ).run(real, command_source=str(cfg["command_source"]), max_duration_s=args.max_duration, record_dt_s=args.record_dt)
            result = summarize_replay(real, sim, cfg)
            bag_result["replays"][cfg["name"]] = result
            npz_path = bag_out / f"{cfg['name']}_timeseries.npz"
            np.savez_compressed(npz_path, **sim)
            plot_overlay(real, sim, f"{real.name} {cfg['name']}", bag_out / f"{cfg['name']}_overlay.png")
        payload["bags"][real.name] = bag_result
        (bag_out / "summary.json").write_text(json.dumps(bag_result, indent=2))
    (out_dir / "summary.json").write_text(json.dumps(payload, indent=2))
    write_report(out_dir, payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay April 1 real robot RC commands through MuJoCo and compare responses.")
    parser.add_argument("--root", default=str(DEFAULT_ROOT), help="April 1 extracted bag root or one .db3 path")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="Output directory")
    parser.add_argument("--bag", default="", help="Only process bags whose name contains this string")
    parser.add_argument(
        "--configs",
        default="current_rc_override,legacy_rc_override,current_rc_out",
        help="Comma-separated config names or 'all'",
    )
    parser.add_argument("--max-duration", type=float, default=None, help="Optional replay duration cap in seconds")
    parser.add_argument("--record-dt", type=float, default=0.02, help="Recorded simulation sample interval")
    parser.add_argument("--rc-out-scale", type=float, default=1.0, help="Scale for direct rc/out motor replay")
    args = parser.parse_args()
    analyze(args)


if __name__ == "__main__":
    main()
