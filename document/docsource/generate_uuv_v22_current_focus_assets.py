#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from matplotlib import patches


ROOT = Path(__file__).resolve().parents[2]
DOCSRC_DIR = Path(__file__).resolve().parent
FIG_DIR = DOCSRC_DIR / "figures_v22_latest"
SIM_DIR = ROOT / "uuv_mujoco" / "v2.2"
CONFIG_PATH = SIM_DIR / "config" / "sim_profiles.json"
THRUSTER_PARAMS_PATH = SIM_DIR / "config" / "thruster_params.json"
THRUSTER_PERF_PATH = SIM_DIR / "config" / "thruster_performance.json"
CURRENT_SCENE_PATH = SIM_DIR / "scenes" / "tank_current_scene.xml"
CURRENT_STEP_SUMMARY_PATH = DOCSRC_DIR / "measurement_summary_current_heavefix_latest_v30.json"
CURRENT_STEP_EVENTS_PATH = (
    DOCSRC_DIR / "measurements" / "mavros_step_test_current_heavefix_20260406_042120" / "sequence_events.json"
)
FOCUS_METRICS_PATH = DOCSRC_DIR / "uuv_v22_current_focus_metrics.json"

sys.path.insert(0, str(SIM_DIR))

from physics.hydrodynamics_helpers import first_order_response, scaled_polynomial_force, shape_thruster_command  # noqa: E402
from physics.sim_profile_helpers import build_hydrodynamics_config  # noqa: E402
from physics.thruster_mapping import PHYSICAL_VERTICAL_THRUSTERS, PHYSICAL_YAW_THRUSTERS  # noqa: E402


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Apple SD Gothic Neo", "Noto Sans CJK KR", "Arial", "DejaVu Sans"],
        "axes.titlesize": 15,
        "axes.labelsize": 13,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
        "legend.fontsize": 12,
        "figure.titlesize": 18,
        "axes.unicode_minus": False,
    }
)


@dataclass
class RuntimeContext:
    variant_key: str
    model: mujoco.MjModel
    data: mujoco.MjData
    base_id: int
    cob_site_id: int
    depth_site_id: int
    water_surface_site_id: int
    act: dict[str, int]
    horiz_order: tuple[str, ...]
    horiz_pinv: np.ndarray
    thruster_scale: dict[str, float]
    thruster_state: dict[str, float]
    thruster_target: dict[str, float]
    thruster_force_cmd: dict[str, float]
    thruster_extra_torque_world: np.ndarray
    profile: dict
    hydro_cfg: object
    thruster_global: dict
    vehicle_mass: float
    neutral_volume: float
    water_surface_z: float
    body_children: list[list[int]]


def ensure_dirs() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-12:
        return np.zeros_like(vec)
    return vec / norm


def load_perf_curve(active_voltage: float) -> tuple[np.ndarray, np.ndarray]:
    payload = load_json(THRUSTER_PERF_PATH)
    curves = payload.get("curves", [])
    best_curve = min(curves, key=lambda item: abs(float(item.get("voltage_v", active_voltage)) - active_voltage))
    pwm = np.asarray(best_curve["pwm_us"], dtype=np.float64)
    force = np.asarray(best_curve["force_n"], dtype=np.float64)
    order = np.argsort(pwm)
    return pwm[order], force[order]


def quat_to_rpy_deg(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = [float(v) for v in quat_wxyz]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def quat_wxyz_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
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


def body_subtree_mass(ctx: RuntimeContext, root_body_id: int) -> float:
    total = 0.0
    stack = [int(root_body_id)]
    while stack:
        bid = stack.pop()
        total += float(ctx.model.body_mass[bid])
        stack.extend(ctx.body_children[bid])
    return total


def build_horizontal_allocator(model: mujoco.MjModel, base_id: int, act: dict[str, int], horiz_order: tuple[str, ...]) -> np.ndarray:
    com_body = model.body_ipos[base_id].copy()
    alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
    for idx, name in enumerate(horiz_order):
        aid = act[name]
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
        fdir = normalize(model.actuator_gear[aid, :3].copy())
        r = model.site_pos[sid].copy() - com_body
        tau = np.cross(r, fdir)
        alloc[:, idx] = np.array([fdir[0], fdir[1], tau[2]], dtype=np.float64)
    row_scale = np.sum(np.abs(alloc), axis=1)
    row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
    return np.linalg.pinv(alloc / row_scale[:, None])


def mix_horizontal_thrusters(horiz_pinv: np.ndarray, fwd_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
    wrench_cmd = np.array([fwd_cmd, sway_cmd, yaw_cmd], dtype=np.float64)
    thr = horiz_pinv @ wrench_cmd
    max_abs = float(np.max(np.abs(thr)))
    if max_abs > 1.0:
        thr /= max_abs
    return np.clip(thr, -1.0, 1.0)


def body_velocity_local(model: mujoco.MjModel, data: mujoco.MjData, base_id: int) -> tuple[np.ndarray, np.ndarray]:
    vel6 = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, int(base_id), vel6, 1)
    return vel6[3:].copy(), vel6[:3].copy()


def parse_scene_sites() -> dict[str, np.ndarray]:
    root = ET.parse(CURRENT_SCENE_PATH).getroot()
    base = root.find(".//body[@name='base_link']")
    if base is None:
        raise RuntimeError("base_link not found in current scene")
    return {
        site.attrib["name"]: np.array([float(v) for v in site.attrib.get("pos", "0 0 0").split()], dtype=float)
        for site in base.findall("site")
        if site.attrib.get("name")
    }


def composite_summary(profile: dict) -> dict:
    components = profile["body_components"]
    total_mass = float(sum(component["mass"] for component in components))
    com = np.zeros(3, dtype=np.float64)
    for comp in components:
        com += float(comp["mass"]) * np.asarray(comp["mass_pos"], dtype=np.float64)
    com /= max(total_mass, 1e-9)

    inertia = np.zeros(3, dtype=np.float64)
    for comp in components:
        a, b, c = [float(v) for v in comp["size"]]
        mass = float(comp["mass"])
        self_inertia = np.array(
            [
                mass * (b * b + c * c) / 5.0,
                mass * (a * a + c * c) / 5.0,
                mass * (a * a + b * b) / 5.0,
            ],
            dtype=np.float64,
        )
        offset = np.asarray(comp["mass_pos"], dtype=np.float64) - com
        inertia += self_inertia + mass * np.array(
            [
                offset[1] ** 2 + offset[2] ** 2,
                offset[0] ** 2 + offset[2] ** 2,
                offset[0] ** 2 + offset[1] ** 2,
            ],
            dtype=np.float64,
        )

    scale = np.asarray(profile.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]), dtype=np.float64)
    buoy_points = profile.get("buoyancy_points", [])
    buoy_center = np.zeros(3, dtype=np.float64)
    if buoy_points:
        total_share = sum(float(point["share"]) for point in buoy_points)
        for point in buoy_points:
            buoy_center += float(point["share"]) * np.asarray(point["pos"], dtype=np.float64)
        buoy_center /= max(total_share, 1e-9)
    buoy_center[0] += float(profile.get("cob_x_offset", 0.0))
    buoy_center[2] += float(profile.get("cob_z_offset", 0.0))
    return {
        "mass_total": total_mass,
        "com": com,
        "inertia_raw": inertia,
        "inertia_scaled": inertia * scale,
        "buoy_center": buoy_center,
    }


def apply_body_component_distribution(model: mujoco.MjModel, data: mujoco.MjData, base_id: int, profile: dict) -> None:
    summary = composite_summary(profile)
    model.body_mass[base_id] = float(summary["mass_total"])
    model.body_ipos[base_id, :] = summary["com"]
    model.body_inertia[base_id, :] = summary["inertia_scaled"]
    if hasattr(mujoco, "mj_setConst"):
        mujoco.mj_setConst(model, data)
    mujoco.mj_forward(model, data)


def prepare_runtime(variant_key: str, profile: dict, thruster_global: dict) -> RuntimeContext:
    model = mujoco.MjModel.from_xml_path(str(CURRENT_SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    cob_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cob_site")
    depth_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site")
    water_surface_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "water_surface_ref")
    if base_id < 0 or cob_site_id < 0 or depth_site_id < 0 or water_surface_site_id < 0:
        raise RuntimeError("base_link/cob_site/bar30_site/water_surface_ref not found in scene")

    apply_body_component_distribution(model, data, base_id, profile)

    cob_x_offset = float(profile.get("cob_x_offset", 0.0))
    cob_z_offset = float(profile.get("cob_z_offset", 0.0))
    model.site_pos[cob_site_id][0] = float(model.body_ipos[base_id][0] + cob_x_offset)
    model.site_pos[cob_site_id][2] = float(model.body_ipos[base_id][2] + cob_z_offset)
    mujoco.mj_forward(model, data)

    act = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx): idx
        for idx in range(model.nu)
    }
    horiz_order = tuple(PHYSICAL_YAW_THRUSTERS)
    horiz_pinv = build_horizontal_allocator(model, base_id, act, horiz_order)

    body_children = [[] for _ in range(model.nbody)]
    for body_idx in range(1, model.nbody):
        parent_idx = int(model.body_parentid[body_idx])
        if 0 <= parent_idx < model.nbody:
            body_children[parent_idx].append(body_idx)

    hydro_cfg = build_hydrodynamics_config(profile)
    thruster_scale = {}
    gain_scale_all = float(np.clip(thruster_global.get("gain_scale_all", 1.0), 0.1, 20.0))
    for name in list(PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS):
        thruster_scale[name] = gain_scale_all

    vehicle_mass = float(sum(model.body_mass[1:]))
    if body_children:
        ctx_stub = RuntimeContext(
            variant_key=variant_key,
            model=model,
            data=data,
            base_id=base_id,
            cob_site_id=cob_site_id,
            depth_site_id=depth_site_id,
            water_surface_site_id=water_surface_site_id,
            act=act,
            horiz_order=horiz_order,
            horiz_pinv=horiz_pinv,
            thruster_scale=thruster_scale,
            thruster_state={},
            thruster_target={},
            thruster_force_cmd={},
            thruster_extra_torque_world=np.zeros(3, dtype=np.float64),
            profile=profile,
            hydro_cfg=hydro_cfg,
            thruster_global=thruster_global,
            vehicle_mass=vehicle_mass,
            neutral_volume=0.0,
            water_surface_z=0.0,
            body_children=body_children,
        )
        vehicle_mass = body_subtree_mass(ctx_stub, base_id)

    water_surface_z = float(data.site_xpos[water_surface_site_id][2])
    neutral_volume = float(vehicle_mass / max(float(model.opt.density), 1e-9))

    names = list(PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS)
    return RuntimeContext(
        variant_key=variant_key,
        model=model,
        data=data,
        base_id=base_id,
        cob_site_id=cob_site_id,
        depth_site_id=depth_site_id,
        water_surface_site_id=water_surface_site_id,
        act=act,
        horiz_order=horiz_order,
        horiz_pinv=horiz_pinv,
        thruster_scale=thruster_scale,
        thruster_state={name: 0.0 for name in names},
        thruster_target={name: 0.0 for name in names},
        thruster_force_cmd={name: 0.0 for name in names},
        thruster_extra_torque_world=np.zeros(3, dtype=np.float64),
        profile=profile,
        hydro_cfg=hydro_cfg,
        thruster_global=thruster_global,
        vehicle_mass=vehicle_mass,
        neutral_volume=neutral_volume,
        water_surface_z=water_surface_z,
        body_children=body_children,
    )


def force_from_shaped_command(ctx: RuntimeContext, command_shaped: float, gain: float) -> float:
    if abs(command_shaped) <= 1e-9:
        return 0.0
    magnitude = abs(command_shaped)
    force_max = float(ctx.hydro_cfg.thruster_force_max)
    if command_shaped >= 0.0:
        return float(
            scaled_polynomial_force(magnitude, ctx.thruster_global["forward_poly"], force_max) * gain
        )
    reverse_force_max = force_max * float(np.clip(ctx.thruster_global["reverse_asymmetry"], 0.1, 1.5))
    return float(
        -scaled_polynomial_force(magnitude, ctx.thruster_global["reverse_poly"], reverse_force_max) * gain
    )


def update_thruster_forces(ctx: RuntimeContext, dt: float) -> None:
    ctx.thruster_extra_torque_world[:] = 0.0
    base_rot = ctx.data.xmat[ctx.base_id].reshape(3, 3)
    com_body = ctx.model.body_ipos[ctx.base_id].copy()
    deadzone = float(np.clip(ctx.thruster_global["deadzone"], 0.0, 0.95))
    tau_up = float(max(ctx.thruster_global["tau_up"], 1e-4))
    tau_down = float(max(ctx.thruster_global["tau_down"], 1e-4))
    command_limit = float(np.clip(ctx.thruster_global["command_limit"], deadzone + 1e-3, 1.0))
    yaw_torque_scale = float(max(ctx.hydro_cfg.yaw_torque_scale, 0.0))
    vertical_gain = float(ctx.hydro_cfg.vertical_thruster_gain_scale)

    for name in ctx.thruster_state:
        aid = ctx.act[name]
        lo, hi = ctx.model.actuator_ctrlrange[aid]
        gain = float(ctx.thruster_scale.get(name, 1.0))
        if name in PHYSICAL_VERTICAL_THRUSTERS:
            gain *= vertical_gain
        target = float(np.clip(ctx.thruster_target[name], -1.0, 1.0))
        ctx.thruster_state[name] = first_order_response(ctx.thruster_state[name], target, dt, tau_up, tau_down)
        shaped = shape_thruster_command(ctx.thruster_state[name], deadzone, command_limit)
        force = float(np.clip(force_from_shaped_command(ctx, shaped, gain), lo, hi))
        ctx.data.ctrl[aid] = force
        ctx.thruster_force_cmd[name] = force

        if yaw_torque_scale > 1.0 and name in PHYSICAL_YAW_THRUSTERS:
            sid = mujoco.mj_name2id(ctx.model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            fdir = normalize(ctx.model.actuator_gear[aid, :3].copy())
            r_body = ctx.model.site_pos[sid].copy() - com_body
            tau_body = np.cross(r_body, fdir * force)
            extra_tau_body = np.array([0.0, 0.0, tau_body[2] * (yaw_torque_scale - 1.0)], dtype=np.float64)
            ctx.thruster_extra_torque_world += base_rot @ extra_tau_body


def apply_underwater_wrench(ctx: RuntimeContext, variant_key: str) -> None:
    data = ctx.data
    model = ctx.model
    base_id = ctx.base_id
    data.xfrc_applied[base_id, :] = 0.0

    com = data.xipos[base_id].copy()
    base_rot = data.xmat[base_id].reshape(3, 3)
    base_origin = data.xpos[base_id].copy()
    cob = data.site_xpos[ctx.cob_site_id].copy()
    depth = ctx.water_surface_z - float(base_origin[2])
    rho = float(model.opt.density)
    g = abs(float(model.opt.gravity[2]))

    buoy_model = str(ctx.hydro_cfg.buoyancy_model)
    half_height = float(ctx.hydro_cfg.half_height)
    cob_torque_scale = float(ctx.hydro_cfg.cob_torque_scale)
    buoyancy_scale = float(ctx.hydro_cfg.buoyancy_scale)
    buoyancy_slope_scale = float(ctx.hydro_cfg.buoyancy_slope_scale)
    buoyancy_point_blend = float(ctx.hydro_cfg.buoyancy_point_blend)
    point_depth_scale = buoyancy_slope_scale

    def immersed_fraction(depth_value: float, local_half_height: float) -> float:
        if str(buoy_model).lower() == "linear":
            frac = (float(depth_value) + local_half_height) / max(2.0 * local_half_height, 1e-9)
            return float(np.clip(frac, 0.0, 1.0))
        normalized_depth = float(np.clip(float(depth_value) / max(local_half_height, 1e-9), -1.0, 1.0))
        frac = 0.5 + 0.75 * normalized_depth - 0.25 * (normalized_depth ** 3)
        return float(np.clip(frac, 0.0, 1.0))

    buoy_tau_world = np.zeros(3, dtype=np.float64)
    buoy_force_world = np.zeros(3, dtype=np.float64)
    buoy_point = cob.copy()

    if variant_key == "four_point":
        points = list(ctx.hydro_cfg.buoyancy_points)
        total_share = float(sum(point.share for point in points))
        if total_share <= 1e-9:
            total_share = float(len(points))
        weighted_point = np.zeros(3, dtype=np.float64)
        for point in points:
            share = point.share / max(total_share, 1e-9)
            point_local = point.pos.copy()
            point_local[0] += float(ctx.profile.get("cob_x_offset", 0.0))
            point_local[2] += float(ctx.profile.get("cob_z_offset", 0.0))
            point_world = base_origin + base_rot @ point_local
            point_depth = ctx.water_surface_z - float(point_world[2])
            point_submerged = immersed_fraction(point_depth * point_depth_scale, point.half_height)
            point_buoyancy = rho * g * ctx.neutral_volume * share * point_submerged * buoyancy_scale
            point_force = np.array([0.0, 0.0, point_buoyancy], dtype=np.float64)
            buoy_force_world += point_force
            weighted_point += point_buoyancy * point_world
            buoy_tau_world += np.cross(point_world - com, point_force) * cob_torque_scale
        total_buoyancy = float(np.linalg.norm(buoy_force_world))
        if total_buoyancy > 1e-9:
            buoy_point = weighted_point / total_buoyancy
    else:
        buoyancy_submerged = immersed_fraction(depth * point_depth_scale, half_height)
        buoyancy_blend = buoyancy_point_blend * buoyancy_submerged
        buoy_point = ((1.0 - buoyancy_blend) * com) + (buoyancy_blend * cob)
        buoy = rho * g * ctx.neutral_volume * buoyancy_submerged * buoyancy_scale
        buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)
        buoy_tau_world = np.cross(buoy_point - com, buoy_force_world) * cob_torque_scale

    data.xfrc_applied[base_id, 0:3] += buoy_force_world
    data.xfrc_applied[base_id, 3:6] += buoy_tau_world
    data.xfrc_applied[base_id, 3:6] += ctx.thruster_extra_torque_world


def reset_thrusters(ctx: RuntimeContext) -> None:
    for name in ctx.thruster_target:
        ctx.thruster_target[name] = 0.0
        ctx.thruster_state[name] = 0.0
        ctx.thruster_force_cmd[name] = 0.0
    ctx.data.ctrl[:] = 0.0
    ctx.data.xfrc_applied[ctx.base_id, :] = 0.0
    ctx.thruster_extra_torque_world[:] = 0.0


def simulate_phase_step(variant_key: str, profile: dict, thruster_global: dict, forward_cmd: float, heave_cmd: float) -> dict:
    ctx = prepare_runtime(variant_key, profile, thruster_global)
    dt = float(ctx.model.opt.timestep)
    settle_s = 6.0
    active_s = 4.0
    relax_s = 2.5
    total_s = settle_s + active_s + relax_s
    steps = int(math.ceil(total_s / dt))

    times = np.zeros(steps, dtype=np.float64)
    surge = np.zeros(steps, dtype=np.float64)
    pitch_deg = np.zeros(steps, dtype=np.float64)
    roll_deg = np.zeros(steps, dtype=np.float64)
    depth_m = np.zeros(steps, dtype=np.float64)

    active_start = settle_s
    active_end = settle_s + active_s
    for idx in range(steps):
        sim_t = float(ctx.data.time)
        for name in ctx.thruster_target:
            ctx.thruster_target[name] = 0.0
        if active_start <= sim_t < active_end:
            if abs(forward_cmd) > 1e-9:
                horiz_cmd = mix_horizontal_thrusters(ctx.horiz_pinv, forward_cmd, 0.0, 0.0)
                for thr_idx, name in enumerate(ctx.horiz_order):
                    ctx.thruster_target[name] = float(horiz_cmd[thr_idx])
            if abs(heave_cmd) > 1e-9:
                for name in PHYSICAL_VERTICAL_THRUSTERS:
                    ctx.thruster_target[name] = float(np.clip(heave_cmd, -1.0, 1.0))

        update_thruster_forces(ctx, dt)
        apply_underwater_wrench(ctx, variant_key)
        mujoco.mj_step(ctx.model, ctx.data)

        lin_vel_body, _ = body_velocity_local(ctx.model, ctx.data, ctx.base_id)
        roll, pitch, _ = quat_to_rpy_deg(ctx.data.xquat[ctx.base_id].copy())
        times[idx] = float(ctx.data.time)
        surge[idx] = float(lin_vel_body[0])
        pitch_deg[idx] = float(pitch)
        roll_deg[idx] = float(roll)
        depth_m[idx] = float(ctx.water_surface_z - ctx.data.site_xpos[ctx.depth_site_id][2])

    active_mask = (times >= active_start) & (times <= active_end + 1e-9)
    depth_start = float(depth_m[np.searchsorted(times, active_start, side="left")])
    depth_end = float(depth_m[np.searchsorted(times, active_end, side="right") - 1])
    return {
        "time": times.tolist(),
        "surge": surge.tolist(),
        "pitch_deg": pitch_deg.tolist(),
        "roll_deg": roll_deg.tolist(),
        "depth_m": depth_m.tolist(),
        "active_start_s": active_start,
        "active_end_s": active_end,
        "metrics": {
            "peak_surge_mps": float(np.max(np.abs(surge[active_mask]))) if np.any(active_mask) else 0.0,
            "peak_pitch_deg": float(np.max(np.abs(pitch_deg[active_mask]))) if np.any(active_mask) else 0.0,
            "peak_roll_deg": float(np.max(np.abs(roll_deg[active_mask]))) if np.any(active_mask) else 0.0,
            "depth_delta_mm": float(abs(depth_end - depth_start) * 1000.0),
        },
    }


def simulate_attitude_release(variant_key: str, profile: dict, thruster_global: dict) -> dict:
    ctx = prepare_runtime(variant_key, profile, thruster_global)
    dt = float(ctx.model.opt.timestep)
    settle_s = 6.0
    settle_steps = int(math.ceil(settle_s / dt))
    for _ in range(settle_steps):
        reset_thrusters(ctx)
        update_thruster_forces(ctx, dt)
        apply_underwater_wrench(ctx, variant_key)
        mujoco.mj_step(ctx.model, ctx.data)

    qpos = ctx.data.qpos.copy()
    qvel = np.zeros_like(ctx.data.qvel)
    disturbed_quat = quat_wxyz_from_rpy_deg(6.0, 10.0, 0.0)
    qpos[3:7] = disturbed_quat
    ctx.data.qpos[:] = qpos
    ctx.data.qvel[:] = qvel
    mujoco.mj_forward(ctx.model, ctx.data)

    release_s = 6.0
    steps = int(math.ceil(release_s / dt))
    times = np.zeros(steps, dtype=np.float64)
    angle_norm_deg = np.zeros(steps, dtype=np.float64)
    roll_deg = np.zeros(steps, dtype=np.float64)
    pitch_deg = np.zeros(steps, dtype=np.float64)
    for idx in range(steps):
        reset_thrusters(ctx)
        update_thruster_forces(ctx, dt)
        apply_underwater_wrench(ctx, variant_key)
        mujoco.mj_step(ctx.model, ctx.data)
        roll, pitch, _ = quat_to_rpy_deg(ctx.data.xquat[ctx.base_id].copy())
        angle_norm_deg[idx] = float(math.hypot(roll, pitch))
        roll_deg[idx] = float(roll)
        pitch_deg[idx] = float(pitch)
        times[idx] = float(idx * dt)

    threshold = 0.2 * float(angle_norm_deg[0])
    settling_time = float(times[-1])
    for idx, value in enumerate(angle_norm_deg):
        if value <= threshold:
            settling_time = float(times[idx])
            break
    return {
        "time": times.tolist(),
        "angle_norm_deg": angle_norm_deg.tolist(),
        "roll_deg": roll_deg.tolist(),
        "pitch_deg": pitch_deg.tolist(),
        "metrics": {
            "initial_angle_norm_deg": float(angle_norm_deg[0]),
            "rms_angle_norm_deg": float(np.sqrt(np.mean(angle_norm_deg ** 2))),
            "settling_time_s": settling_time,
        },
    }


def metric_gap_percent(sim_value: float, ref_value: float, floor: float) -> float:
    scale = max(abs(float(ref_value)), float(floor))
    return abs(float(sim_value) - float(ref_value)) / scale * 100.0


def build_actual_reference(current_step: dict) -> dict:
    step = current_step["summary"]
    return {
        "forward_peak_surge_mps": float(step["stabilize_forward_step"]["surge_mps"]["max"]),
        "forward_peak_pitch_deg": float(math.degrees(step["stabilize_forward_step"]["pitch_rad"]["max"])),
        "forward_peak_roll_deg": float(math.degrees(step["stabilize_forward_step"]["roll_rad"]["max"])),
        "heave_depth_delta_mm": float(abs(step["manual_heave_step"]["depth_m"]["delta"]) * 1000.0),
    }


def build_variant_metrics(variant_key: str, forward_case: dict, heave_case: dict, release_case: dict, actual_ref: dict) -> dict:
    forward_metrics = forward_case["metrics"]
    heave_metrics = heave_case["metrics"]
    release_metrics = release_case["metrics"]
    gaps = {
        "surge_gap_pct": metric_gap_percent(
            forward_metrics["peak_surge_mps"],
            actual_ref["forward_peak_surge_mps"],
            0.10,
        ),
        "pitch_gap_pct": metric_gap_percent(
            forward_metrics["peak_pitch_deg"],
            actual_ref["forward_peak_pitch_deg"],
            0.10,
        ),
        "roll_gap_pct": metric_gap_percent(
            forward_metrics["peak_roll_deg"],
            actual_ref["forward_peak_roll_deg"],
            0.10,
        ),
        "heave_gap_pct": metric_gap_percent(
            heave_metrics["depth_delta_mm"],
            actual_ref["heave_depth_delta_mm"],
            5.0,
        ),
    }
    gap_index = float(sum(gaps.values()) / len(gaps))
    return {
        "variant": variant_key,
        "forward_step": forward_metrics,
        "heave_step": heave_metrics,
        "attitude_release": release_metrics,
        "actual_gap_percent": gaps,
        "actual_gap_index_pct": gap_index,
    }


def add_box(ax, x: float, y: float, w: float, h: float, title: str, body: str, fc: str, ec: str = "#334155") -> None:
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.018,rounding_size=0.03",
        linewidth=1.25,
        facecolor=fc,
        edgecolor=ec,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2.0, y + h - 0.030, title, ha="center", va="top", fontsize=14.5, fontweight="bold", color="#0f172a")
    ax.plot([x + 0.02, x + w - 0.02], [y + h - 0.060, y + h - 0.060], color="#d1d5db", linewidth=0.8)
    ax.text(
        x + w / 2.0,
        y + h - 0.080,
        body,
        ha="center",
        va="top",
        fontsize=11.6,
        color="#334155",
        multialignment="center",
        family="DejaVu Sans",
        linespacing=1.25,
    )


def add_flow_box(ax, x: float, y: float, w: float, h: float, title: str, body: str, fc: str, ec: str = "#334155") -> None:
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.018,rounding_size=0.03",
        linewidth=1.35,
        facecolor=fc,
        edgecolor=ec,
    )
    ax.add_patch(patch)
    ax.text(
        x + w / 2.0,
        y + h - 0.055,
        title,
        ha="center",
        va="top",
        fontsize=16.2,
        fontweight="bold",
        color="#0f172a",
    )
    divider_y = y + h - 0.105
    ax.plot([x + 0.02, x + w - 0.02], [divider_y, divider_y], color="#d1d5db", linewidth=0.9)
    ax.text(
        x + w / 2.0,
        y + h * 0.52,
        body,
        ha="center",
        va="center",
        fontsize=13.8,
        fontweight="bold",
        color="#334155",
        multialignment="center",
        family="DejaVu Sans",
        linespacing=1.24,
    )


def add_data_box(ax, x: float, y: float, w: float, h: float, title: str) -> None:
    points = np.array(
        [
            [x + 0.05 * w, y],
            [x + 0.95 * w, y],
            [x + w, y + 0.5 * h],
            [x + 0.95 * w, y + h],
            [x + 0.05 * w, y + h],
            [x, y + 0.5 * h],
        ]
    )
    patch = patches.Polygon(points, closed=True, linewidth=1.35, edgecolor="#334155", facecolor="#fff7ed")
    ax.add_patch(patch)
    ax.text(x + w / 2.0, y + h / 2.0, title, ha="center", va="center", fontsize=15.5, color="#b91c1c", family="DejaVu Sans Mono", fontweight="bold")


def add_arrow(ax, start: tuple[float, float], end: tuple[float, float]) -> None:
    arrow = patches.FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=18, linewidth=1.7, color="#334155")
    ax.add_patch(arrow)


def plot_current_engine_block_diagram() -> None:
    fig, ax = plt.subplots(figsize=(14.4, 16.0), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    add_box(ax, 0.27, 0.87, 0.46, 0.09, "[입력 명령]", "QGC / ROS2 / SITL servo", "#ffffff")
    add_box(
        ax,
        0.24,
        0.62,
        0.52,
        0.18,
        "[Python 커스텀: Thruster Model]",
        "- deadzone\n- tau_up / tau_down\n- forward / reverse force curve\n- gain_scale_all\n- reverse_asymmetry\n- vertical_thruster_gain_scale\n- horizontal thruster mixing\n- yaw_torque_scale",
        "#ffffff",
    )
    add_data_box(ax, 0.26, 0.49, 0.48, 0.07, "[data.ctrl + extra yaw torque]")
    add_box(
        ax,
        0.10,
        0.25,
        0.33,
        0.18,
        "[Python 커스텀: Hydrostatics]",
        "- buoyancy\n- 4-point buoyancy\n- restoring torque\n- runtime mass / CoM / inertia",
        "#ffffff",
    )
    add_box(
        ax,
        0.57,
        0.25,
        0.33,
        0.18,
        "[MuJoCo Built-in: Ellipsoid Fluid]",
        "- ellipsoid fluid drag\n- viscous fluid force / torque\n- rigid-body / contact integration",
        "#ffffff",
    )
    add_box(ax, 0.31, 0.10, 0.38, 0.08, "[MuJoCo mj_step()]", "", "#ffffff")
    add_box(ax, 0.31, 0.00, 0.38, 0.07, "[pose / vel / sensors]", "", "#eef2ff")

    add_arrow(ax, (0.50, 0.87), (0.50, 0.80))
    add_arrow(ax, (0.50, 0.62), (0.50, 0.56))
    ax.plot([0.20, 0.80], [0.49, 0.49], color="#334155", linewidth=1.0)
    add_arrow(ax, (0.20, 0.49), (0.20, 0.43))
    add_arrow(ax, (0.80, 0.49), (0.80, 0.43))
    ax.plot([0.26, 0.74], [0.22, 0.22], color="#334155", linewidth=1.0)
    add_arrow(ax, (0.26, 0.22), (0.50, 0.18))
    add_arrow(ax, (0.74, 0.22), (0.50, 0.18))
    add_arrow(ax, (0.50, 0.10), (0.50, 0.07))

    fig.savefig(FIG_DIR / "uuv_v22_latest_current_engine_block_diagram.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_thruster_model_figure(profile: dict, thruster_global: dict) -> None:
    fig = plt.figure(figsize=(17.0, 11.4), dpi=220, constrained_layout=True)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.08, 1.02])
    ax0 = fig.add_subplot(gs[0, :])
    ax1 = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[1, 1])

    ax0.set_xlim(0, 1)
    ax0.set_ylim(0, 1)
    ax0.axis("off")

    x_positions = [0.02, 0.22, 0.42, 0.62, 0.82]
    widths = [0.16, 0.16, 0.16, 0.16, 0.16]
    titles = [
        "1. Raw command",
        "2. Dynamic lag",
        "3. Shape / clamp",
        "4. Force map",
        "5. Runtime output",
    ]
    bodies = [
        "normalized\nthrust request\n[-1, 1]",
        f"first-order lag\nup {thruster_global['tau_up']:.2f} s\ndown {thruster_global['tau_down']:.2f} s",
        f"deadzone {thruster_global['deadzone']:.3f}\ncommand limit {thruster_global['command_limit']:.1f}",
        f"max thrust {profile['thruster_force_max']:.1f} N\nforward / reverse\npolynomial map",
        f"data.ctrl\nvertical x{profile['vertical_thruster_gain_scale']:.2f}\nyaw torque x{profile['yaw_torque_scale']:.2f}",
    ]
    for x0, width, title, body in zip(x_positions, widths, titles, bodies):
        add_flow_box(ax0, x0, 0.22, width, 0.54, title, body, "#ffffff")
    for idx in range(len(x_positions) - 1):
        add_arrow(ax0, (x_positions[idx] + widths[idx] + 0.006, 0.49), (x_positions[idx + 1] - 0.006, 0.49))
    ax0.text(0.5, 0.90, "Current thrust handling path", ha="center", va="center", fontsize=17, fontweight="bold")
    ax0.text(
        0.5,
        0.08,
        "Current default path uses the simple polynomial model.\nPerformance-table interpolation is not the default SITL path.",
        ha="center",
        va="center",
        fontsize=12.4,
        color="#475569",
        linespacing=1.3,
    )

    command = np.linspace(-1.0, 1.0, 801)
    shaped = np.array(
        [shape_thruster_command(v, thruster_global["deadzone"], thruster_global["command_limit"]) for v in command],
        dtype=np.float64,
    )
    force = np.zeros_like(command)
    for idx, shaped_cmd in enumerate(shaped):
        if shaped_cmd >= 0.0:
            force[idx] = scaled_polynomial_force(
                abs(shaped_cmd),
                thruster_global["forward_poly"],
                float(profile["thruster_force_max"]),
            )
        else:
            force[idx] = -scaled_polynomial_force(
                abs(shaped_cmd),
                thruster_global["reverse_poly"],
                float(profile["thruster_force_max"]) * float(thruster_global["reverse_asymmetry"]),
            )

    ax1.plot(command, shaped, color="#0f766e", linewidth=2.2)
    ax1.axvline(-thruster_global["deadzone"], color="#94a3b8", linewidth=1.0, linestyle="--")
    ax1.axvline(thruster_global["deadzone"], color="#94a3b8", linewidth=1.0, linestyle="--")
    ax1.set_title("Command shaping")
    ax1.set_xlabel("Raw normalized command")
    ax1.set_ylabel("Shaped command")
    ax1.grid(True, alpha=0.25)

    ax2.plot(command, force, color="#2563eb", linewidth=2.2, label="Base horizontal thruster force")
    lag_time = np.linspace(0.0, 0.25, 250)
    lag_up = np.array([first_order_response(0.0, 1.0, t, thruster_global["tau_up"], thruster_global["tau_down"]) for t in lag_time])
    lag_down = np.array([first_order_response(1.0, 0.0, t, thruster_global["tau_up"], thruster_global["tau_down"]) for t in lag_time])
    ax2b = ax2.twinx()
    ax2b.plot(lag_time * 8.0 - 1.0, lag_up, color="#dc2626", linewidth=1.8, linestyle="--", label="Step-up lag")
    ax2b.plot(lag_time * 8.0, lag_down, color="#ea580c", linewidth=1.8, linestyle=":", label="Step-down lag")
    ax2.set_title("Force curve and lag behavior")
    ax2.set_xlabel("Normalized command / scaled time")
    ax2.set_ylabel("Force [N]")
    ax2b.set_ylabel("Lag state")
    ax2.grid(True, alpha=0.25)
    lines_1, labels_1 = ax2.get_legend_handles_labels()
    lines_2, labels_2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper left", frameon=False)

    fig.savefig(FIG_DIR / "uuv_v22_latest_thruster_model_detail.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_thruster_performance_curve(profile: dict, thruster_global: dict) -> None:
    active_voltage = float(profile["thruster_voltage"])
    pwm, perf_force = load_perf_curve(active_voltage)
    norm_cmd = np.clip((pwm - 1500.0) / 400.0, -1.0, 1.0)
    center_mask = (pwm >= 1460.0) & (pwm <= 1540.0)
    plateau_mask = np.isclose(perf_force, 0.0)
    plateau_pwm = pwm[plateau_mask]
    plateau_min = float(np.min(plateau_pwm)) if plateau_pwm.size else 1500.0
    plateau_max = float(np.max(plateau_pwm)) if plateau_pwm.size else 1500.0

    fig, axes = plt.subplots(1, 2, figsize=(15.4, 6.3), dpi=220, constrained_layout=True)
    ax0, ax1 = axes

    ax0.plot(norm_cmd, perf_force, color="#0f766e", linewidth=2.4, label=f"16V performance table")
    ax0.scatter([-1.0, 1.0], [float(perf_force[0]), float(perf_force[-1])], color="#0f766e", s=30, zorder=3)
    ax0.axvline(0.0, color="#94a3b8", linewidth=1.0, linestyle="--")
    ax0.axhline(0.0, color="#cbd5e1", linewidth=1.0)
    ax0.set_title("16V performance table mapped to normalized command")
    ax0.set_xlabel("Normalized command")
    ax0.set_ylabel("Force [N]")
    ax0.grid(True, alpha=0.24)
    ax0.legend(frameon=False, loc="upper left")
    ax0.text(
        -0.98,
        float(perf_force[0]) + 2.0,
        f"1100us: {float(perf_force[0]):.1f}N",
        fontsize=11.0,
        color="#475569",
        ha="left",
        va="bottom",
    )
    ax0.text(
        0.78,
        float(perf_force[-1]) - 3.0,
        f"1900us: {float(perf_force[-1]):.1f}N",
        fontsize=11.0,
        color="#475569",
        ha="left",
        va="top",
    )

    ax1.plot(pwm[center_mask], perf_force[center_mask], color="#0f766e", linewidth=2.4)
    ax1.axvspan(plateau_min, plateau_max, color="#dbeafe", alpha=0.75, label="0-force plateau")
    ax1.axvline(1500.0, color="#94a3b8", linewidth=1.0, linestyle="--")
    ax1.axhline(0.0, color="#cbd5e1", linewidth=1.0)
    ax1.set_title("Center zoom: reduced 0-force plateau")
    ax1.set_xlabel("PWM [us]")
    ax1.set_ylabel("Force [N]")
    ax1.grid(True, alpha=0.24)
    ax1.legend(frameon=False, loc="upper left")
    ax1.text(
        1462.0,
        max(float(np.max(perf_force[center_mask])) * 0.88, 0.55),
        f"0N only from {plateau_min:.0f}us to {plateau_max:.0f}us",
        fontsize=11.0,
        color="#475569",
        ha="left",
        va="top",
    )
    ax1.text(
        1462.0,
        max(float(np.max(perf_force[center_mask])) * 0.62, 0.25),
        "Outside this band the table immediately returns\nsmall positive / negative thrust.",
        fontsize=10.8,
        color="#475569",
        ha="left",
        va="top",
    )

    fig.savefig(FIG_DIR / "uuv_v22_latest_thruster_performance_curve.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_buoyancy_layout_compare(profile: dict) -> None:
    summary = composite_summary(profile)
    com = summary["com"]
    cob = summary["buoy_center"]
    point_sites = [np.asarray(item["pos"], dtype=np.float64) for item in profile["buoyancy_points"]]

    fig, axes = plt.subplots(1, 2, figsize=(14.8, 6.6), dpi=220, constrained_layout=True)
    side_ax, top_ax = axes

    side_ax.set_title("Side view: single-point vs 4-point buoyancy")
    side_ax.plot([-0.34, 0.34], [-0.10, -0.10], color="#cbd5e1", linewidth=18, solid_capstyle="round", alpha=0.85)
    side_ax.plot([-0.20, 0.20], [0.02, 0.02], color="#e2e8f0", linewidth=24, solid_capstyle="round", alpha=0.9)
    side_ax.scatter([com[0]], [com[2]], marker="x", s=120, color="black", label="CoM")
    side_ax.scatter([cob[0]], [cob[2]], marker="^", s=120, color="#16a34a", label="CoB / center buoyancy point")
    for point in point_sites:
        side_ax.scatter([point[0]], [point[2]], s=76, color="#2563eb")
    side_ax.text(cob[0] + 0.012, cob[2] + 0.010, "single-point", color="#166534", fontsize=12)
    side_ax.text(point_sites[0][0] + 0.015, point_sites[0][2] + 0.008, "4-point", color="#1d4ed8", fontsize=12)
    side_ax.set_xlabel("X [m]")
    side_ax.set_ylabel("Z [m]")
    side_ax.grid(True, alpha=0.22)
    side_ax.set_xlim(-0.32, 0.22)
    side_ax.set_ylim(-0.12, 0.11)
    side_ax.legend(frameon=False, loc="upper left")

    top_ax.set_title("Top view: restoring moment arm distribution")
    top_ax.plot([-0.34, 0.34], [0.20, 0.20], color="#cbd5e1", linewidth=12, solid_capstyle="round", alpha=0.85)
    top_ax.plot([-0.34, 0.34], [-0.20, -0.20], color="#cbd5e1", linewidth=12, solid_capstyle="round", alpha=0.85)
    top_ax.scatter([com[0]], [com[1]], marker="x", s=120, color="black", label="CoM")
    top_ax.scatter([cob[0]], [cob[1]], marker="^", s=120, color="#16a34a", label="Center model point")
    top_ax.scatter([point[0] for point in point_sites], [point[1] for point in point_sites], s=80, color="#2563eb", label="4-point model")
    for point in point_sites:
        top_ax.plot([com[0], point[0]], [com[1], point[1]], color="#93c5fd", linewidth=1.2, alpha=0.9)
    top_ax.plot([com[0], cob[0]], [com[1], cob[1]], color="#86efac", linewidth=1.4, alpha=0.95)
    top_ax.set_xlabel("X [m]")
    top_ax.set_ylabel("Y [m]")
    top_ax.grid(True, alpha=0.22)
    top_ax.set_xlim(-0.24, 0.20)
    top_ax.set_ylim(-0.30, 0.30)
    top_ax.legend(frameon=False, loc="upper left")

    fig.savefig(FIG_DIR / "uuv_v22_latest_buoyancy_layout_compare.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_buoyancy_variant_responses(center_forward: dict, four_forward: dict, center_heave: dict, four_heave: dict, center_release: dict, four_release: dict) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(15.0, 10.6), dpi=220, constrained_layout=True)
    color_center = "#b91c1c"
    color_four = "#0f766e"

    for ax in axes.flat:
        ax.grid(True, alpha=0.24)
        ax.set_axisbelow(True)

    axes[0, 0].plot(center_forward["time"], center_forward["surge"], color=color_center, linewidth=2.1, label="Single-point center")
    axes[0, 0].plot(four_forward["time"], four_forward["surge"], color=color_four, linewidth=2.1, label="4-point current")
    axes[0, 0].axvspan(center_forward["active_start_s"], center_forward["active_end_s"], color="#e5e7eb", alpha=0.7)
    axes[0, 0].set_title("Forward pulse: surge")
    axes[0, 0].set_ylabel("Surge [m/s]")
    axes[0, 0].legend(frameon=False, loc="upper left")

    axes[0, 1].plot(center_forward["time"], np.abs(center_forward["pitch_deg"]), color=color_center, linewidth=2.1)
    axes[0, 1].plot(four_forward["time"], np.abs(four_forward["pitch_deg"]), color=color_four, linewidth=2.1)
    axes[0, 1].axvspan(center_forward["active_start_s"], center_forward["active_end_s"], color="#e5e7eb", alpha=0.7)
    axes[0, 1].set_title("Forward pulse: |pitch|")
    axes[0, 1].set_ylabel("|pitch| [deg]")

    axes[1, 0].plot(center_heave["time"], center_heave["depth_m"], color=color_center, linewidth=2.1)
    axes[1, 0].plot(four_heave["time"], four_heave["depth_m"], color=color_four, linewidth=2.1)
    axes[1, 0].axvspan(center_heave["active_start_s"], center_heave["active_end_s"], color="#e5e7eb", alpha=0.7)
    axes[1, 0].set_title("Heave pulse: depth")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Depth [m]")

    axes[1, 1].plot(center_release["time"], center_release["angle_norm_deg"], color=color_center, linewidth=2.1)
    axes[1, 1].plot(four_release["time"], four_release["angle_norm_deg"], color=color_four, linewidth=2.1)
    axes[1, 1].set_title("Attitude release: sqrt(roll^2 + pitch^2)")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Angle norm [deg]")

    fig.savefig(FIG_DIR / "uuv_v22_latest_buoyancy_variant_responses.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_buoyancy_variant_metrics(center_metrics: dict, four_metrics: dict, actual_ref: dict) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(15.2, 6.4), dpi=220, constrained_layout=True)
    color_center = "#b91c1c"
    color_four = "#0f766e"
    color_actual = "#6b7280"

    metric_keys = [
        ("forward_peak_surge_mps", "Forward peak surge [m/s]"),
        ("forward_peak_pitch_deg", "Forward peak |pitch| [deg]"),
        ("forward_peak_roll_deg", "Forward peak |roll| [deg]"),
        ("heave_depth_delta_mm", "Heave depth delta [mm]"),
    ]
    x = np.arange(len(metric_keys))
    width = 0.24
    center_vals = [
        center_metrics["forward_step"]["peak_surge_mps"],
        center_metrics["forward_step"]["peak_pitch_deg"],
        center_metrics["forward_step"]["peak_roll_deg"],
        center_metrics["heave_step"]["depth_delta_mm"],
    ]
    four_vals = [
        four_metrics["forward_step"]["peak_surge_mps"],
        four_metrics["forward_step"]["peak_pitch_deg"],
        four_metrics["forward_step"]["peak_roll_deg"],
        four_metrics["heave_step"]["depth_delta_mm"],
    ]
    actual_vals = [actual_ref[key] for key, _ in metric_keys]
    axes[0].bar(x - width, center_vals, width=width, color=color_center, label="Single-point center")
    axes[0].bar(x, four_vals, width=width, color=color_four, label="4-point current")
    axes[0].bar(x + width, actual_vals, width=width, color=color_actual, label="Latest actual reference")
    axes[0].set_xticks(x, [label for _, label in metric_keys], rotation=15, ha="right")
    axes[0].set_title("Variant metrics vs latest actual reference")
    axes[0].grid(True, axis="y", alpha=0.25)
    axes[0].legend(frameon=False, loc="upper left")

    gap_labels = ["surge gap", "pitch gap", "roll gap", "heave gap", "mean gap"]
    center_gap_vals = [
        center_metrics["actual_gap_percent"]["surge_gap_pct"],
        center_metrics["actual_gap_percent"]["pitch_gap_pct"],
        center_metrics["actual_gap_percent"]["roll_gap_pct"],
        center_metrics["actual_gap_percent"]["heave_gap_pct"],
        center_metrics["actual_gap_index_pct"],
    ]
    four_gap_vals = [
        four_metrics["actual_gap_percent"]["surge_gap_pct"],
        four_metrics["actual_gap_percent"]["pitch_gap_pct"],
        four_metrics["actual_gap_percent"]["roll_gap_pct"],
        four_metrics["actual_gap_percent"]["heave_gap_pct"],
        four_metrics["actual_gap_index_pct"],
    ]
    x2 = np.arange(len(gap_labels))
    axes[1].bar(x2 - 0.18, center_gap_vals, width=0.36, color=color_center, label="Single-point center")
    axes[1].bar(x2 + 0.18, four_gap_vals, width=0.36, color=color_four, label="4-point current")
    axes[1].set_xticks(x2, gap_labels, rotation=12, ha="right")
    axes[1].set_title("Gap to latest actual reference")
    axes[1].set_ylabel("Relative error [%]")
    axes[1].grid(True, axis="y", alpha=0.25)

    fig.savefig(FIG_DIR / "uuv_v22_latest_buoyancy_variant_metrics.png", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def build_focus_metrics(profile: dict, thruster_global: dict, actual_ref: dict) -> dict:
    forward_cmd = 0.5
    heave_cmd = 0.023
    center_forward = simulate_phase_step("center", profile, thruster_global, forward_cmd=forward_cmd, heave_cmd=0.0)
    four_forward = simulate_phase_step("four_point", profile, thruster_global, forward_cmd=forward_cmd, heave_cmd=0.0)
    center_heave = simulate_phase_step("center", profile, thruster_global, forward_cmd=0.0, heave_cmd=heave_cmd)
    four_heave = simulate_phase_step("four_point", profile, thruster_global, forward_cmd=0.0, heave_cmd=heave_cmd)
    center_release = simulate_attitude_release("center", profile, thruster_global)
    four_release = simulate_attitude_release("four_point", profile, thruster_global)

    center_metrics = build_variant_metrics("center", center_forward, center_heave, center_release, actual_ref)
    four_metrics = build_variant_metrics("four_point", four_forward, four_heave, four_release, actual_ref)

    plot_buoyancy_variant_responses(center_forward, four_forward, center_heave, four_heave, center_release, four_release)
    plot_buoyancy_variant_metrics(center_metrics, four_metrics, actual_ref)

    return {
        "comparison_commands": {
            "forward_cmd_norm": forward_cmd,
            "heave_cmd_norm": heave_cmd,
        },
        "actual_reference": actual_ref,
        "variants": {
            "center": center_metrics,
            "four_point": four_metrics,
        },
    }


def main() -> None:
    ensure_dirs()
    profiles = load_json(CONFIG_PATH)
    current_profile = profiles["current"]
    thruster_params = load_json(THRUSTER_PARAMS_PATH)
    thruster_global = thruster_params["global"]
    current_step = load_json(CURRENT_STEP_SUMMARY_PATH)
    _ = load_json(CURRENT_STEP_EVENTS_PATH)

    actual_ref = build_actual_reference(current_step)

    plot_current_engine_block_diagram()
    plot_thruster_model_figure(current_profile, thruster_global)
    plot_thruster_performance_curve(current_profile, thruster_global)
    plot_buoyancy_layout_compare(current_profile)
    focus_metrics = build_focus_metrics(current_profile, thruster_global, actual_ref)

    payload = {
        "current_profile_snapshot": {
            "thruster_force_max": float(current_profile["thruster_force_max"]),
            "vertical_thruster_gain_scale": float(current_profile["vertical_thruster_gain_scale"]),
            "yaw_torque_scale": float(current_profile["yaw_torque_scale"]),
            "buoyancy_scale": float(current_profile["buoyancy_scale"]),
            "cob_torque_scale": float(current_profile["cob_torque_scale"]),
            "cob_x_offset": float(current_profile["cob_x_offset"]),
            "cob_z_offset": float(current_profile["cob_z_offset"]),
        },
        "thruster_model_snapshot": {
            "deadzone": float(thruster_global["deadzone"]),
            "tau_up": float(thruster_global["tau_up"]),
            "tau_down": float(thruster_global["tau_down"]),
            "reverse_asymmetry": float(thruster_global["reverse_asymmetry"]),
            "gain_scale_all": float(thruster_global["gain_scale_all"]),
            "command_limit": float(thruster_global["command_limit"]),
            "forward_poly": [float(v) for v in thruster_global["forward_poly"]],
            "reverse_poly": [float(v) for v in thruster_global["reverse_poly"]],
        },
        **focus_metrics,
    }
    FOCUS_METRICS_PATH.write_text(json.dumps(payload, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
