#!/usr/bin/env python3
"""Generate concrete comparison figures for active UUV physics engines.

The figures in this script are not conceptual diagrams. They are produced by
running headless MuJoCo simulations with the same MJCF scenes and hydrodynamic
logic that the runtime uses:

- legacy baseline: scenes/tank_legacy_scene.xml + profile "legacy"
- current model: scenes/tank_current_scene.xml + profile "current"
- archived legacy custom: delete/competition_scene.xml + reconstructed 2026-03
  sim_simple / thruster settings from archived logs

Outputs:
- figures/engine_forward_step_comparison.png
- figures/engine_yaw_step_comparison.png
- figures/engine_summary_metrics.png
- figures/active_custom_hydrodynamic_coeffs.png
- engine_comparison_metrics.json
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "uuv_mujoco" / "v2.2"
DOC_DIR = ROOT / "document"
FIG_DIR = DOC_DIR / "figures"
METRICS_PATH = DOC_DIR / "engine_comparison_metrics.json"
THRUSTER_PERF_PATH = SIM_DIR / "config" / "thruster_performance.json"
THRUSTER_PARAMS_PATH = SIM_DIR / "config" / "thruster_params.json"
CURRENT_PROFILE_PATH = SIM_DIR / "config" / "sim_profiles.json"

sys.path.insert(0, str(SIM_DIR))

from physics.hydrodynamics_helpers import (  # noqa: E402
    added_mass_coriolis,
    first_order_response,
    scaled_polynomial_force,
    shape_thruster_command,
)
from physics.sim_profile_helpers import (  # noqa: E402
    DEFAULT_SIM_PROFILES,
    build_hydrodynamics_config,
    build_sim_profile,
    load_sim_profiles,
)
from physics.thruster_mapping import PHYSICAL_YAW_THRUSTERS, PHYSICAL_VERTICAL_THRUSTERS  # noqa: E402


CURRENT_FORWARD_STEP = (0.5, 1.5, 0.16)
CURRENT_YAW_STEP = (0.5, 1.2, 0.16)
SIM_DURATION = 3.0


@dataclass(frozen=True)
class Scenario:
    key: str
    label: str
    scene_path: Path
    fluid_model: str
    profile: dict[str, Any]
    thruster_global: dict[str, Any]
    per_thruster_gain: dict[str, float]
    note: str


@dataclass
class RuntimeContext:
    scenario: Scenario
    model: mujoco.MjModel
    data: mujoco.MjData
    base_id: int
    cob_site_id: int
    act: dict[str, int]
    horiz_order: tuple[str, ...]
    horiz_pinv: np.ndarray
    vehicle_mass: float
    neutral_volume: float
    hydro_cfg: Any
    perf_pwm: np.ndarray | None
    perf_force: np.ndarray | None
    thruster_scale: dict[str, float]
    thruster_state: dict[str, float]
    thruster_target: dict[str, float]
    thruster_force_cmd: dict[str, float]
    thruster_reaction_torque_world: np.ndarray
    prev_rel_nu_body: np.ndarray
    water_surface_z: float
    rho: float
    g: float
    half_height: float


def _body_children(model: mujoco.MjModel) -> list[list[int]]:
    children = [[] for _ in range(model.nbody)]
    for body_idx in range(1, model.nbody):
        parent_idx = int(model.body_parentid[body_idx])
        if 0 <= parent_idx < model.nbody:
            children[parent_idx].append(body_idx)
    return children


def body_subtree_mass(model: mujoco.MjModel, root_body_id: int) -> float:
    children = _body_children(model)
    total = 0.0
    stack = [int(root_body_id)]
    while stack:
        bid = stack.pop()
        total += float(model.body_mass[bid])
        stack.extend(children[bid])
    return total


def normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        return np.zeros_like(v)
    return v / n


def build_horizontal_allocator(
    model: mujoco.MjModel,
    base_id: int,
    act: dict[str, int],
    horiz_order: tuple[str, ...],
) -> np.ndarray:
    com_body = model.body_ipos[base_id].copy()
    alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
    for i, name in enumerate(horiz_order):
        aid = act[name]
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
        fdir = normalize(model.actuator_gear[aid, :3].copy())
        r = model.site_pos[sid].copy() - com_body
        tau = np.cross(r, fdir)
        alloc[:, i] = np.array([fdir[0], fdir[1], tau[2]], dtype=np.float64)

    row_scale = np.sum(np.abs(alloc), axis=1)
    row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
    return np.linalg.pinv(alloc / row_scale[:, None])


def mix_horizontal_thrusters(horiz_pinv: np.ndarray, fwd_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
    wrench_cmd = np.array([fwd_cmd, sway_cmd, yaw_cmd], dtype=np.float64)
    u = horiz_pinv @ wrench_cmd
    max_abs = float(np.max(np.abs(u)))
    if max_abs > 1.0:
        u /= max_abs
    return np.clip(u, -1.0, 1.0)


def load_thruster_curve(voltage: float) -> tuple[np.ndarray | None, np.ndarray | None]:
    payload = json.loads(THRUSTER_PERF_PATH.read_text())
    curves = payload.get("curves", [])
    candidates = []
    for curve in curves:
        if not isinstance(curve, dict):
            continue
        pwm = np.asarray(curve.get("pwm_us", []), dtype=np.float64)
        force = np.asarray(curve.get("force_n", []), dtype=np.float64)
        if pwm.size < 2 or pwm.size != force.size:
            continue
        order = np.argsort(pwm)
        candidates.append(
            {
                "voltage": float(curve.get("voltage_v", voltage)),
                "pwm": pwm[order],
                "force": force[order],
            }
        )
    if not candidates:
        return None, None
    selected = min(candidates, key=lambda item: abs(item["voltage"] - float(voltage)))
    return selected["pwm"], selected["force"]


def pwm_to_force_from_perf(norm_cmd: float, pwm: np.ndarray, force: np.ndarray) -> float:
    pwm_us = float(np.clip(norm_cmd, -1.0, 1.0) * 400.0 + 1500.0)
    return float(np.interp(pwm_us, pwm, force))


def current_thruster_params() -> tuple[dict[str, Any], dict[str, float]]:
    payload = json.loads(THRUSTER_PARAMS_PATH.read_text())
    global_cfg = dict(payload.get("global", {}))
    per_thruster = {}
    for name, cfg in payload.get("per_thruster", {}).items():
        if isinstance(cfg, dict):
            per_thruster[name] = float(cfg.get("gain_scale", 1.0))
    return global_cfg, per_thruster


def legacy_profile() -> dict[str, Any]:
    # Reconstructed from archived 2026-03 competition_scene + sim_simple logs.
    return {
        "half_height": 0.147,
        "buoyancy_scale": 1.0,
        "cob_torque_scale": 0.20,
        "buoyancy_point_blend": 1.0,
        "cob_x_offset": -0.003,
        "cob_z_offset": 0.010,
        "thruster_voltage": 20.0,
        "thruster_force_max": 65.0,
        "linear_drag": 1.10,
        "angular_drag": 0.32,
        "linear_damping_linear": [1.10, 1.32, 1.54],
        "linear_damping_angular": [0.32, 0.36, 0.28],
        "quadratic_damping_linear": [1.40, 2.00, 2.40],
        "quadratic_damping_angular": [0.10, 0.12, 0.08],
        "added_mass_linear": [1.80, 2.30, 3.00],
        "added_mass_angular": [0.10, 0.12, 0.08],
        "current_world": [0.0, 0.0, 0.0],
        "spin_gain": 22.0,
    }


def legacy_thruster_params() -> tuple[dict[str, Any], dict[str, float]]:
    global_cfg = {
        "deadzone": 0.05,
        "tau_up": 0.12,
        "tau_down": 0.18,
        "reverse_asymmetry": 0.75,
        "command_limit": 0.65,
        "reaction_torque_gain": 0.012,
        "gain_scale_all": 1.0,
        "forward_poly": [0.0, 3.5, 7.0, 12.0],
        "reverse_poly": [0.0, 2.8, 5.5, 9.5],
    }
    names = list(PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS)
    return global_cfg, {name: 1.0 for name in names}


def build_scenarios() -> list[Scenario]:
    profiles, _ = load_sim_profiles(CURRENT_PROFILE_PATH)
    current_global, current_per_thruster = current_thruster_params()
    legacy_global, legacy_per_thruster = legacy_thruster_params()

    current_custom = build_sim_profile(profiles, "legacy")
    current_ellipsoid = build_sim_profile(profiles, "current")
    legacy_scene_path = SIM_DIR / "delete" / "competition_scene.xml"
    legacy_note = "Archived competition_scene + sim_simple log reconstruction"
    if not legacy_scene_path.exists():
        legacy_scene_path = SIM_DIR / "scenes" / "tank_legacy_scene.xml"
        legacy_note = "Legacy 2026-03 profile reconstructed on current custom scene scaffold"

    return [
        Scenario(
            key="legacy_custom",
            label="Legacy Custom (2026-03)",
            scene_path=legacy_scene_path,
            fluid_model="custom",
            profile=legacy_profile(),
            thruster_global=legacy_global,
            per_thruster_gain=legacy_per_thruster,
            note=legacy_note,
        ),
        Scenario(
            key="current_custom",
            label="Current Custom",
            scene_path=SIM_DIR / "scenes" / "tank_legacy_scene.xml",
            fluid_model="legacy",
            profile=current_custom,
            thruster_global=current_global,
            per_thruster_gain=current_per_thruster,
            note="Active legacy runtime path",
        ),
        Scenario(
            key="current_ellipsoid",
            label="Current Ellipsoid",
            scene_path=SIM_DIR / "scenes" / "tank_current_scene.xml",
            fluid_model="current",
            profile=current_ellipsoid,
            thruster_global=current_global,
            per_thruster_gain=current_per_thruster,
            note="Active built-in ellipsoid runtime path",
        ),
    ]


def prepare_runtime(scenario: Scenario) -> RuntimeContext:
    scene_path = scenario.scene_path
    if scene_path.parent.name == "delete":
        scene_text = scene_path.read_text()
        scene_text = scene_text.replace(
            'meshdir="assets/urdf_full/meshes_split/"',
            'meshdir="../assets/urdf_full/meshes_split/"',
        )
        with tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".xml",
            prefix="legacy_scene_",
            dir=str(scene_path.parent),
            delete=False,
        ) as tmp:
            tmp.write(scene_text)
            tmp_path = Path(tmp.name)
        try:
            model = mujoco.MjModel.from_xml_path(str(tmp_path))
        finally:
            tmp_path.unlink(missing_ok=True)
    else:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        raise RuntimeError(f"base_link not found in {scenario.scene_path}")

    cob_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cob_site")
    if cob_site_id >= 0:
        model.site_pos[cob_site_id][0] = float(model.body_ipos[base_id][0] + scenario.profile.get("cob_x_offset", 0.0))
        model.site_pos[cob_site_id][2] = float(model.body_ipos[base_id][2] + scenario.profile.get("cob_z_offset", 0.0))
        mujoco.mj_forward(model, data)

    act = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
        for i in range(model.nu)
    }
    horiz_order = tuple(PHYSICAL_YAW_THRUSTERS)
    horiz_pinv = build_horizontal_allocator(model, base_id, act, horiz_order)

    rho = float(model.opt.density)
    g = abs(float(model.opt.gravity[2]))
    vehicle_mass = body_subtree_mass(model, base_id)
    neutral_volume = vehicle_mass / max(rho, 1e-9)

    active_voltage = float(scenario.profile.get("thruster_voltage", 20.0))
    perf_pwm, perf_force = load_thruster_curve(active_voltage)
    perf_force_max = None
    if perf_force is not None and perf_force.size > 0:
        perf_force_max = float(np.max(np.abs(perf_force)))
    hydro_cfg = build_hydrodynamics_config(
        scenario.profile,
        perf_force_max=perf_force_max,
        fluid_density=rho,
    )
    if hydro_cfg.displaced_volume is not None and hydro_cfg.displaced_volume > 0.0:
        neutral_volume = float(hydro_cfg.displaced_volume)

    names = list(PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS)
    gain_scale_all = float(np.clip(scenario.thruster_global.get("gain_scale_all", 1.0), 0.1, 20.0))
    thruster_scale = {}
    for name in names:
        base_gain = float(np.clip(scenario.per_thruster_gain.get(name, 1.0), 0.1, 20.0))
        thruster_scale[name] = float(np.clip(base_gain * gain_scale_all, 0.1, 20.0))

    return RuntimeContext(
        scenario=scenario,
        model=model,
        data=data,
        base_id=base_id,
        cob_site_id=cob_site_id,
        act=act,
        horiz_order=horiz_order,
        horiz_pinv=horiz_pinv,
        vehicle_mass=vehicle_mass,
        neutral_volume=neutral_volume,
        hydro_cfg=hydro_cfg,
        perf_pwm=perf_pwm,
        perf_force=perf_force,
        thruster_scale=thruster_scale,
        thruster_state={name: 0.0 for name in names},
        thruster_target={name: 0.0 for name in names},
        thruster_force_cmd={name: 0.0 for name in names},
        thruster_reaction_torque_world=np.zeros(3, dtype=np.float64),
        prev_rel_nu_body=np.zeros(6, dtype=np.float64),
        water_surface_z=0.0,
        rho=rho,
        g=g,
        half_height=float(hydro_cfg.half_height),
    )


def body_velocity_local(model: mujoco.MjModel, data: mujoco.MjData, base_id: int) -> tuple[np.ndarray, np.ndarray]:
    vel6 = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(
        model,
        data,
        mujoco.mjtObj.mjOBJ_BODY,
        int(base_id),
        vel6,
        1,
    )
    ang_local = vel6[:3].copy()
    lin_local = vel6[3:].copy()
    return lin_local, ang_local


def force_from_shaped_command(ctx: RuntimeContext, command_shaped: float, gain: float) -> float:
    if abs(command_shaped) <= 1e-9:
        return 0.0
    if ctx.perf_pwm is not None and ctx.perf_force is not None:
        return pwm_to_force_from_perf(command_shaped, ctx.perf_pwm, ctx.perf_force) * gain

    force_max = float(ctx.hydro_cfg.thruster_force_max)
    magnitude = abs(command_shaped)
    forward_poly = ctx.scenario.thruster_global.get("forward_poly", [0.0, 3.5, 7.0, 12.0])
    reverse_poly = ctx.scenario.thruster_global.get("reverse_poly", [0.0, 2.8, 5.5, 9.5])
    reverse_asymmetry = float(np.clip(ctx.scenario.thruster_global.get("reverse_asymmetry", 0.75), 0.1, 1.5))
    if command_shaped >= 0.0:
        return scaled_polynomial_force(magnitude, forward_poly, force_max) * gain
    reverse_force_max = force_max * reverse_asymmetry
    return -scaled_polynomial_force(magnitude, reverse_poly, reverse_force_max) * gain


def update_thruster_forces(ctx: RuntimeContext, dt: float) -> None:
    data = ctx.data
    model = ctx.model
    base_rot = data.xmat[ctx.base_id].reshape(3, 3)
    ctx.thruster_reaction_torque_world[:] = 0.0

    deadzone = float(np.clip(ctx.scenario.thruster_global.get("deadzone", 0.05), 0.0, 0.95))
    tau_up = float(max(ctx.scenario.thruster_global.get("tau_up", 0.12), 1e-4))
    tau_down = float(max(ctx.scenario.thruster_global.get("tau_down", 0.18), 1e-4))
    command_limit = float(np.clip(ctx.scenario.thruster_global.get("command_limit", 0.65), deadzone + 1e-3, 1.0))
    reaction_torque_gain = float(max(ctx.scenario.thruster_global.get("reaction_torque_gain", 0.012), 0.0))

    for name in ctx.thruster_state:
        aid = ctx.act[name]
        lo, hi = model.actuator_ctrlrange[aid]
        gain = float(ctx.thruster_scale.get(name, 1.0))
        target_norm = float(np.clip(ctx.thruster_target[name], -1.0, 1.0))
        ctx.thruster_state[name] = first_order_response(ctx.thruster_state[name], target_norm, dt, tau_up, tau_down)
        shaped_cmd = shape_thruster_command(ctx.thruster_state[name], deadzone, command_limit)
        force = force_from_shaped_command(ctx, shaped_cmd, gain)
        force = float(np.clip(force, lo, hi))
        data.ctrl[aid] = force
        ctx.thruster_force_cmd[name] = force

        fdir = normalize(model.actuator_gear[aid, :3].copy())
        world_dir = base_rot @ fdir
        spin_sign = 1.0 if name.endswith(("lf", "rr")) else -1.0
        ctx.thruster_reaction_torque_world += -spin_sign * world_dir * force * reaction_torque_gain


def apply_underwater_wrench(ctx: RuntimeContext, dt: float) -> None:
    model = ctx.model
    data = ctx.data
    base_id = ctx.base_id
    data.xfrc_applied[base_id, :] = 0.0

    com = data.xipos[base_id].copy()
    base_rot = data.xmat[base_id].reshape(3, 3)
    base_origin = data.xpos[base_id].copy()
    cob = data.site_xpos[ctx.cob_site_id].copy() if ctx.cob_site_id >= 0 else com
    buoyancy_point_blend = float(ctx.hydro_cfg.buoyancy_point_blend)
    buoy_point = (1.0 - buoyancy_point_blend) * com + buoyancy_point_blend * cob

    depth = ctx.water_surface_z - float(base_origin[2])
    frac = float(np.clip((depth + ctx.half_height) / (2.0 * ctx.half_height), 0.0, 1.0))
    buoy = ctx.rho * ctx.g * ctx.neutral_volume * frac * float(ctx.hydro_cfg.buoyancy_scale)
    buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)
    data.xfrc_applied[base_id, 0:3] += buoy_force_world
    if abs(float(ctx.hydro_cfg.cob_torque_scale)) > 1e-9:
        r = buoy_point - com
        data.xfrc_applied[base_id, 3:6] += np.cross(r, buoy_force_world) * float(ctx.hydro_cfg.cob_torque_scale)

    if ctx.scenario.fluid_model == "custom":
        lin_vel_body, ang_vel_body = body_velocity_local(model, data, base_id)
        current_body = base_rot.T @ ctx.hydro_cfg.water_current_world
        rel_lin_vel_body = lin_vel_body - current_body
        nu_rel_body = np.concatenate((rel_lin_vel_body, ang_vel_body))
        if dt > 0.0:
            rel_acc_body = (nu_rel_body - ctx.prev_rel_nu_body) / max(dt, 1e-6)
        else:
            rel_acc_body = np.zeros(6, dtype=np.float64)
        ctx.prev_rel_nu_body = nu_rel_body.copy()

        submerged = frac
        immersed_added_mass = ctx.hydro_cfg.added_mass_diag * submerged
        immersed_linear_damping = ctx.hydro_cfg.air_linear_damping_diag + submerged * (
            ctx.hydro_cfg.linear_damping_diag - ctx.hydro_cfg.air_linear_damping_diag
        )
        immersed_quadratic_damping = ctx.hydro_cfg.quadratic_damping_diag * submerged

        hydro_wrench_body = np.zeros(6, dtype=np.float64)
        if np.any(immersed_added_mass > 1e-9):
            hydro_wrench_body -= immersed_added_mass * rel_acc_body
            hydro_wrench_body -= added_mass_coriolis(immersed_added_mass, nu_rel_body) @ nu_rel_body
        hydro_wrench_body -= immersed_linear_damping * nu_rel_body
        hydro_wrench_body -= immersed_quadratic_damping * np.abs(nu_rel_body) * nu_rel_body

        data.xfrc_applied[base_id, 0:3] += base_rot @ hydro_wrench_body[:3]
        data.xfrc_applied[base_id, 3:6] += base_rot @ hydro_wrench_body[3:]
    else:
        ctx.prev_rel_nu_body[:] = 0.0

    data.xfrc_applied[base_id, 3:6] += ctx.thruster_reaction_torque_world


def quat_to_pitch_yaw_deg(quat_wxyz: np.ndarray) -> tuple[float, float]:
    w, x, y, z = [float(v) for v in quat_wxyz]
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return pitch, yaw


def simulate_step_case(scenario: Scenario, mode: str) -> dict[str, np.ndarray]:
    ctx = prepare_runtime(scenario)
    data = ctx.data
    model = ctx.model
    dt = float(model.opt.timestep)
    steps = int(math.ceil(SIM_DURATION / dt))

    times = np.zeros(steps, dtype=np.float64)
    surge = np.zeros(steps, dtype=np.float64)
    pitch_deg = np.zeros(steps, dtype=np.float64)
    yaw_rate = np.zeros(steps, dtype=np.float64)
    yaw_deg = np.zeros(steps, dtype=np.float64)

    if mode == "forward":
        t_on, t_off, amp = CURRENT_FORWARD_STEP
    elif mode == "yaw":
        t_on, t_off, amp = CURRENT_YAW_STEP
    else:
        raise ValueError(mode)

    for step in range(steps):
        t = float(data.time)
        for name in ctx.thruster_target:
            ctx.thruster_target[name] = 0.0

        if t_on <= t < t_off:
            if mode == "forward":
                horiz_cmd = mix_horizontal_thrusters(ctx.horiz_pinv, amp, 0.0, 0.0)
            else:
                horiz_cmd = mix_horizontal_thrusters(ctx.horiz_pinv, 0.0, 0.0, amp)
        else:
            horiz_cmd = np.zeros(len(ctx.horiz_order), dtype=np.float64)

        for i, name in enumerate(ctx.horiz_order):
            ctx.thruster_target[name] = float(horiz_cmd[i])

        update_thruster_forces(ctx, dt)
        apply_underwater_wrench(ctx, dt)
        mujoco.mj_step(model, data)

        lin_vel_body, ang_vel_body = body_velocity_local(model, data, ctx.base_id)
        pitch, yaw = quat_to_pitch_yaw_deg(data.xquat[ctx.base_id].copy())

        times[step] = float(data.time)
        surge[step] = float(lin_vel_body[0])
        pitch_deg[step] = float(pitch)
        yaw_rate[step] = float(ang_vel_body[2])
        yaw_deg[step] = float(yaw)

    return {
        "time": times,
        "surge": surge,
        "pitch_deg": pitch_deg,
        "yaw_rate": yaw_rate,
        "yaw_deg": yaw_deg,
        "t_on": np.array([t_on], dtype=np.float64),
        "t_off": np.array([t_off], dtype=np.float64),
    }


def compute_capability_metrics(scenario: Scenario) -> dict[str, float]:
    ctx = prepare_runtime(scenario)
    horiz_forward = mix_horizontal_thrusters(ctx.horiz_pinv, 1.0, 0.0, 0.0)
    horiz_yaw = mix_horizontal_thrusters(ctx.horiz_pinv, 0.0, 0.0, 1.0)
    force_forward = np.zeros(3, dtype=np.float64)
    torque_forward = np.zeros(3, dtype=np.float64)
    force_yaw = np.zeros(3, dtype=np.float64)
    torque_yaw = np.zeros(3, dtype=np.float64)
    com_body = ctx.model.body_ipos[ctx.base_id].copy()

    for cmd_vec, out_force, out_torque in (
        (horiz_forward, force_forward, torque_forward),
        (horiz_yaw, force_yaw, torque_yaw),
    ):
        for i, name in enumerate(ctx.horiz_order):
            aid = ctx.act[name]
            sid = mujoco.mj_name2id(ctx.model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            gain = float(ctx.thruster_scale[name])
            shaped = shape_thruster_command(float(cmd_vec[i]), ctx.scenario.thruster_global["deadzone"], ctx.scenario.thruster_global["command_limit"])
            force_mag = force_from_shaped_command(ctx, shaped, gain)
            fdir = normalize(ctx.model.actuator_gear[aid, :3].copy())
            r = ctx.model.site_pos[sid].copy() - com_body
            out_force += fdir * force_mag
            out_torque += np.cross(r, fdir * force_mag)

    z_cob_minus_com_mm = (
        float(ctx.model.site_pos[ctx.cob_site_id][2] - ctx.model.body_ipos[ctx.base_id][2]) * 1000.0
        if ctx.cob_site_id >= 0
        else 0.0
    )
    return {
        "vehicle_mass_kg": float(ctx.vehicle_mass),
        "z_cob_minus_com_mm": z_cob_minus_com_mm,
        "forward_force_n": float(force_forward[0]),
        "yaw_moment_nm": float(torque_yaw[2]),
    }


def summarize_step_metrics(forward: dict[str, np.ndarray], yaw: dict[str, np.ndarray]) -> dict[str, float]:
    t_fwd = forward["time"]
    t_on_fwd = float(forward["t_on"][0])
    t_off_fwd = float(forward["t_off"][0])
    surge = forward["surge"]
    pitch = np.abs(forward["pitch_deg"])

    t_yaw = yaw["time"]
    t_on_yaw = float(yaw["t_on"][0])
    t_off_yaw = float(yaw["t_off"][0])
    yaw_rate = np.abs(yaw["yaw_rate"])

    forward_window = (t_fwd >= t_on_fwd) & (t_fwd <= t_off_fwd + 1e-9)
    release_idx_fwd = int(np.searchsorted(t_fwd, t_off_fwd + 0.6))
    release_idx_fwd = min(release_idx_fwd, len(t_fwd) - 1)
    release_idx_yaw = int(np.searchsorted(t_yaw, t_off_yaw + 0.5))
    release_idx_yaw = min(release_idx_yaw, len(t_yaw) - 1)

    peak_surge = float(np.max(surge[forward_window])) if np.any(forward_window) else 0.0
    residual_surge_ratio = 0.0
    if abs(peak_surge) > 1e-9:
        residual_surge_ratio = float(abs(surge[release_idx_fwd]) / abs(peak_surge))

    yaw_window = (t_yaw >= t_on_yaw) & (t_yaw <= t_off_yaw + 1e-9)
    peak_yaw = float(np.max(yaw_rate[yaw_window])) if np.any(yaw_window) else 0.0
    residual_yaw_ratio = 0.0
    if abs(peak_yaw) > 1e-9:
        residual_yaw_ratio = float(abs(yaw_rate[release_idx_yaw]) / abs(peak_yaw))

    return {
        "peak_surge_mps": peak_surge,
        "peak_pitch_deg": float(np.max(pitch)),
        "residual_surge_ratio": residual_surge_ratio,
        "peak_yaw_rate_radps": peak_yaw,
        "residual_yaw_ratio": residual_yaw_ratio,
    }


def style_axes(ax: plt.Axes, ylabel: str) -> None:
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.set_axisbelow(True)


def generate_forward_figure(results: dict[str, dict[str, dict[str, np.ndarray]]]) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 7.2), sharex=True)
    colors = {
        "legacy_custom": "#7c3aed",
        "current_custom": "#0f766e",
        "current_ellipsoid": "#b45309",
    }
    labels = {
        "legacy_custom": "Legacy Custom (2026-03)",
        "current_custom": "Current Custom",
        "current_ellipsoid": "Current Ellipsoid",
    }

    t_on, t_off, _ = CURRENT_FORWARD_STEP
    for ax in axes:
        ax.axvspan(t_on, t_off, color="#e5e7eb", alpha=0.8, label="Forward step window")

    for key, case in results.items():
        data = case["forward"]
        axes[0].plot(data["time"], data["surge"], label=labels[key], lw=2.3, color=colors[key])
        axes[1].plot(data["time"], data["pitch_deg"], label=labels[key], lw=2.3, color=colors[key])

    axes[0].set_title("Forward Step Response Comparison")
    style_axes(axes[0], "Surge velocity [m/s]")
    style_axes(axes[1], "Pitch angle [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[0].legend(ncol=2, fontsize=9, loc="upper left")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "engine_forward_step_comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def generate_yaw_figure(results: dict[str, dict[str, dict[str, np.ndarray]]]) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10.5, 7.2), sharex=True)
    colors = {
        "legacy_custom": "#7c3aed",
        "current_custom": "#0f766e",
        "current_ellipsoid": "#b45309",
    }
    labels = {
        "legacy_custom": "Legacy Custom (2026-03)",
        "current_custom": "Current Custom",
        "current_ellipsoid": "Current Ellipsoid",
    }

    t_on, t_off, _ = CURRENT_YAW_STEP
    for ax in axes:
        ax.axvspan(t_on, t_off, color="#e5e7eb", alpha=0.8, label="Yaw step window")

    for key, case in results.items():
        data = case["yaw"]
        axes[0].plot(data["time"], data["yaw_rate"], label=labels[key], lw=2.3, color=colors[key])
        axes[1].plot(data["time"], data["yaw_deg"], label=labels[key], lw=2.3, color=colors[key])

    axes[0].set_title("Yaw Step Response Comparison")
    style_axes(axes[0], "Yaw rate [rad/s]")
    style_axes(axes[1], "Heading [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[0].legend(ncol=2, fontsize=9, loc="upper left")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "engine_yaw_step_comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def generate_summary_metrics_figure(metrics: dict[str, dict[str, float]]) -> None:
    labels = {
        "legacy_custom": "Legacy\nCustom",
        "current_custom": "Current\nCustom",
        "current_ellipsoid": "Current\nEllipsoid",
    }
    order = ["legacy_custom", "current_custom", "current_ellipsoid"]
    x = np.arange(len(order))
    bar_color = ["#7c3aed", "#0f766e", "#b45309"]

    panels = [
        ("vehicle_mass_kg", "Vehicle mass [kg]"),
        ("z_cob_minus_com_mm", "zCoB - zCoM [mm]"),
        ("forward_force_n", "Max forward force [N]"),
        ("peak_surge_mps", "Peak surge speed [m/s]"),
        ("peak_pitch_deg", "Peak |pitch| [deg]"),
        ("residual_yaw_ratio", "Yaw residual ratio [-]"),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(12, 7.6))
    axes = axes.ravel()
    for ax, (metric_key, title) in zip(axes, panels):
        values = [metrics[name][metric_key] for name in order]
        bars = ax.bar(x, values, color=bar_color, width=0.62)
        ax.set_title(title, fontsize=11)
        ax.set_xticks(x, [labels[name] for name in order], fontsize=9)
        ax.grid(True, axis="y", alpha=0.25)
        ax.set_axisbelow(True)
        ymin, ymax = ax.get_ylim()
        pad = 0.04 * (ymax - ymin if ymax > ymin else 1.0)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height() + pad, f"{value:.2f}", ha="center", va="bottom", fontsize=8)

    fig.suptitle("Engine Summary Metrics", fontsize=15, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(FIG_DIR / "engine_summary_metrics.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def generate_active_custom_coeff_figure(profile: dict[str, Any]) -> None:
    cfg = build_hydrodynamics_config(profile, fluid_density=1000.0)
    trans_labels = ["surge", "sway", "heave"]
    rot_labels = ["roll", "pitch", "yaw"]

    added = cfg.added_mass_diag
    linear = cfg.linear_damping_diag
    quad = cfg.quadratic_damping_diag

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.4))
    width = 0.24
    x0 = np.arange(3)

    axes[0].bar(x0 - width, added[:3], width=width, label="Added mass")
    axes[0].bar(x0, linear[:3], width=width, label="Linear damping")
    axes[0].bar(x0 + width, quad[:3], width=width, label="Quadratic damping")
    axes[0].set_xticks(x0, trans_labels)
    axes[0].set_title("Current Custom Profile: Translational Axes")
    axes[0].grid(True, axis="y", alpha=0.25)
    axes[0].set_axisbelow(True)

    axes[1].bar(x0 - width, added[3:], width=width, label="Added mass")
    axes[1].bar(x0, linear[3:], width=width, label="Linear damping")
    axes[1].bar(x0 + width, quad[3:], width=width, label="Quadratic damping")
    axes[1].set_xticks(x0, rot_labels)
    axes[1].set_title("Current Custom Profile: Rotational Axes")
    axes[1].grid(True, axis="y", alpha=0.25)
    axes[1].set_axisbelow(True)
    axes[1].legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(FIG_DIR / "active_custom_hydrodynamic_coeffs.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    scenarios = build_scenarios()

    results: dict[str, dict[str, dict[str, np.ndarray]]] = {}
    metrics: dict[str, dict[str, float]] = {}

    for scenario in scenarios:
        forward = simulate_step_case(scenario, "forward")
        yaw = simulate_step_case(scenario, "yaw")
        results[scenario.key] = {"forward": forward, "yaw": yaw}

        combined = compute_capability_metrics(scenario)
        combined.update(summarize_step_metrics(forward, yaw))
        combined["note"] = scenario.note
        combined["scene_path"] = str(scenario.scene_path.relative_to(ROOT))
        combined["fluid_model"] = scenario.fluid_model
        metrics[scenario.key] = combined

    profiles, _ = load_sim_profiles(CURRENT_PROFILE_PATH)
    generate_forward_figure(results)
    generate_yaw_figure(results)
    generate_summary_metrics_figure(metrics)
    generate_active_custom_coeff_figure(build_sim_profile(profiles, "custom"))
    METRICS_PATH.write_text(json.dumps(metrics, indent=2, ensure_ascii=False))
    print(f"[ok] wrote {FIG_DIR / 'engine_forward_step_comparison.png'}")
    print(f"[ok] wrote {FIG_DIR / 'engine_yaw_step_comparison.png'}")
    print(f"[ok] wrote {FIG_DIR / 'engine_summary_metrics.png'}")
    print(f"[ok] wrote {FIG_DIR / 'active_custom_hydrodynamic_coeffs.png'}")
    print(f"[ok] wrote {METRICS_PATH}")


if __name__ == "__main__":
    main()
