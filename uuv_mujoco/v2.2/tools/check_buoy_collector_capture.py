#!/usr/bin/env python3
"""Smoke-check that the front collector is a physical pocket, not a sticky latch."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
RUNTIME = ROOT / "sim" / "runtime" / "course_buoy_runtime.py"
sys.path.insert(0, str(ROOT))

from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402


COLLECTOR_COLLISION_GEOMS = (
    "collector_bottom_rail_left",
    "collector_bottom_rail_right",
    "collector_bottom_cross_back",
    "collector_bottom_cross_front",
    "collector_back_top_cross",
    "collector_post_back_left",
    "collector_post_back_right",
    "collector_post_front_left",
    "collector_post_front_right",
    "collector_bottom_center_rail",
    "collector_back_mid_cross",
    "collector_left_net_proxy",
    "collector_right_net_proxy",
    "collector_back_net_proxy",
    "collector_bottom_net_proxy",
    "collector_top_rail_left",
    "collector_top_rail_right",
    "collector_top_net_proxy",
    "collector_rear_mount_left",
    "collector_rear_mount_right",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def body_id(mujoco, model, name: str) -> int:
    result = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
    require(result >= 0, f"body not found: {name}")
    return result


def geom_id(mujoco, model, name: str) -> int:
    result = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
    require(result >= 0, f"geom not found: {name}")
    return result


def joint_id(mujoco, model, name: str) -> int:
    result = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
    require(result >= 0, f"joint not found: {name}")
    return result


def world_from_base_local(data, base: int, local_pos: np.ndarray) -> np.ndarray:
    rot = np.array(data.xmat[base], dtype=np.float64).reshape(3, 3)
    return np.array(data.xpos[base], dtype=np.float64) + rot @ local_pos


def base_local_from_world(data, base: int, world_pos: np.ndarray) -> np.ndarray:
    rot = np.array(data.xmat[base], dtype=np.float64).reshape(3, 3)
    return rot.T @ (world_pos - np.array(data.xpos[base], dtype=np.float64))


def place_slide_buoy(mujoco, model, data, body: int, prefix: str, target_world: np.ndarray) -> None:
    mujoco.mj_forward(model, data)
    current = np.array(data.xipos[body], dtype=np.float64)
    delta_xy = target_world[0:2] - current[0:2]
    for suffix, delta in (("slide_x", delta_xy[0]), ("slide_y", delta_xy[1])):
        jid = joint_id(mujoco, model, f"{prefix}_{suffix}")
        data.qpos[int(model.jnt_qposadr[jid])] += float(delta)
        data.qvel[int(model.jnt_dofadr[jid])] = 0.0
    mujoco.mj_forward(model, data)


def place_free_buoy(mujoco, model, data, body: int, prefix: str, target_world: np.ndarray) -> None:
    jid = joint_id(mujoco, model, f"{prefix}_free")
    qposadr = int(model.jnt_qposadr[jid])
    dofadr = int(model.jnt_dofadr[jid])
    current = np.array(data.xipos[body], dtype=np.float64)
    data.qpos[qposadr : qposadr + 3] += target_world - current
    data.qpos[qposadr + 3 : qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[dofadr : dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)


def env_float(_name: str, default: float) -> float:
    return default


def env_flag(_name: str, default: bool) -> bool:
    return default


def make_runtime(mujoco, model, data) -> CourseBuoyRuntime:
    return CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=0.0,
        env_float=env_float,
        env_flag=env_flag,
        log=lambda _message: None,
    )


def contact_names(mujoco, model, data) -> set[tuple[str, str]]:
    names: set[tuple[str, str]] = set()
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1) or str(geom1)
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2) or str(geom2)
        names.add(tuple(sorted((name1, name2))))
    return names


def assert_contact(
    mujoco,
    model,
    data,
    *,
    buoy_geom: str,
    collector_geom: str,
    label: str,
) -> None:
    pairs = contact_names(mujoco, model, data)
    expected = tuple(sorted((buoy_geom, collector_geom)))
    require(expected in pairs, f"{label} did not produce contact {expected}; got {sorted(pairs)}")


def check_no_sticky_runtime() -> None:
    text = RUNTIME.read_text(encoding="utf-8")
    forbidden = (
        "captured",
        "capture_local_offset",
        "_hold_captured_buoy",
        "UUV_COURSE_BUOY_COLLECTOR_CAPTURE_ENABLE",
    )
    found = [needle for needle in forbidden if needle in text]
    require(not found, f"sticky collector runtime code remains: {found}")


def check_collision_flags(mujoco, model) -> None:
    for name in COLLECTOR_COLLISION_GEOMS:
        gid = geom_id(mujoco, model, name)
        require(int(model.geom_contype[gid]) != 0, f"{name} contype must be nonzero")
        require(int(model.geom_conaffinity[gid]) != 0, f"{name} conaffinity must be nonzero")
    marker = geom_id(mujoco, model, "collector_open_mouth_marker")
    require(int(model.geom_contype[marker]) == 0, "open mouth marker must stay visual-only")


def check_red_buoy_pocket_contacts(mujoco, model) -> None:
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    buoy_geom = "course_buoy_a_red_1_float_geom"

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(data, base, np.array([-0.205, 0.0, 0.230], dtype=np.float64)),
    )
    assert_contact(mujoco, model, data, buoy_geom=buoy_geom, collector_geom="collector_back_net_proxy", label="red back wall")

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(data, base, np.array([0.0, -0.205, 0.230], dtype=np.float64)),
    )
    assert_contact(mujoco, model, data, buoy_geom=buoy_geom, collector_geom="collector_left_net_proxy", label="red side wall")


def check_red_buoy_moves_by_contact(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    target_z = runtime._surface_target_center_z(runtime_buoy)
    back_geom = geom_id(mujoco, model, "collector_back_net_proxy")
    data.qpos[base_qposadr + 2] += target_z - float(data.geom_xpos[back_geom, 2])
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    start_base = np.array(data.xpos[base], dtype=np.float64)

    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(
            data,
            base,
            np.array([-0.135, 0.0, target_z - float(data.xpos[base, 2])], dtype=np.float64),
        ),
    )
    start_buoy = np.array(data.xipos[buoy], dtype=np.float64)

    for _ in range(800):
        data.qvel[base_dofadr + 0] = 0.35
        data.qvel[base_dofadr + 1] = 0.0
        data.qvel[base_dofadr + 2] = 0.0
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        # Keep this isolated smoke focused on collector contact, not vehicle hydrostatics.
        data.qpos[base_qposadr + 1] = start_base[1]
        data.qpos[base_qposadr + 2] = start_base[2]
        data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)

    base_delta_x = float(data.xpos[base, 0] - start_base[0])
    buoy_delta_x = float(data.xipos[buoy, 0] - start_buoy[0])
    require(base_delta_x > 0.45, f"base did not move enough for contact carry check: {base_delta_x:.3f}m")
    require(buoy_delta_x > 0.40, f"red buoy was not pushed by collector contact: {buoy_delta_x:.3f}m")
    require(abs(buoy_delta_x - base_delta_x) < 0.08, f"red buoy motion is not contact-coupled: base={base_delta_x:.3f} buoy={buoy_delta_x:.3f}")


def check_red_buoy_descends_with_collector_roof(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    dofadr = int(model.jnt_dofadr[joint])
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    target_z = runtime._surface_target_center_z(runtime_buoy)
    roof_geom = geom_id(mujoco, model, "collector_top_net_proxy")
    desired_roof_world_z = target_z + 0.10
    data.qpos[base_qposadr + 2] += desired_roof_world_z - float(data.geom_xpos[roof_geom, 2])
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    start_base = np.array(data.xpos[base], dtype=np.float64)

    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(
            data,
            base,
            np.array([0.0, 0.0, target_z - float(data.xpos[base, 2])], dtype=np.float64),
        ),
    )
    start_buoy = np.array(data.xipos[buoy], dtype=np.float64)

    top_contact_samples = 0
    expected_top = tuple(sorted(("course_buoy_a_red_1_float_geom", "collector_top_net_proxy")))
    for _ in range(1800):
        data.qvel[base_dofadr + 0] = 0.0
        data.qvel[base_dofadr + 1] = 0.0
        data.qvel[base_dofadr + 2] = -0.18
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        if expected_top in contact_names(mujoco, model, data):
            top_contact_samples += 1
        data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        mujoco.mj_forward(model, data)

    base_delta_z = float(data.xpos[base, 2] - start_base[2])
    buoy_delta_z = float(data.xipos[buoy, 2] - start_buoy[2])
    buoy_local = base_local_from_world(data, base, np.array(data.xipos[buoy], dtype=np.float64))
    require(base_delta_z < -0.25, f"base did not descend enough for roof carry check: dz={base_delta_z:.3f}m")
    require(top_contact_samples > 20, f"collector roof did not contact red buoy during descent: samples={top_contact_samples}")
    require(buoy_delta_z < -0.18, f"red buoy stayed pinned to the waterline during collector descent: dz={buoy_delta_z:.3f}m")
    require(float(data.qvel[dofadr + 2]) < 0.20, f"red buoy is bouncing upward out of the collector: vz={float(data.qvel[dofadr + 2]):.3f}m/s")
    require(abs(float(buoy_local[0])) < 0.17, f"red buoy escaped collector in x: local_x={float(buoy_local[0]):.3f}m")
    require(abs(float(buoy_local[1])) < 0.17, f"red buoy escaped collector in y: local_y={float(buoy_local[1]):.3f}m")
    require(0.08 < float(buoy_local[2]) < 0.46, f"red buoy escaped collector vertically: local_z={float(buoy_local[2]):.3f}m")


def check_magnet_release_buoyancy(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_yellow_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_yellow_1_free")
    equality = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "course_buoy_a_yellow_1_magnet_weld"))
    require(equality >= 0, "course_buoy_a_yellow_1 magnet weld equality not found")

    runtime = make_runtime(mujoco, model, data)
    dofadr = int(model.jnt_dofadr[joint])
    for _ in range(20):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
    require(int(data.eq_active[equality]) == 1, "yellow buoy magnet detached without an external/contact break force")

    data.xfrc_applied[buoy, 2] += 16.0
    runtime.apply(float(model.opt.timestep))
    require(int(data.eq_active[equality]) == 0, "yellow buoy magnet did not detach under a 16N upward force")
    data.xfrc_applied[buoy, 2] -= 16.0

    for _ in range(30000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    final_z = float(data.xipos[buoy, 2])
    final_vz = float(data.qvel[dofadr + 2])
    require(final_z > 0.045, f"released yellow buoy did not float to the waterline: final_z={final_z:.3f}")
    require(final_vz > -0.30, f"released yellow buoy is still sinking at the waterline: vz={final_vz:.3f}")


def check_full_immersion_net_lift_contract(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)

    for name in ("course_buoy_a_red_1", "course_buoy_a_yellow_1", "course_buoy_pinger_white_1"):
        buoy = next(item for item in runtime.buoys if item.name == name)
        weight_n = runtime._body_weight_n(buoy)
        upthrust_n = runtime._full_immersion_upthrust_n(buoy)
        net_lift_n = upthrust_n - weight_n
        require(
            abs(net_lift_n - runtime.buoyancy_n) < 1e-9,
            f"{name} full-immersion net lift must be {runtime.buoyancy_n:.3f}N, got {net_lift_n:.3f}N",
        )
        require(
            runtime._surface_target_center_z(buoy) > 0.05,
            f"{name} must settle visibly above the water surface, target_z={runtime._surface_target_center_z(buoy):.3f}",
        )


def check_surface_buoy_moves_under_force(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    qposadr = int(model.jnt_qposadr[joint])
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)

    data.qpos[qposadr + 2] = target_z
    data.qvel[dofadr : dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    start = np.array(data.xipos[buoy], dtype=np.float64)
    for _ in range(400):
        data.xfrc_applied[buoy, :] = 0.0
        data.xfrc_applied[buoy, 0] = 0.18
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    delta = np.array(data.xipos[buoy], dtype=np.float64) - start
    require(delta[0] > 0.08, f"red buoy did not move freely under horizontal force: dx={delta[0]:.3f}m")
    require(abs(float(data.qpos[qposadr + 2]) - target_z) < 0.20, "red buoy left the surface band while moving horizontally")


def check_scene_red_buoy_floats_without_input(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)
    start_z = float(data.xipos[buoy, 2])

    z_values: list[float] = []
    for _ in range(10000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        z_values.append(float(data.xipos[buoy, 2]))

    final_z = float(data.xipos[buoy, 2])
    final_vz = float(data.qvel[dofadr + 2])
    require(abs(start_z - target_z) < 0.015, f"scene red buoy must start at surface target: start={start_z:.3f} target={target_z:.3f}")
    require(abs(final_z - target_z) < 0.08, f"scene red buoy did not stay near the surface band: final={final_z:.3f} target={target_z:.3f}")
    require(final_vz > -0.30, f"scene red buoy is sinking through the surface band: vz={final_vz:.3f}")
    require(max(z_values) < target_z + 0.06, f"scene red buoy overshot unrealistically above surface: max_z={max(z_values):.3f}")


def check_surface_buoy_bobs_without_depth_lock(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    qposadr = int(model.jnt_qposadr[joint])
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)

    data.qpos[qposadr + 2] = target_z
    data.qvel[dofadr + 2] = 0.90
    mujoco.mj_forward(model, data)
    z_values: list[float] = []
    for _ in range(600):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        z_values.append(float(data.xipos[buoy, 2]))

    require(max(z_values) > target_z + 0.005, f"red buoy depth appears clamped: max_z={max(z_values):.3f}, target={target_z:.3f}")
    require(min(z_values) < target_z + 0.010, f"red buoy did not settle back toward surface: min_z={min(z_values):.3f}, target={target_z:.3f}")


def check_free_buoy_rises_from_subsurface(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    qposadr = int(model.jnt_qposadr[joint])
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)

    data.qpos[qposadr + 2] = target_z - 0.35
    data.qvel[dofadr + 2] = -0.20
    mujoco.mj_forward(model, data)
    start_z = float(data.xipos[buoy, 2])
    for _ in range(2500):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    final_z = float(data.xipos[buoy, 2])
    require(final_z > start_z + 0.20, f"free red buoy did not rise from subsurface: start_z={start_z:.3f} final_z={final_z:.3f}")
    require(final_z > target_z - 0.12, f"free red buoy did not return near surface: final_z={final_z:.3f} target_z={target_z:.3f}")


def check_released_buoy_recovers_from_downward_impulse(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_yellow_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_yellow_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_yellow_1")
    qposadr = int(model.jnt_qposadr[joint])
    dofadr = int(model.jnt_dofadr[joint])

    data.xfrc_applied[buoy, 2] += 16.0
    runtime.apply(float(model.opt.timestep))
    data.xfrc_applied[buoy, 2] -= 16.0
    require(runtime_buoy.detached, "yellow buoy did not detach for recovery check")

    data.qpos[qposadr + 2] = -2.0
    data.qvel[dofadr + 2] = -3.0
    mujoco.mj_forward(model, data)
    for _ in range(3000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    final_z = float(data.xipos[buoy, 2])
    final_vz = float(data.qvel[dofadr + 2])
    require(final_z > -0.20, f"released yellow buoy did not recover upward after impulse: final_z={final_z:.3f}")
    require(final_vz >= -0.30, f"released yellow buoy is still sinking too fast after impulse: vz={final_vz:.3f}")


def check_free_buoy_pocket_contacts(mujoco, model) -> None:
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_yellow_1_float")
    buoy_geom = "course_buoy_a_yellow_1_float_geom"

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_yellow_1",
        world_from_base_local(data, base, np.array([0.0, 0.0, 0.153], dtype=np.float64)),
    )
    assert_contact(mujoco, model, data, buoy_geom=buoy_geom, collector_geom="collector_bottom_net_proxy", label="yellow bottom")

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_yellow_1",
        world_from_base_local(data, base, np.array([0.0, 0.0, 0.372], dtype=np.float64)),
    )
    assert_contact(mujoco, model, data, buoy_geom=buoy_geom, collector_geom="collector_top_net_proxy", label="yellow top")


def main() -> int:
    import mujoco

    check_no_sticky_runtime()
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    check_collision_flags(mujoco, model)
    check_red_buoy_pocket_contacts(mujoco, model)
    check_red_buoy_moves_by_contact(mujoco, model)
    check_red_buoy_descends_with_collector_roof(mujoco, model)
    check_full_immersion_net_lift_contract(mujoco, model)
    check_magnet_release_buoyancy(mujoco, model)
    check_surface_buoy_moves_under_force(mujoco, model)
    check_scene_red_buoy_floats_without_input(mujoco, model)
    check_surface_buoy_bobs_without_depth_lock(mujoco, model)
    check_free_buoy_rises_from_subsurface(mujoco, model)
    check_released_buoy_recovers_from_downward_impulse(mujoco, model)
    check_free_buoy_pocket_contacts(mujoco, model)
    print("buoy collector physical pocket: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
