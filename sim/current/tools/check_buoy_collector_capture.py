#!/usr/bin/env python3
"""Smoke-check that the front collector is a physical pocket, not a sticky latch."""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
RUNTIME = ROOT / "sim" / "runtime" / "course_buoy_runtime.py"
PRODUCTION_COLLECTOR_TIMESTEP_S = 0.005
sys.path.insert(0, str(ROOT))

from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402


COLLECTOR_COLLISION_GEOMS = (
    "collector_bottom_rail_left",
    "collector_bottom_rail_right",
    "collector_bottom_cross_back",
    "collector_bottom_cross_front",
    "collector_back_top_cross",
    "collector_front_top_cross",
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
    "collector_front_net_flap",
    "collector_top_rail_left",
    "collector_top_rail_right",
    "collector_rear_mount_left",
    "collector_rear_mount_right",
)

COLLECTOR_NETTED_ONLY_GEOMS = (
    "collector_front_net_flap",
)

COLLECTOR_RETENTION_GEOMS = (
    "collector_top_net_proxy",
    "collector_front_net_flap",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def require_netted_gate_state(model, buoy, *, enabled: bool, label: str) -> None:
    """Verify category 4 is applied only to physical buoy collision geoms."""

    gate_category = 4
    for geom_id, original_contype, original_conaffinity in buoy.geom_collision_bits:
        base_conaffinity = int(original_conaffinity) & ~gate_category
        is_physical = int(original_contype) != 0 or base_conaffinity != 0
        expected_conaffinity = (
            base_conaffinity | gate_category if enabled and is_physical else base_conaffinity
        )
        require(
            int(model.geom_contype[int(geom_id)]) == int(original_contype)
            and int(model.geom_conaffinity[int(geom_id)]) == expected_conaffinity,
            f"{label}: geom {int(geom_id)} collision bits are "
            f"({int(model.geom_contype[int(geom_id)])}, "
            f"{int(model.geom_conaffinity[int(geom_id)])}), expected "
            f"({int(original_contype)}, {expected_conaffinity})",
        )


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


def set_base_local_linear_velocity(data, base: int, dofadr: int, velocity_local: np.ndarray) -> None:
    rot = np.array(data.xmat[base], dtype=np.float64).reshape(3, 3)
    data.qvel[dofadr : dofadr + 3] = rot @ np.asarray(velocity_local, dtype=np.float64)


def body_tilt_deg(data, body: int) -> float:
    rot = np.array(data.xmat[body], dtype=np.float64).reshape(3, 3)
    z_axis = rot[:, 2]
    return float(np.degrees(np.arccos(np.clip(z_axis[2] / np.linalg.norm(z_axis), -1.0, 1.0))))


def place_slide_buoy(mujoco, model, data, body: int, prefix: str, target_world: np.ndarray) -> None:
    mujoco.mj_forward(model, data)
    current = np.array(data.xpos[body], dtype=np.float64)
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
    data.qpos[qposadr + 3 : qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[dofadr : dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    current = buoy_center_world(mujoco, model, data, body, prefix)
    data.qpos[qposadr : qposadr + 3] += target_world - current
    mujoco.mj_forward(model, data)


def buoy_center_world(mujoco, model, data, body: int, prefix: str) -> np.ndarray:
    cob_site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}_cob_site"))
    if cob_site >= 0:
        return np.asarray(data.site_xpos[cob_site], dtype=np.float64).copy()
    return np.asarray(data.xpos[body], dtype=np.float64).copy()


def set_buoy_center_z(mujoco, model, data, body: int, prefix: str, target_z: float) -> None:
    jid = joint_id(mujoco, model, f"{prefix}_free")
    qposadr = int(model.jnt_qposadr[jid])
    mujoco.mj_forward(model, data)
    current_z = float(buoy_center_world(mujoco, model, data, body, prefix)[2])
    data.qpos[qposadr + 2] += float(target_z) - current_z
    mujoco.mj_forward(model, data)


def env_float(_name: str, default: float) -> float:
    return default


def env_flag(_name: str, default: bool) -> bool:
    if _name == "UUV_COURSE_BUOY_TRACK_CSV_ENABLE":
        return False
    return default


def make_runtime_from_env(mujoco, model, data, *, env_float_fn=env_float, env_flag_fn=env_flag) -> CourseBuoyRuntime:
    return CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=0.0,
        env_float=env_float_fn,
        env_flag=env_flag_fn,
        log=lambda _message: None,
    )


def make_runtime(mujoco, model, data) -> CourseBuoyRuntime:
    return make_runtime_from_env(mujoco, model, data)


def make_low_profile_runtime(mujoco, model, data) -> CourseBuoyRuntime:
    values = {
        "UUV_COURSE_BUOY_UPDATE_HZ": 30.0,
        "UUV_COURSE_BUOY_TRACK_CSV_INTERVAL_S": 3.0,
    }

    def low_profile_env_float(name: str, default: float) -> float:
        return float(values.get(name, default))

    def low_profile_env_flag(name: str, default: bool) -> bool:
        if name == "UUV_COURSE_BUOY_TRACK_CSV_ENABLE":
            return False
        return default

    return make_runtime_from_env(
        mujoco,
        model,
        data,
        env_float_fn=low_profile_env_float,
        env_flag_fn=low_profile_env_flag,
    )


def runtime_detection_steps(runtime: CourseBuoyRuntime, model, minimum_s: float = 0.020) -> int:
    dt = float(model.opt.timestep)
    wait_s = max(float(minimum_s), float(runtime.update_period_s))
    return max(2, int(np.ceil(wait_s / dt)) + 2)


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


def pair_min_contact_distance(mujoco, model, data, expected_pair: tuple[str, str]) -> float | None:
    """Return real separation/penetration for a pair, ignoring contact margins."""

    minimum: float | None = None
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
        if tuple(sorted((name1, name2))) != expected_pair:
            continue
        distance = float(contact.dist)
        minimum = distance if minimum is None else min(minimum, distance)
    return minimum


def contact_details(mujoco, model, data) -> list[tuple[tuple[str, str], tuple[str, str]]]:
    details: list[tuple[tuple[str, str], tuple[str, str]]] = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        pair = []
        for geom in (int(contact.geom1), int(contact.geom2)):
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or str(geom)
            contact_body_id = int(model.geom_bodyid[geom])
            body_name = (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, contact_body_id)
                or str(contact_body_id)
            )
            pair.append((geom_name, body_name))
        details.append((pair[0], pair[1]))
    return details


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


def check_soft_net_runtime_contract() -> None:
    text = RUNTIME.read_text(encoding="utf-8")
    forbidden = (
        "buoy.captured",
        "captured: bool",
        "capture_local_offset",
        "_hold_captured_buoy",
        "UUV_COURSE_BUOY_COLLECTOR_CAPTURE_ENABLE",
    )
    found = [needle for needle in forbidden if needle in text]
    require(not found, f"old sticky collector runtime code remains: {found}")
    require("collector_net_enable" in text, "collector soft-net runtime is missing")
    require("netted" in text, "collector soft-net state is missing")
    require("_collector_net_ready_for_capture" in text, "collector net must gate capture by actual surface proximity")
    require("_activate_collector_weld" in text, "collector soft weld activation is missing")
    require("_deactivate_collector_weld" in text, "collector score-zone weld release is missing")
    require("_collector_reverse_release_requested" in text, "collector reverse exit release is missing")
    require("data.ncon = 0" not in text, "runtime must not globally erase MuJoCo contacts")


def check_collision_flags(mujoco, model) -> None:
    collector_body = body_id(mujoco, model, "front_open_buoy_collector")
    base = body_id(mujoco, model, "base_link")
    require(int(model.body_parentid[collector_body]) == base, "collector must be a direct base_link child")
    require(int(model.body_dofnum[collector_body]) == 0, "collector net body must be rigid, not free or articulated")
    for name in COLLECTOR_COLLISION_GEOMS:
        gid = geom_id(mujoco, model, name)
        require(int(model.geom_bodyid[gid]) == collector_body, f"{name} is not attached to the rigid collector body")
        require(int(model.geom_contype[gid]) != 0, f"{name} contype must be nonzero")
        require(int(model.geom_conaffinity[gid]) != 0, f"{name} conaffinity must be nonzero")
    for name in ("collector_left_net_proxy", "collector_right_net_proxy"):
        gid = geom_id(mujoco, model, name)
        require(
            int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_MESH),
            f"{name} must use the non-overlapping trapezoidal mesh panel",
        )
        mesh_name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_MESH,
            int(model.geom_dataid[gid]),
        )
        require(
            mesh_name == "front_open_buoy_collector_side_proxy_v1",
            f"{name} uses the wrong trapezoidal side mesh: {mesh_name}",
        )
    back_gid = geom_id(mujoco, model, "collector_back_net_proxy")
    require(
        int(model.geom_type[back_gid]) == int(mujoco.mjtGeom.mjGEOM_BOX),
        "collector_back_net_proxy must remain a rigid box collision panel",
    )
    marker = geom_id(mujoco, model, "collector_open_mouth_marker")
    require(int(model.geom_contype[marker]) == 0, "open mouth marker must stay visual-only")
    roof = geom_id(mujoco, model, "collector_top_net_proxy")
    require(int(model.geom_contype[roof]) == 1, "collector roof must remain permanently physical")
    require(int(model.geom_conaffinity[roof]) & 1, "collector roof must collide with physical buoy geoms")
    for name in COLLECTOR_NETTED_ONLY_GEOMS:
        gid = geom_id(mujoco, model, name)
        require(int(model.geom_contype[gid]) == 4, f"{name} must use the netted-only collision category")
        require(
            not (int(model.geom_conaffinity[gid]) & 2),
            f"{name} must not receive buoy contacts",
        )
    rod = geom_id(mujoco, model, "course_buoy_a_yellow_1_mooring_rod")
    require(int(model.geom_contype[rod]) == 0, "fixed mooring rod must not create vehicle contacts")
    require(int(model.geom_conaffinity[rod]) == 0, "fixed mooring rod must be visual-only")
    for course in ("a", "b"):
        for number in range(1, 6):
            body = body_id(mujoco, model, f"course_buoy_{course}_red_{number}_float")
            gid = geom_id(mujoco, model, f"course_buoy_{course}_red_{number}_float_geom")
            require(
                np.allclose(
                    np.asarray(model.body_inertia[body], dtype=np.float64),
                    np.array([0.000025, 0.000025, 0.000020]),
                    rtol=0.0,
                    atol=1.0e-9,
                ),
                f"course_buoy_{course}_red_{number} does not use the 10 g baseline inertia",
            )
            require(float(model.geom_solref[gid, 0]) >= 0.05, "red buoy contact response is too stiff")
            require(int(model.geom_condim[gid]) == 3, "red buoy must avoid unstable torsional contact friction")
    for gid in range(int(model.ngeom)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if not (name.startswith("course_buoy_") and name.endswith("_float_geom")):
            continue
        require(float(model.geom_solref[gid, 0]) >= 0.05, f"{name} contact response is too stiff")
        require(int(model.geom_condim[gid]) == 3, f"{name} must avoid unstable torsional contact friction")
        body_name = name.removesuffix("_geom")
        prefix = body_name.removesuffix("_float")
        collector_eq = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_EQUALITY,
            f"{prefix}_collector_weld",
        )
        require(collector_eq >= 0, f"{prefix} is missing its soft collector weld")
        require(int(model.eq_active0[collector_eq]) == 0, f"{prefix} collector weld must start inactive")
        require(float(model.eq_solref[collector_eq, 0]) >= 0.05, f"{prefix} collector weld is too stiff")
        body = body_id(mujoco, model, body_name)
        dof = int(model.body_dofadr[body])
        require(dof >= 0, f"{body_name} must have a free joint")
        require(
            np.all(model.dof_damping[dof : dof + 6] >= 0.019),
            f"{body_name} needs per-step hydrodynamic contact damping",
        )


def check_rigid_mooring_rods(mujoco, model) -> None:
    rod_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id_) or ""
        for geom_id_ in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id_) or "").endswith("_mooring_rod")
    ]
    require(len(rod_names) == 15, f"expected 15 fixed mooring rods, got {len(rod_names)}")
    require(
        not any("_flex_line_" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or "") for body in range(model.nbody)),
        "legacy flexible cable bodies remain in the compiled model",
    )
    require(
        not any("_flex_line_" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, eq) or "") for eq in range(model.neq)),
        "legacy flexible cable equalities remain in the compiled model",
    )


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
        world_from_base_local(data, base, np.array([-0.265, 0.0, 0.230], dtype=np.float64)),
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
        world_from_base_local(data, base, np.array([0.0, -0.265, 0.230], dtype=np.float64)),
    )
    assert_contact(mujoco, model, data, buoy_geom=buoy_geom, collector_geom="collector_left_net_proxy", label="red side wall")


def check_red_buoy_moves_by_contact(mujoco, model, *, runtime_factory=make_runtime) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    runtime = runtime_factory(mujoco, model, data)
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
            np.array([-0.195, 0.0, target_z - float(data.xpos[base, 2])], dtype=np.float64),
        ),
    )
    start_buoy = buoy_center_world(mujoco, model, data, buoy, "course_buoy_a_red_1")

    contact_steps = max(1, int(round(1.60 / float(model.opt.timestep))))
    for _ in range(contact_steps):
        data.qvel[base_dofadr + 0] = 0.35
        data.qvel[base_dofadr + 1] = 0.0
        data.qvel[base_dofadr + 2] = 0.0
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "collector contact produced non-finite position")
        require(np.all(np.isfinite(data.qvel)), "collector contact produced non-finite velocity")
        require(np.all(np.isfinite(data.qacc)), "collector contact produced non-finite acceleration")
        # Keep this isolated smoke focused on collector contact, not vehicle hydrostatics.
        data.qpos[base_qposadr + 1] = start_base[1]
        data.qpos[base_qposadr + 2] = start_base[2]
        data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)

    base_delta_x = float(data.xpos[base, 0] - start_base[0])
    buoy_delta_x = float(buoy_center_world(mujoco, model, data, buoy, "course_buoy_a_red_1")[0] - start_buoy[0])
    require(base_delta_x > 0.45, f"base did not move enough for contact carry check: {base_delta_x:.3f}m")
    require(buoy_delta_x > 0.40, f"red buoy was not pushed by collector contact: {buoy_delta_x:.3f}m")
    require(abs(buoy_delta_x - base_delta_x) < 0.08, f"red buoy motion is not contact-coupled: base={base_delta_x:.3f} buoy={buoy_delta_x:.3f}")


def check_red_buoy_stays_surface_when_collector_descends(mujoco, model, *, runtime_factory=make_runtime) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    runtime = runtime_factory(mujoco, model, data)
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
    start_buoy = buoy_center_world(mujoco, model, data, buoy, "course_buoy_a_red_1")

    expected_top = tuple(sorted(("course_buoy_a_red_1_float_geom", "collector_top_net_proxy")))
    descent_steps = max(1, int(round(3.60 / float(model.opt.timestep))))
    for _ in range(descent_steps):
        data.qvel[base_dofadr + 0] = 0.0
        data.qvel[base_dofadr + 1] = 0.0
        data.qvel[base_dofadr + 2] = -0.18
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "descending collector produced non-finite position")
        require(np.all(np.isfinite(data.qvel)), "descending collector produced non-finite velocity")
        require(np.all(np.isfinite(data.qacc)), "descending collector produced non-finite acceleration")
        # MuJoCo emits a contact record while two geoms are still separated
        # but inside their soft margin.  Treat only physical penetration as a
        # failure; the final displacement check below detects actual pushing.
        top_distance = pair_min_contact_distance(mujoco, model, data, expected_top)
        require(
            top_distance is None or top_distance >= -0.010,
            f"collector top net penetrated the red buoy: dist={top_distance}",
        )
        data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        mujoco.mj_forward(model, data)

    base_delta_z = float(data.xpos[base, 2] - start_base[2])
    final_buoy = buoy_center_world(mujoco, model, data, buoy, "course_buoy_a_red_1")
    buoy_delta_z = float(final_buoy[2] - start_buoy[2])
    buoy_local = base_local_from_world(data, base, final_buoy)
    require(base_delta_z < -0.25, f"base did not descend enough for roof clearance check: dz={base_delta_z:.3f}m")
    require(abs(buoy_delta_z) < 0.06, f"red buoy was pushed down by collector top: dz={buoy_delta_z:.3f}m")
    require(float(buoy_local[2]) > 0.45, f"red buoy should remain above descending collector: local_z={float(buoy_local[2]):.3f}m")


def check_collector_net_surface_gate(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    require(runtime.status_by_name()[runtime_buoy.name]["capture_state"] == "FREE", "initial collector state is not FREE")
    target_z = runtime._surface_target_center_z(runtime_buoy)
    hold = np.asarray(runtime.collector_net_hold_local, dtype=np.float64)
    capture_x = float(hold[0]) + 0.65 * float(runtime.collector_net_window_x_m)
    capture_y = float(hold[1]) + 0.80 * float(runtime.collector_net_window_y_m)

    data.qpos[base_qposadr : base_qposadr + 3] = [float(data.xpos[base, 0]), float(data.xpos[base, 1]), -8.8]
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(data, base, np.array([capture_x, capture_y, target_z - float(data.xpos[base, 2])], dtype=np.float64)),
    )
    runtime.apply(float(model.opt.timestep))
    require(not runtime_buoy.netting, "surface red buoy started net entry while collector was underwater")
    require(not runtime_buoy.netted, "surface red buoy was netted while the collector was underwater")

    data.qpos[base_qposadr + 2] = target_z - hold[2]
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(data, base, np.array([capture_x, capture_y, target_z - float(data.xpos[base, 2])], dtype=np.float64)),
    )
    for _ in range(runtime_detection_steps(runtime, model)):
        runtime.apply(float(model.opt.timestep))
        if runtime_buoy.netting:
            break
        mujoco.mj_step(model, data)
    require(runtime_buoy.netting, "surface red buoy did not begin physical net entry at the mouth")
    require(not runtime_buoy.netted, "surface red buoy was counted before reaching an interior slot")
    require(runtime.status_by_name()[runtime_buoy.name]["capture_state"] == "NETTING", "collector skipped NETTING")
    require_netted_gate_state(
        model,
        runtime_buoy,
        enabled=False,
        label="net flap closed before the buoy reached an interior slot",
    )

    held_base_pose = np.array(data.qpos[base_qposadr : base_qposadr + 7], dtype=np.float64)
    entry_steps = max(1, int(round(4.0 / float(model.opt.timestep))))
    activation_position_jump = -1.0
    activation_velocity_jump = -1.0
    for _ in range(entry_steps):
        data.qpos[base_qposadr : base_qposadr + 7] = held_base_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        before_position = runtime._buoy_center_world(runtime_buoy)
        before_velocity = np.asarray(
            data.qvel[runtime_buoy.free_dofadr : runtime_buoy.free_dofadr + 6], dtype=np.float64
        ).copy()
        was_netted = runtime_buoy.netted
        runtime.apply(float(model.opt.timestep))
        if runtime_buoy.netted and not was_netted:
            activation_position_jump = float(
                np.linalg.norm(runtime._buoy_center_world(runtime_buoy) - before_position)
            )
            activation_velocity_jump = float(
                np.linalg.norm(
                    data.qvel[runtime_buoy.free_dofadr : runtime_buoy.free_dofadr + 6]
                    - before_velocity
                )
            )
        mujoco.mj_step(model, data)
        if runtime_buoy.netted:
            break
    require(runtime_buoy.netted, "surface red buoy did not settle into an interior slot")
    require(not runtime_buoy.netting, "surface red buoy stayed in NETTING after physical settlement")
    require(runtime.status_by_name()[runtime_buoy.name]["capture_state"] == "NETTED", "collector skipped NETTED")
    require(activation_position_jump <= 1.0e-12, f"weld activation changed qpos by {activation_position_jump:.3e}m")
    require(activation_velocity_jump <= 1.0e-12, f"weld activation changed qvel by {activation_velocity_jump:.3e}m/s")
    require(
        runtime_buoy.collector_eq_id >= 0 and bool(data.eq_active[runtime_buoy.collector_eq_id]),
        "surface red buoy collector weld did not activate after settlement",
    )
    collector_local = np.asarray(model.body_pos[runtime.collector_body_id], dtype=np.float64)
    relpose = np.asarray(model.eq_data[runtime_buoy.collector_eq_id, 3:6], dtype=np.float64)
    cob_local = np.asarray(model.site_pos[runtime_buoy.cob_site_id], dtype=np.float64)
    require(
        np.allclose(collector_local + relpose + cob_local, runtime._collector_net_slot_local(runtime_buoy), atol=1.0e-12),
        "collector weld relpose did not compensate the CoB offset",
    )
    require(not runtime_buoy.collisions_suppressed, "netted buoy must keep physical collisions inside the collector")
    require_netted_gate_state(
        model,
        runtime_buoy,
        enabled=True,
        label="NETTED transition did not close the physical front flap",
    )
    held_base_z = float(data.qpos[base_qposadr + 2])
    carry_steps = max(1, int(round(8.0 / float(model.opt.timestep))))
    for _ in range(carry_steps):
        data.qpos[base_qposadr + 2] = held_base_z
        set_base_local_linear_velocity(data, base, base_dofadr, np.array([0.45, 0.0, 0.0]))
        data.qvel[base_dofadr + 5] = 0.35
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qacc)), "netted buoy caused non-finite MuJoCo acceleration")
    buoy_local = base_local_from_world(data, base, runtime._buoy_center_world(runtime_buoy))
    require(
        -0.33 <= float(buoy_local[0]) <= 0.31
        and abs(float(buoy_local[1])) <= 0.32
        and 0.15 <= float(buoy_local[2]) <= 0.62,
        f"netted buoy trailed outside the physical collector: local={buoy_local}",
    )

    slots = []
    for index, item in enumerate(runtime.buoys[:13]):
        item.net_slot_index = index
        slots.append(runtime._collector_net_slot_local(item))
    require(len({tuple(np.round(slot, 4)) for slot in slots}) == 13, "collector must provide unique slots for 13 buoys")


def check_score_release_requires_release_phase_and_target_zone(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    with tempfile.TemporaryDirectory(prefix="uuv_score_gate_") as directory:
        status_path = Path(directory) / "mission_fsm_status.json"
        runtime.collector_net_score_release_status_path = status_path

        status_path.write_text(
            json.dumps(
                {
                    "state": "SCORE_ZONE_TRANSIT",
                    "score_zone": {"xyz": list(runtime.score_zone_a)},
                }
            )
        )
        require(
            not runtime._score_release_phase_allowed_now(1.0),
            "collector released during SCORE_ZONE_TRANSIT",
        )

        target = np.asarray(runtime.score_zone_a, dtype=np.float64) + np.array([0.12, -0.08, 0.0])
        status_path.write_text(
            json.dumps(
                {
                    "state": "SCORE_ZONE_CONFIRM",
                    "score_zone": {"xyz": target.tolist()},
                }
            )
        )
        require(
            not runtime._score_release_phase_allowed_now(1.3),
            "collector released during SCORE_ZONE_CONFIRM",
        )
        require(
            runtime._score_release_zone_target is not None
            and np.allclose(runtime._score_release_zone_target, target),
            "collector did not retain the FSM-selected score zone while release was gated",
        )

        observation_target = target + np.array([0.05, 0.03, 0.0])
        status_path.write_text(
            json.dumps(
                {
                    "state": "RELEASE",
                    "score_zone": {"xyz": observation_target.tolist()},
                }
            )
        )
        require(
            runtime._score_release_phase_allowed_now(1.6),
            "collector did not allow the observation FSM RELEASE phase",
        )
        require(
            runtime._score_release_zone_target is not None
            and np.allclose(runtime._score_release_zone_target, observation_target),
            "collector did not bind RELEASE to the observation FSM score zone",
        )


def capture_buoy_through_physical_mouth(
    mujoco,
    model,
    data,
    runtime,
    base: int,
    base_qposadr: int,
    base_dofadr: int,
    held_base_pose: np.ndarray,
    item,
    *,
    expected_slot: int,
) -> np.ndarray:
    """Drive one buoy through FREE -> NETTING -> NETTED using runtime logic."""

    hold = np.asarray(runtime.collector_net_hold_local, dtype=np.float64)
    capture_x = float(hold[0]) + 0.65 * float(runtime.collector_net_window_x_m)
    if not item.detached:
        runtime._detach(item, reason="collector_test_release", force_n=runtime.break_force_n)
    item.netting = False
    item.netted = False
    item.net_reverse_released = False
    item.net_score_released = False
    item.net_slot_index = -1
    if item.collector_eq_id >= 0:
        data.eq_active[item.collector_eq_id] = 0
    runtime._restore_buoy_collisions(item)
    runtime._set_netted_gate_collision(item, enabled=False)
    item_target_z = runtime._surface_target_center_z(item)
    place_free_buoy(
        mujoco,
        model,
        data,
        item.body_id,
        item.name,
        world_from_base_local(
            data,
            base,
            np.array(
                [capture_x, hold[1], item_target_z - float(data.xpos[base, 2])],
                dtype=np.float64,
            ),
        ),
    )

    for _ in range(runtime_detection_steps(runtime, model)):
        data.qpos[base_qposadr : base_qposadr + 7] = held_base_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        if item.netting:
            break
        mujoco.mj_step(model, data)
    require(item.netting, f"{item.name} did not enter NETTING at the physical mouth")
    require(not item.netted, f"{item.name} was counted before entering the physical net")
    require_netted_gate_state(
        model,
        item,
        enabled=False,
        label=f"{item.name} gate closed during NETTING",
    )

    settle_steps = max(1, int(round(5.0 / float(model.opt.timestep))))
    for _ in range(settle_steps):
        data.qpos[base_qposadr : base_qposadr + 7] = held_base_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qacc)), f"{item.name} net entry produced non-finite acceleration")
        if item.netted:
            break

    require(item.netted and not item.netting, f"{item.name} did not settle before collection confirmation")
    require(
        item.collector_eq_id >= 0 and bool(data.eq_active[item.collector_eq_id]),
        f"{item.name} soft collector weld did not activate",
    )
    require_netted_gate_state(
        model,
        item,
        enabled=True,
        label=f"{item.name} actual NETTED transition did not close the front flap",
    )
    require(item.net_slot_index == expected_slot, f"{item.name} received unexpected slot {item.net_slot_index}")
    local = base_local_from_world(data, base, runtime._buoy_center_world(item))
    slot = runtime._collector_net_slot_local(item)
    require(
        float(np.linalg.norm(local - slot)) <= 0.11,
        f"{item.name} was confirmed outside its physical slot: local={local}, slot={slot}",
    )
    require(
        -0.25 <= float(local[0]) <= 0.23
        and abs(float(local[1])) <= 0.25
        and 0.22 <= float(local[2]) <= 0.55,
        f"{item.name} was confirmed outside the physical net interior: local={local}",
    )
    return local


def check_thirteen_buoys_enter_and_settle_sequentially(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    base = body_id(mujoco, model, "base_link")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    hold = np.asarray(runtime.collector_net_hold_local, dtype=np.float64)
    target_z = runtime._surface_target_center_z(runtime.buoys[0])
    data.qpos[base_qposadr + 2] = target_z - hold[2]
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    held_base_pose = np.array(data.qpos[base_qposadr : base_qposadr + 7], dtype=np.float64)
    for index, item in enumerate(runtime.buoys[:13]):
        capture_buoy_through_physical_mouth(
            mujoco,
            model,
            data,
            runtime,
            base,
            base_qposadr,
            base_dofadr,
            held_base_pose,
            item,
            expected_slot=index,
        )

    held_base_z = float(data.qpos[base_qposadr + 2])
    carry_steps = max(1, int(round(12.0 / float(model.opt.timestep))))
    carry_start_time = float(data.time)
    for step in range(carry_steps):
        data.qpos[base_qposadr + 2] = held_base_z
        set_base_local_linear_velocity(data, base, base_dofadr, np.array([0.30, 0.0, 0.0]))
        data.qvel[base_dofadr + 5] = 0.10
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qacc)), "sequentially loaded net produced non-finite acceleration")
        expected_time = carry_start_time + (step + 1) * float(model.opt.timestep)
        require(float(data.time) >= expected_time - 0.5 * float(model.opt.timestep), "sequentially loaded net reset MuJoCo time")

    for item in runtime.buoys[:13]:
        local = base_local_from_world(data, base, runtime._buoy_center_world(item))
        require(item.netted, f"{item.name} lost its collection state during physical carry")
        require(
            -0.33 <= float(local[0]) <= 0.31
            and abs(float(local[1])) <= 0.32
            and 0.15 <= float(local[2]) <= 0.62,
            f"{item.name} trailed outside the physical net after sequential loading: local={local}",
        )


def check_thirteen_buoys_stay_inside_physical_net(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    base = body_id(mujoco, model, "base_link")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    target_z = runtime._surface_target_center_z(runtime.buoys[0])
    data.qpos[base_qposadr + 2] = target_z - runtime.collector_net_hold_local[2]
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    held_base_pose = np.asarray(data.qpos[base_qposadr : base_qposadr + 7], dtype=np.float64).copy()

    carried = runtime.buoys[:13]
    slots = []
    for index, item in enumerate(carried):
        capture_buoy_through_physical_mouth(
            mujoco,
            model,
            data,
            runtime,
            base,
            base_qposadr,
            base_dofadr,
            held_base_pose,
            item,
            expected_slot=index,
        )
        slots.append(runtime._collector_net_slot_local(item))

    require(len({tuple(np.round(slot, 4)) for slot in slots}) == 13, "physical net slots must be unique")
    for left in range(len(slots)):
        for right in range(left + 1, len(slots)):
            delta = np.abs(slots[left] - slots[right])
            separated = delta[0] >= 0.11 or delta[1] >= 0.11 or delta[2] >= 0.17
            require(separated, f"physical net slots overlap: {left} and {right}")

    mujoco.mj_forward(model, data)
    carried_geom_ids = {gid for item in carried for gid in item.geom_ids}
    collector_geom_ids = {geom_id(mujoco, model, name) for name in COLLECTOR_COLLISION_GEOMS}
    collector_gate_geom_ids = {
        geom_id(mujoco, model, name) for name in COLLECTOR_RETENTION_GEOMS
    }
    base_geom_ids = {
        gid
        for gid in range(int(model.ngeom))
        if int(model.geom_bodyid[gid]) == base
        and (int(model.geom_contype[gid]) != 0 or int(model.geom_conaffinity[gid]) != 0)
    }
    penetrations = []
    gate_penetrations = []
    base_penetrations = []
    for contact_id in range(int(data.ncon)):
        contact = data.contact[contact_id]
        pair = {int(contact.geom1), int(contact.geom2)}
        if pair & carried_geom_ids and pair & collector_geom_ids and float(contact.dist) < -0.002:
            penetrations.append(float(contact.dist))
        if pair & carried_geom_ids and pair & collector_gate_geom_ids and float(contact.dist) < -0.002:
            gate_penetrations.append(float(contact.dist))
        if pair & carried_geom_ids and pair & base_geom_ids and float(contact.dist) < -0.002:
            base_penetrations.append(float(contact.dist))
    require(not penetrations, f"buoy slots initially penetrate collector geoms: {penetrations[:8]}")
    require(not gate_penetrations, f"buoy slots initially penetrate roof/front gate: {gate_penetrations[:8]}")
    require(not base_penetrations, f"buoy slots initially penetrate base_link geoms: {base_penetrations[:8]}")

    held_base_z = float(data.qpos[base_qposadr + 2])
    carry_steps = max(1, int(round(12.0 / float(model.opt.timestep))))
    carry_start_time = float(data.time)
    worst_base_penetration = 0.0
    worst_gate_penetration = 0.0
    max_local_x = {item.name: -float("inf") for item in carried}
    max_local_z = {item.name: -float("inf") for item in carried}
    for step in range(carry_steps):
        data.qpos[base_qposadr + 2] = held_base_z
        set_base_local_linear_velocity(data, base, base_dofadr, np.array([0.30, 0.0, 0.0]))
        data.qvel[base_dofadr + 5] = 0.10
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qacc)), "13-buoy physical net produced non-finite acceleration")
        for contact_id in range(int(data.ncon)):
            contact = data.contact[contact_id]
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair & carried_geom_ids and pair & base_geom_ids:
                worst_base_penetration = min(worst_base_penetration, float(contact.dist))
            if pair & carried_geom_ids and pair & collector_gate_geom_ids:
                worst_gate_penetration = min(worst_gate_penetration, float(contact.dist))
        for item in carried:
            local = base_local_from_world(data, base, runtime._buoy_center_world(item))
            max_local_x[item.name] = max(max_local_x[item.name], float(local[0]))
            max_local_z[item.name] = max(max_local_z[item.name], float(local[2]))
        expected_time = carry_start_time + (step + 1) * float(model.opt.timestep)
        require(float(data.time) >= expected_time - 0.5 * float(model.opt.timestep), "13-buoy physical net reset MuJoCo time")

    require(
        worst_base_penetration >= -0.002,
        f"carried buoy penetrated the robot body: depth={worst_base_penetration:.4f}m",
    )
    require(
        worst_gate_penetration >= -0.002,
        f"carried buoy penetrated the collector roof/front gate: depth={worst_gate_penetration:.4f}m",
    )

    for item in carried:
        local = base_local_from_world(data, base, runtime._buoy_center_world(item))
        require(
            -0.33 <= float(local[0]) <= 0.275
            and abs(float(local[1])) <= 0.32
            and 0.15 <= float(local[2]) <= 0.545,
            f"{item.name} escaped the physical collector envelope: local={local}",
        )
        require(
            max_local_x[item.name] <= 0.275,
            f"{item.name} crossed the closed front flap during carry: max_x={max_local_x[item.name]:.4f}",
        )
        require(
            max_local_z[item.name] <= 0.545,
            f"{item.name} crossed the closed roof during carry: max_z={max_local_z[item.name]:.4f}",
        )
        require(
            item.collector_eq_id >= 0 and bool(data.eq_active[item.collector_eq_id]),
            f"{item.name} did not retain its soft collector weld while carried",
        )
        require_netted_gate_state(
            model,
            item,
            enabled=True,
            label=f"{item.name} lost the NETTED front-flap collision category during carry",
        )


def check_reverse_motion_opens_physical_mouth(mujoco, model) -> None:
    """A deliberate reverse must release the soft hold and let the buoy exit +X."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    runtime.collector_net_score_release_enable = False
    base = body_id(mujoco, model, "base_link")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    item = next(buoy for buoy in runtime.buoys if buoy.name == "course_buoy_a_red_1")

    target_z = runtime._surface_target_center_z(item)
    data.qpos[base_qposadr + 2] = target_z - runtime.collector_net_hold_local[2]
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    held_base_pose = np.asarray(data.qpos[base_qposadr : base_qposadr + 7], dtype=np.float64).copy()
    capture_buoy_through_physical_mouth(
        mujoco,
        model,
        data,
        runtime,
        base,
        base_qposadr,
        base_dofadr,
        held_base_pose,
        item,
        expected_slot=0,
    )
    # Reverse exit happens after collection, well after the short magnet
    # release stabilization window. Keep that unrelated limiter out of the
    # equality-deactivation jump measurement.
    item.release_time_s = -10.0

    # Forward and turning motion must retain the captured state.
    for _ in range(max(1, int(round(0.25 / float(model.opt.timestep))))):
        set_base_local_linear_velocity(data, base, base_dofadr, np.array([0.20, 0.0, -0.04]))
        data.qvel[base_dofadr + 5] = 0.08
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(item.netted, "forward/turn/depth motion unexpectedly released a captured buoy")
        require(bool(data.eq_active[item.collector_eq_id]), "collector weld dropped during normal carry")
        require_netted_gate_state(
            model,
            item,
            enabled=True,
            label="normal carry opened the NETTED front flap",
        )

    # Establish actual reverse velocity, then verify the equality transition
    # itself does not edit the buoy state.
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    set_base_local_linear_velocity(data, base, base_dofadr, np.array([-0.30, 0.0, 0.0]))
    mujoco.mj_forward(model, data)
    qpos_before = np.asarray(data.qpos[item.free_qposadr : item.free_qposadr + 7]).copy()
    qvel_before = np.asarray(data.qvel[item.free_dofadr : item.free_dofadr + 6]).copy()
    runtime.apply(float(model.opt.timestep))
    require(item.net_reverse_released and not item.netted, "reverse did not open the collector mouth")
    require(not bool(data.eq_active[item.collector_eq_id]), "reverse left the collector weld active")
    require_netted_gate_state(
        model,
        item,
        enabled=False,
        label="reverse release did not reopen the front flap",
    )
    require(
        float(np.max(np.abs(data.qpos[item.free_qposadr : item.free_qposadr + 7] - qpos_before))) <= 1.0e-9,
        "reverse collector release changed buoy qpos",
    )
    require(
        float(np.max(np.abs(data.qvel[item.free_dofadr : item.free_dofadr + 6] - qvel_before))) <= 1.0e-9,
        "reverse collector release changed buoy qvel",
    )
    mujoco.mj_step(model, data)

    max_steps = max(1, int(round(3.0 / float(model.opt.timestep))))
    mouth_clear_x = 0.315 + float(item.release_radius_m) + 0.015
    local = base_local_from_world(data, base, runtime._buoy_center_world(item))
    for _ in range(max_steps):
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        set_base_local_linear_velocity(data, base, base_dofadr, np.array([-0.30, 0.0, 0.0]))
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "reverse net exit produced non-finite qpos")
        require(np.all(np.isfinite(data.qvel)), "reverse net exit produced non-finite qvel")
        local = base_local_from_world(data, base, runtime._buoy_center_world(item))
        if float(local[0]) >= mouth_clear_x:
            break
    require(
        float(local[0]) >= mouth_clear_x,
        f"buoy did not naturally exit the open +X mouth: local={local}, clear_x={mouth_clear_x:.3f}",
    )
    # The clearance is observed after mj_step; let the next physics-rate
    # runtime sample retire the temporary no-recapture latch.
    if item.net_reverse_released:
        runtime.apply(float(model.opt.timestep))
    require(not item.net_reverse_released, "reverse exit re-capture latch did not clear outside the mouth")
    require(not item.netted and not bool(data.eq_active[item.collector_eq_id]), "exited buoy was re-captured")


def check_thirteen_buoys_release_into_selected_score_zone(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    base = body_id(mujoco, model, "base_link")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    target_z = runtime._surface_target_center_z(runtime.buoys[0])
    score_zone = np.asarray(runtime.score_zone_a, dtype=np.float64)
    score_probe = np.asarray(runtime.collector_net_score_probe_local, dtype=np.float64)
    hold = np.asarray(runtime.collector_net_hold_local, dtype=np.float64)
    data.qpos[base_qposadr : base_qposadr + 3] = [
        score_zone[0] - score_probe[0],
        score_zone[1] - score_probe[1],
        target_z - hold[2],
    ]
    data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)

    carried = runtime.buoys[:13]
    for index, item in enumerate(carried):
        item.detached = True
        item.netted = True
        item.netted_time_s = float(data.time)
        item.net_slot_index = index
        for equality in (item.eq_id,):
            if equality >= 0:
                data.eq_active[equality] = 0
        runtime._restore_buoy_collisions(item)
        place_free_buoy(
            mujoco,
            model,
            data,
            item.body_id,
            item.name,
            world_from_base_local(data, base, runtime._collector_net_slot_local(item)),
        )

    with tempfile.TemporaryDirectory(prefix="uuv_score_release_") as directory:
        status_path = Path(directory) / "mission_fsm_status.json"
        status_path.write_text(
            json.dumps(
                {
                    "state": "SCORE_ZONE_TRANSIT",
                    "score_zone": {"xyz": score_zone.tolist()},
                }
            )
        )
        runtime.collector_net_score_release_status_path = status_path
        runtime.apply(float(model.opt.timestep))
        require(
            all(item.collector_eq_id >= 0 and bool(data.eq_active[item.collector_eq_id]) for item in carried),
            "collector weld released before SCORE_RELEASE",
        )
        require(not any(item.net_score_released for item in carried), "buoy released during SCORE_ZONE_TRANSIT")
        for item in carried:
            require_netted_gate_state(
                model,
                item,
                enabled=True,
                label=f"{item.name} front flap opened before SCORE_RELEASE",
            )
        status_path.write_text(
            json.dumps(
                {
                    "state": "RELEASE",
                    "score_zone": {"xyz": score_zone.tolist()},
                }
            )
        )
        release_state_checked = False
        for _ in range(2600):
            data.qpos[base_qposadr : base_qposadr + 3] = [
                score_zone[0] - score_probe[0],
                score_zone[1] - score_probe[1],
                target_z - hold[2],
            ]
            data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[base_dofadr : base_dofadr + 6] = 0.0
            before_release = {
                item.name: (
                    np.asarray(
                        data.qpos[item.free_qposadr : item.free_qposadr + 7],
                        dtype=np.float64,
                    ).copy(),
                    np.asarray(
                        data.qvel[item.free_dofadr : item.free_dofadr + 6],
                        dtype=np.float64,
                    ).copy(),
                )
                for item in carried
            }
            runtime.apply(float(model.opt.timestep))
            if not release_state_checked and all(item.net_score_released for item in carried):
                for item in carried:
                    before_qpos, before_qvel = before_release[item.name]
                    qpos_jump = float(np.linalg.norm(
                        data.qpos[item.free_qposadr : item.free_qposadr + 7] - before_qpos
                    ))
                    qvel_jump = float(np.linalg.norm(
                        data.qvel[item.free_dofadr : item.free_dofadr + 6] - before_qvel
                    ))
                    require(qpos_jump <= 1.0e-12, f"{item.name} score release changed qpos")
                    require(qvel_jump <= 1.0e-12, f"{item.name} score release changed qvel")
                release_state_checked = True
            mujoco.mj_step(model, data)
            require(np.all(np.isfinite(data.qacc)), "score release produced non-finite acceleration")
        require(release_state_checked, "score release state preservation was not observed")

    for item in carried:
        require(item.net_score_released, f"{item.name} was not physically released in score phase")
        require(not item.netted, f"{item.name} remained latched to the net after score release")
        require(runtime.status_by_name()[item.name]["capture_state"] == "SCORE_RELEASED", f"{item.name} state is not SCORE_RELEASED")
        require(
            item.collector_eq_id < 0 or not bool(data.eq_active[item.collector_eq_id]),
            f"{item.name} collector weld stayed active after score release",
        )
        require_netted_gate_state(
            model,
            item,
            enabled=False,
            label=f"{item.name} did not reopen the front flap after score release",
        )
        position = runtime._buoy_center_world(item)
        horizontal_error = float(np.linalg.norm(position[:2] - score_zone[:2]))
        require(
            horizontal_error <= runtime.collector_net_score_radius_m,
            f"{item.name} settled outside selected score zone: error={horizontal_error:.3f}m",
        )


def check_magnet_release_buoyancy(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_yellow_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_yellow_1_free")
    equality = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "course_buoy_a_yellow_1_magnet_weld"))
    require(equality >= 0, "course_buoy_a_yellow_1 magnet weld equality not found")
    rod = geom_id(mujoco, model, "course_buoy_a_yellow_1_mooring_rod")

    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_yellow_1")
    dofadr = int(model.jnt_dofadr[joint])
    for _ in range(20):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
    require(int(data.eq_active[equality]) == 1, "yellow buoy magnet detached without an external/contact break force")

    data.xfrc_applied[buoy, 2] += 16.0
    for _ in range(runtime_detection_steps(runtime, model)):
        runtime.apply(float(model.opt.timestep))
        if int(data.eq_active[equality]) == 0:
            break
        mujoco.mj_step(model, data)
    require(int(data.eq_active[equality]) == 0, "yellow buoy magnet did not detach under a 16N upward force")
    require(int(model.geom_contype[rod]) == 0, "fixed mooring rod collision mask changed after release")
    require(int(model.geom_conaffinity[rod]) == 0, "fixed mooring rod affinity changed after release")
    data.xfrc_applied[buoy, 2] -= 16.0

    qposadr = int(model.jnt_qposadr[joint])
    set_buoy_center_z(mujoco, model, data, buoy, "course_buoy_a_yellow_1", -0.80)
    data.qvel[dofadr : dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    for _ in range(5000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        if float(runtime._buoy_center_world(runtime_buoy)[2]) > 0.045 and abs(float(data.qvel[dofadr + 2])) < 0.30:
            break

    final_z = float(runtime._buoy_center_world(runtime_buoy)[2])
    final_vz = float(data.qvel[dofadr + 2])
    require(final_z > 0.045, f"released yellow buoy did not float to the waterline: final_z={final_z:.3f}")
    require(final_vz > -0.30, f"released yellow buoy is still sinking at the waterline: vz={final_vz:.3f}")


def check_physical_rake_magnet_release(mujoco) -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    runtime = make_runtime(mujoco, model, data)
    dt = float(model.opt.timestep)
    vehicle_joint = joint_id(mujoco, model, "world_joint")
    vehicle_qposadr = int(model.jnt_qposadr[vehicle_joint])
    vehicle_dofadr = int(model.jnt_dofadr[vehicle_joint])
    mujoco.mj_forward(model, data)

    def set_vehicle(position: np.ndarray) -> None:
        data.qpos[vehicle_qposadr : vehicle_qposadr + 3] = position
        data.qpos[vehicle_qposadr + 3 : vehicle_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[vehicle_dofadr : vehicle_dofadr + 6] = 0.0
        mujoco.mj_forward(model, data)

    # A side/collector hit must not count as a rake-root hit.
    side_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_yellow_2")
    side_position = runtime._buoy_center_world(side_buoy).copy()
    side_contacts: set[tuple[str, str]] = set()
    for _ in range(80):
        set_vehicle(side_position + np.array([0.0, -0.300, -0.070], dtype=np.float64))
        side_contacts.update(contact_names(mujoco, model, data))
        runtime.apply(dt)
        mujoco.mj_step(model, data)
    require(bool(side_contacts), "side-contact rejection test produced no vehicle/buoy contact")
    require(not side_buoy.detached, "side or collector contact detached a buoy without a rake hit")

    # The side-contact fixture advances every attached soft weld. Reload a
    # clean model before measuring the root strike so residual compliant-weld
    # deflection cannot change the contact geometry.
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    runtime = make_runtime(mujoco, model, data)
    dt = float(model.opt.timestep)
    vehicle_joint = joint_id(mujoco, model, "world_joint")
    vehicle_qposadr = int(model.jnt_qposadr[vehicle_joint])
    vehicle_dofadr = int(model.jnt_dofadr[vehicle_joint])
    mujoco.mj_forward(model, data)

    def release_with_rake(buoy_name: str, *, check_tine_gap: bool) -> dict[str, float]:
        buoy = next(item for item in runtime.buoys if item.name == buoy_name)
        buoy_position = runtime._buoy_center_world(buoy).copy()
        observed_contacts: set[tuple[str, str]] = set()
        gap_y = -0.1000

        if check_tine_gap:
            # The 13 mm PVC stem fits through the measured 20 mm clear slot.
            gap_position = buoy_position + np.array([-0.390, -gap_y, -0.070], dtype=np.float64)
            for _ in range(30):
                set_vehicle(gap_position)
                observed_contacts.update(contact_names(mujoco, model, data))
                runtime.apply(dt)
                mujoco.mj_step(model, data)
            require(not buoy.detached, f"{buoy_name} released while its PVC passed between rake tines")

        root_position = buoy_position + np.array([-0.340, -gap_y, -0.070], dtype=np.float64)
        push_speed_mps = 0.35 if dt >= 0.006 else 0.45
        push_steps = int(np.ceil(0.65 / dt))
        contact_hold_s = 0.0
        contact_peak_n = 0.0
        release_hold_s = -1.0
        release_peak_n = -1.0
        release_qpos_jump = -1.0
        release_qvel_jump = -1.0
        release_z = float(buoy_position[2])

        for step in range(push_steps):
            set_vehicle(root_position + np.array([push_speed_mps * dt * step, 0.0, 0.0], dtype=np.float64))
            observed_contacts.update(contact_names(mujoco, model, data))
            rake_contact = buoy.body_id in runtime._release_probe_contacted_buoy_body_ids()
            if rake_contact:
                contact_hold_s += dt
                contact_peak_n = max(
                    contact_peak_n,
                    runtime._contact_force_norm(
                        buoy, vehicle_geom_ids=runtime.vehicle_release_probe_geom_ids
                    ),
                )
            else:
                contact_hold_s = 0.0
                contact_peak_n = 0.0

            before_qpos = np.asarray(
                data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7], dtype=np.float64
            ).copy()
            before_qvel = np.asarray(
                data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6], dtype=np.float64
            ).copy()
            was_detached = buoy.detached
            runtime.apply(dt)
            if buoy.detached and not was_detached:
                release_qpos_jump = float(
                    np.linalg.norm(data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7] - before_qpos)
                )
                release_qvel_jump = float(
                    np.linalg.norm(data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6] - before_qvel)
                )
                release_hold_s = contact_hold_s
                release_peak_n = contact_peak_n
                release_z = float(runtime._buoy_center_world(buoy)[2])
            mujoco.mj_step(model, data)
            if buoy.detached:
                break

        root_pair = tuple(sorted(("mission_starboard_rake_root_probe", f"{buoy_name}_pvc_pipe")))
        require(root_pair in observed_contacts, f"{buoy_name} PVC did not physically hit the rake root")
        require(buoy.detached, f"{buoy_name} did not detach on physical rake contact")
        require(
            release_hold_s <= dt + 1.0e-9,
            f"{buoy_name} did not release on first contact step: hold={release_hold_s:.4f}s",
        )
        require(release_qpos_jump <= 1.0e-6, f"{buoy_name} qpos jumped {release_qpos_jump:.9f}")
        require(release_qvel_jump <= 1.0e-6, f"{buoy_name} qvel jumped {release_qvel_jump:.9f}")
        require(
            buoy.eq_id < 0 or not bool(data.eq_active[buoy.eq_id]),
            f"{buoy_name} magnet weld stayed active after release",
        )
        return {
            "release_peak_n": release_peak_n,
            "release_hold_s": release_hold_s,
            "release_qpos_jump": release_qpos_jump,
            "release_qvel_jump": release_qvel_jump,
            "release_z": release_z,
            "push_speed_mps": push_speed_mps,
        }

    checked = {
        name: release_with_rake(name, check_tine_gap=(name.endswith("yellow_1")))
        for name in ("course_buoy_a_yellow_1", "course_buoy_a_orange_1")
    }

    # Leave the vehicle well clear and let both physically released floats rise
    # from their scene depth to the actual waterline in the same simulation.
    set_vehicle(np.array([0.0, 0.0, -4.0], dtype=np.float64))
    checked_buoys = {
        name: next(item for item in runtime.buoys if item.name == name) for name in checked
    }
    target_z = {name: runtime._surface_target_center_z(buoy) for name, buoy in checked_buoys.items()}
    max_z = {name: float(runtime._buoy_center_world(buoy)[2]) for name, buoy in checked_buoys.items()}
    surface_step: dict[str, int] = {}
    post_surface_steps = max(1, int(np.ceil(4.0 / dt)))
    max_steps = max(1, int(np.ceil(90.0 / dt)))

    for step in range(max_steps):
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "rake-release ascent produced non-finite qpos")
        require(np.all(np.isfinite(data.qvel)), "rake-release ascent produced non-finite qvel")
        for name, buoy in checked_buoys.items():
            center_z = float(runtime._buoy_center_world(buoy)[2])
            max_z[name] = max(max_z[name], center_z)
            if buoy.surface_on_waterline and center_z >= target_z[name] - 0.005:
                surface_step.setdefault(name, step)
        if len(surface_step) == len(checked_buoys) and step - max(surface_step.values()) >= post_surface_steps:
            break

    for name, buoy in checked_buoys.items():
        final_z = float(runtime._buoy_center_world(buoy)[2])
        final_vz = float(data.qvel[buoy.free_dofadr + 2])
        rise_m = final_z - checked[name]["release_z"]
        require(name in surface_step, f"{name} never reached the waterline: z={final_z:.3f}")
        require(rise_m > 8.0, f"{name} did not rise from the mooring depth: rise={rise_m:.3f}m")
        require(
            abs(final_z - target_z[name]) <= 0.035,
            f"{name} did not settle at the waterline: z={final_z:.3f}, target={target_z[name]:.3f}",
        )
        require(abs(final_vz) <= 0.20, f"{name} remained vertically unstable: vz={final_vz:.3f}m/s")
        require(
            max_z[name] <= target_z[name] + 0.080,
            f"{name} overshot the waterline: peak={max_z[name]:.3f}, target={target_z[name]:.3f}",
        )
        print(
            "rake-release-surface "
            f"name={name} speed={checked[name]['push_speed_mps']:.2f}m/s "
            f"force={checked[name]['release_peak_n']:.3f}N "
            f"hold={checked[name]['release_hold_s']:.3f}s "
            f"qpos_jump={checked[name]['release_qpos_jump']:.3e} "
            f"qvel_jump={checked[name]['release_qvel_jump']:.3e} "
            f"release_z={checked[name]['release_z']:.3f}m "
            f"final_z={final_z:.3f}m target_z={target_z[name]:.3f}m "
            f"peak_z={max_z[name]:.3f}m final_vz={final_vz:.3f}m/s"
        )


def check_attached_magnet_stays_rigid(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_yellow_1_float")
    equality = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "course_buoy_a_yellow_1_magnet_weld"))
    attach_site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "course_buoy_a_yellow_1_attach_site"))
    magnet_site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "course_buoy_a_yellow_1_magnet_site"))
    require(equality >= 0, "course_buoy_a_yellow_1 magnet equality not found")
    require(attach_site >= 0, "course_buoy_a_yellow_1 attach site not found")
    require(magnet_site >= 0, "course_buoy_a_yellow_1 magnet site not found")

    runtime = make_runtime(mujoco, model, data)
    max_tilt_deg = 0.0
    max_site_gap_m = 0.0
    for step in range(1000):
        data.xfrc_applied[buoy, :] = 0.0
        if 100 <= step < 260:
            data.xfrc_applied[buoy, 0] += 0.18
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        max_tilt_deg = max(max_tilt_deg, body_tilt_deg(data, buoy))
        max_site_gap_m = max(
            max_site_gap_m,
            float(np.linalg.norm(data.site_xpos[attach_site] - data.site_xpos[magnet_site])),
        )

    require(int(data.eq_active[equality]) == 1, "magnet detached under a sub-break lateral force")
    require(max_tilt_deg < 1.0, f"rigid rod magnet allowed excessive tilt: {max_tilt_deg:.2f}deg")
    require(max_site_gap_m < 0.003, f"rigid magnet contact gap too large: {max_site_gap_m:.3f}m")


def check_buoy_cob_generates_righting_torque(mujoco, model) -> None:
    data = mujoco.MjData(model)
    runtime = make_runtime(mujoco, model, data)
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_yellow_1")
    joint = joint_id(mujoco, model, "course_buoy_a_yellow_1_free")
    qposadr = int(model.jnt_qposadr[joint])
    roll_rad = np.deg2rad(30.0)
    data.qpos[qposadr + 2] = -1.0
    data.qpos[qposadr + 3 : qposadr + 7] = [
        np.cos(0.5 * roll_rad),
        np.sin(0.5 * roll_rad),
        0.0,
        0.0,
    ]
    mujoco.mj_forward(model, data)

    wrench = runtime._float_buoyancy_wrench(buoy, vehicle_contact=False)
    require(wrench[2] > 0.0, "submerged buoy produced no upward force")
    require(abs(float(wrench[3])) > 0.01, f"CoB offset produced no meaningful roll torque: {wrench}")
    require(float(wrench[3]) * roll_rad < 0.0, f"CoB torque reinforces roll instead of righting it: {wrench}")


def check_all_rigid_mooring_assemblies_hold(mujoco, model) -> None:
    data = mujoco.MjData(model)
    runtime = make_runtime(mujoco, model, data)
    previous_time = -1.0
    for _ in range(max(1, int(round(2.0 / float(model.opt.timestep))))):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(float(data.time) > previous_time, "rigid mooring simulation reset MuJoCo time")
        require(np.all(np.isfinite(data.qacc)), "rigid mooring simulation produced non-finite acceleration")
        previous_time = float(data.time)

    moored = [buoy for buoy in runtime.buoys if buoy.eq_id >= 0]
    require(len(moored) == 15, f"expected 15 moored buoy assemblies, got {len(moored)}")
    for buoy in moored:
        prefix = buoy.name
        attach_site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}_attach_site"))
        magnet_site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}_magnet_site"))
        base = body_id(mujoco, model, f"{prefix}_magnet_base")
        rod = geom_id(mujoco, model, f"{prefix}_mooring_rod")
        require(min(attach_site, magnet_site, rod) >= 0, f"{prefix} missing fixed mooring component")
        require(bool(data.eq_active[buoy.eq_id]), f"{prefix} magnet released without rake contact")
        require(int(model.body_dofadr[base]) < 0, f"{prefix} diver weight and rod must remain fixed")
        magnet_gap = float(np.linalg.norm(data.site_xpos[attach_site] - data.site_xpos[magnet_site]))
        rod_xy_error = float(np.linalg.norm(data.geom_xpos[rod, :2] - data.xpos[base, :2]))
        magnet_xy_error = float(np.linalg.norm(data.site_xpos[magnet_site, :2] - data.xpos[base, :2]))
        require(magnet_gap < 0.002, f"{prefix} magnetic faces separated by {magnet_gap:.4f}m")
        require(rod_xy_error < 1.0e-6, f"{prefix} mooring rod is not centered on its diver weight")
        require(magnet_xy_error < 1.0e-6, f"{prefix} fixed magnet is not centered on its mooring rod")


def check_surface_buoy_recovers_upright(mujoco, model) -> None:
    data = mujoco.MjData(model)
    runtime = make_runtime(mujoco, model, data)
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    qposadr = int(model.jnt_qposadr[joint])
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(buoy)

    data.qpos[qposadr + 3 : qposadr + 7] = [2.0**-0.5, 2.0**-0.5, 0.0, 0.0]
    data.qvel[dofadr : dofadr + 6] = 0.0
    set_buoy_center_z(mujoco, model, data, buoy.body_id, buoy.name, target_z)
    buoy.detached = True
    buoy.release_time_s = 0.0
    start_tilt_deg = body_tilt_deg(data, buoy.body_id)

    for _ in range(max(1, int(round(8.0 / float(model.opt.timestep))))):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "surface buoy upright recovery produced non-finite pose")

    final_tilt_deg = body_tilt_deg(data, buoy.body_id)
    final_center_z = float(runtime._buoy_center_world(buoy)[2])
    require(start_tilt_deg > 85.0, f"upright recovery test did not start horizontal: {start_tilt_deg:.2f}deg")
    require(final_tilt_deg < 10.0, f"surface buoy remained on its side: final tilt={final_tilt_deg:.2f}deg")
    require(abs(final_center_z - target_z) < 0.08, f"upright surface buoy left waterline: z={final_center_z:.3f} target={target_z:.3f}")


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

    set_buoy_center_z(mujoco, model, data, buoy, "course_buoy_a_red_1", target_z)
    data.qvel[dofadr : dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)
    start = runtime._buoy_center_world(runtime_buoy)
    for _ in range(400):
        data.xfrc_applied[buoy, :] = 0.0
        data.xfrc_applied[buoy, 0] = 0.18
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    delta = runtime._buoy_center_world(runtime_buoy) - start
    require(delta[0] > 0.06, f"red buoy did not move freely under horizontal force: dx={delta[0]:.3f}m")
    require(abs(float(data.qpos[qposadr + 2]) - target_z) < 0.20, "red buoy left the surface band while moving horizontally")


def check_gui_low_profile_red_buoy_contact_stability(mujoco) -> None:
    for check in (check_red_buoy_moves_by_contact, check_red_buoy_stays_surface_when_collector_descends):
        model = mujoco.MjModel.from_xml_path(str(SCENE))
        model.opt.timestep = 0.005
        check(mujoco, model, runtime_factory=make_low_profile_runtime)


def check_scene_red_buoy_floats_without_input(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    joint = joint_id(mujoco, model, "course_buoy_a_red_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)
    start_z = float(runtime._buoy_center_world(runtime_buoy)[2])

    z_values: list[float] = []
    for _ in range(10000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        z_values.append(float(runtime._buoy_center_world(runtime_buoy)[2]))

    final_z = float(runtime._buoy_center_world(runtime_buoy)[2])
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

    set_buoy_center_z(mujoco, model, data, buoy, "course_buoy_a_red_1", target_z)
    data.qvel[dofadr + 2] = 0.90
    mujoco.mj_forward(model, data)
    z_values: list[float] = []
    for _ in range(600):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        z_values.append(float(runtime._buoy_center_world(runtime_buoy)[2]))

    require(max(z_values) > target_z + 0.001, f"red buoy depth appears clamped: max_z={max(z_values):.3f}, target={target_z:.3f}")
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

    set_buoy_center_z(mujoco, model, data, buoy, "course_buoy_a_red_1", target_z - 0.35)
    data.qvel[dofadr + 2] = -0.20
    mujoco.mj_forward(model, data)
    start_z = float(runtime._buoy_center_world(runtime_buoy)[2])
    for _ in range(2500):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    final_z = float(runtime._buoy_center_world(runtime_buoy)[2])
    require(final_z > start_z + 0.20, f"free red buoy did not rise from subsurface: start_z={start_z:.3f} final_z={final_z:.3f}")
    require(final_z > target_z - 0.12, f"free red buoy did not return near surface: final_z={final_z:.3f} target_z={target_z:.3f}")


def check_surface_pinger_angular_guard(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoy = body_id(mujoco, model, "course_buoy_pinger_white_1_float")
    joint = joint_id(mujoco, model, "course_buoy_pinger_white_1_free")
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = next(item for item in runtime.buoys if item.name == "course_buoy_pinger_white_1")
    qposadr = int(model.jnt_qposadr[joint])
    dofadr = int(model.jnt_dofadr[joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)

    data.qpos[qposadr + 3 : qposadr + 7] = [0.707, 0.707, 0.0, 0.0]
    set_buoy_center_z(mujoco, model, data, buoy, "course_buoy_pinger_white_1", target_z + 0.02)
    data.qvel[dofadr + 3 : dofadr + 6] = [8.0, -6.0, 4.0]
    if hasattr(data, "qacc"):
        data.qacc[dofadr + 3 : dofadr + 6] = [1.0e6, -1.0e6, 5.0e5]
    runtime_buoy.detached = True
    runtime_buoy.release_time_s = 0.0
    runtime_buoy.surface_on_waterline = False
    if runtime_buoy.eq_id >= 0:
        data.eq_active[runtime_buoy.eq_id] = 0
    if runtime_buoy.flex_line_top_eq_id >= 0:
        data.eq_active[runtime_buoy.flex_line_top_eq_id] = 0
    mujoco.mj_forward(model, data)

    before_qpos = np.array(data.qpos[qposadr : qposadr + 7], dtype=np.float64)
    before_qvel = np.array(data.qvel[dofadr : dofadr + 6], dtype=np.float64)
    before_qacc = np.array(data.qacc[dofadr : dofadr + 6], dtype=np.float64)
    runtime._apply_surface_float_guard(runtime_buoy, vehicle_contact=False)

    require(
        np.array_equal(data.qpos[qposadr : qposadr + 7], before_qpos),
        "surface guard changed pinger qpos instead of applying a continuous wrench",
    )
    require(
        np.array_equal(data.qvel[dofadr : dofadr + 6], before_qvel),
        "surface guard changed pinger qvel instead of applying a continuous wrench",
    )
    require(
        np.array_equal(data.qacc[dofadr : dofadr + 6], before_qacc),
        "surface guard changed pinger qacc instead of applying a continuous wrench",
    )

    initial_angular_speed = float(np.linalg.norm(before_qvel[3:6]))
    angular_speeds: list[float] = []
    for _ in range(max(1, int(round(2.0 / float(model.opt.timestep))))):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "surface pinger damping produced non-finite pose")
        require(np.all(np.isfinite(data.qvel)), "surface pinger damping produced non-finite velocity")
        require(np.all(np.isfinite(data.qacc)), "surface pinger damping produced non-finite acceleration")
        angular_speeds.append(float(np.linalg.norm(data.qvel[dofadr + 3 : dofadr + 6])))

    final_angular_speed = angular_speeds[-1]
    require(
        max(angular_speeds) <= initial_angular_speed * 1.05,
        f"surface pinger angular damping amplified rate: max={max(angular_speeds):.3f}",
    )
    require(
        final_angular_speed <= max(0.50, initial_angular_speed * 0.10),
        f"surface pinger angular rate did not decay: initial={initial_angular_speed:.3f} "
        f"final={final_angular_speed:.3f}",
    )


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

    set_buoy_center_z(mujoco, model, data, buoy, "course_buoy_a_yellow_1", -2.0)
    data.qvel[dofadr + 2] = -3.0
    mujoco.mj_forward(model, data)
    for _ in range(3000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)

    final_z = float(runtime._buoy_center_world(runtime_buoy)[2])
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
    top_pair = tuple(sorted((buoy_geom, "collector_top_net_proxy")))
    require(
        top_pair not in contact_names(mujoco, model, data),
        "collector top net should not trap or push surface buoys downward",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--extended",
        action="store_true",
        help="also run long-duration force, bobbing, and impulse recovery checks",
    )
    args = parser.parse_args()

    import mujoco

    check_soft_net_runtime_contract()
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    # Exercise the same physics cadence used by both normal GUI competition
    # runs and GUI-started pinger homing.  The XML's 2 ms authoring value is
    # more forgiving and previously let the 13-buoy collector suite pass while
    # the former 8 ms balanced profile missed its slot-settle contract.
    model.opt.timestep = PRODUCTION_COLLECTOR_TIMESTEP_S
    require(
        abs(float(model.opt.timestep) - PRODUCTION_COLLECTOR_TIMESTEP_S) <= 1.0e-12,
        "collector acceptance is not running at the production timestep",
    )
    check_collision_flags(mujoco, model)
    check_rigid_mooring_rods(mujoco, model)
    check_red_buoy_pocket_contacts(mujoco, model)
    check_collector_net_surface_gate(mujoco, model)
    check_score_release_requires_release_phase_and_target_zone(mujoco, model)
    check_thirteen_buoys_enter_and_settle_sequentially(mujoco, model)
    check_thirteen_buoys_stay_inside_physical_net(mujoco, model)
    check_reverse_motion_opens_physical_mouth(mujoco, model)
    check_thirteen_buoys_release_into_selected_score_zone(mujoco, model)
    check_full_immersion_net_lift_contract(mujoco, model)
    check_physical_rake_magnet_release(mujoco)
    check_buoy_cob_generates_righting_torque(mujoco, model)
    check_surface_buoy_recovers_upright(mujoco, model)
    check_free_buoy_pocket_contacts(mujoco, model)
    if args.extended:
        check_red_buoy_moves_by_contact(mujoco, model)
        check_red_buoy_stays_surface_when_collector_descends(mujoco, model)
        check_gui_low_profile_red_buoy_contact_stability(mujoco)
        check_attached_magnet_stays_rigid(mujoco, model)
        check_all_rigid_mooring_assemblies_hold(mujoco, model)
        check_magnet_release_buoyancy(mujoco, model)
        check_surface_buoy_moves_under_force(mujoco, model)
        check_scene_red_buoy_floats_without_input(mujoco, model)
        check_surface_buoy_bobs_without_depth_lock(mujoco, model)
        check_free_buoy_rises_from_subsurface(mujoco, model)
        check_surface_pinger_angular_guard(mujoco, model)
        check_released_buoy_recovers_from_downward_impulse(mujoco, model)
    print(
        "buoy collector physical pocket: ok "
        f"(extended={args.extended}, production_dt={float(model.opt.timestep):.3f}s)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
