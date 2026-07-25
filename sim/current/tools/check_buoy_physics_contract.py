#!/usr/bin/env python3
"""Fast physical-contract check for course buoys and the collector."""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
FSM_SCENE = ROOT.parents[1] / "kmu26_mission_fsm" / "config" / "tank_current_scene.xml"
sys.path.insert(0, str(ROOT))

from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402
from sim.runtime.simulation_loop_viewer_runner import (  # noqa: E402
    _sync_viewer_preserving_applied_wrenches,
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def object_id(mujoco, model, kind, name: str) -> int:
    value = int(mujoco.mj_name2id(model, kind, name))
    require(value >= 0, f"missing MuJoCo object: {name}")
    return value


def runtime_for(mujoco, model, data, *, update_hz: float = 0.0) -> CourseBuoyRuntime:
    def env_float(name: str, default: float) -> float:
        return float(update_hz) if name == "UUV_COURSE_BUOY_UPDATE_HZ" else default

    def env_flag(name: str, default: bool) -> bool:
        if name == "UUV_COURSE_BUOY_TRACK_CSV_ENABLE":
            return False
        return default

    return CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=0.0,
        env_float=env_float,
        env_flag=env_flag,
        log=lambda _message: None,
    )


def contact_pairs(mujoco, model, data) -> set[tuple[str, str]]:
    pairs: set[tuple[str, str]] = set()
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        left = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
        right = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
        pairs.add(tuple(sorted((left, right))))
    return pairs


def has_physical_rake_contact(pairs: set[tuple[str, str]], buoy_name: str) -> bool:
    """Return true for a buoy hit on any of the 10 tines or either rake root."""

    return any(
        any(name.startswith(f"{buoy_name}_") for name in pair)
        and any(name.startswith(("mission_port_rake_", "mission_starboard_rake_")) for name in pair)
        for pair in pairs
    )


def place_body_center(mujoco, model, data, buoy, target: np.ndarray) -> None:
    current = buoy_center(data, buoy)
    data.qpos[buoy.free_qposadr : buoy.free_qposadr + 3] += target - current
    data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)


def buoy_center(data, buoy) -> np.ndarray:
    if buoy.cob_site_id >= 0:
        return np.asarray(data.site_xpos[buoy.cob_site_id], dtype=np.float64).copy()
    return np.asarray(data.xpos[buoy.body_id], dtype=np.float64).copy()


def check_compiled_contract(mujoco, model, runtime: CourseBuoyRuntime) -> None:
    require(len(runtime.buoys) == 25, f"expected 25 dynamic floats, got {len(runtime.buoys)}")
    magnet_welds = 0
    collector_welds = 0
    for equality_id in range(int(model.neq)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_id) or ""
        magnet_welds += int(name.endswith("_magnet_weld"))
        collector_welds += int(name.endswith("_collector_weld"))
    require(magnet_welds == 15, f"expected 15 magnetic attachments, got {magnet_welds}")
    require(collector_welds == 25, f"expected 25 soft collector welds, got {collector_welds}")
    require(len(runtime.vehicle_release_probe_geom_ids) == 12, "all 10 rake tines and 2 rake roots must trigger release")
    require(abs(runtime.buoyancy_n - 1.0) <= 1.0e-12, "net buoyancy must be 1.0 N")
    require(abs(runtime.break_force_n - 15.0) <= 1.0e-12, "magnet break force must be 15 N")
    require(abs(runtime.contact_release_hold_s) <= 1.0e-12, "rake contact must release immediately")
    require(
        abs(runtime.release_collision_grace_s) <= 1.0e-12,
        "released buoy collision continuity must default to zero ghost time",
    )
    require(not runtime.proximity_release_enable, "proximity-based rake release must default off")

    for buoy in runtime.buoys:
        require(buoy.free_qposadr >= 0 and buoy.free_dofadr >= 0, f"{buoy.name} is not a free body")
        require(
            abs(float(model.body_mass[buoy.body_id]) - 0.010) <= 1.0e-9,
            f"{buoy.name} mass is not 10 g",
        )
        require(
            np.allclose(model.body_ipos[buoy.body_id], [0.0, 0.0, -0.035], atol=1.0e-9),
            f"{buoy.name} center of mass is not 35 mm below the body frame",
        )
        require(buoy.cob_site_id >= 0, f"{buoy.name} has no center-of-buoyancy site")
        require(buoy.collector_eq_id >= 0, f"{buoy.name} has no soft collector weld")
        require(not bool(runtime.data.eq_active[buoy.collector_eq_id]), f"{buoy.name} collector weld starts active")
        require(
            float(model.eq_solref[buoy.collector_eq_id, 0]) >= 0.05,
            f"{buoy.name} collector weld is not soft",
        )
        float_geom = object_id(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"{buoy.name}_float_geom",
        )
        require(int(model.geom_bodyid[float_geom]) == buoy.body_id, f"{buoy.name} float geom is not dynamic")
        require(int(model.geom_contype[float_geom]) != 0, f"{buoy.name} float geom cannot collide")


def check_rake_release_and_rise(
    mujoco,
    model,
    data,
    runtime: CourseBuoyRuntime,
    *,
    buoy_name: str = "course_buoy_a_yellow_1",
) -> tuple[float, float, float, float, float]:
    buoy = next(item for item in runtime.buoys if item.name == buoy_name)
    world_joint = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    vehicle_qpos = int(model.jnt_qposadr[world_joint])
    vehicle_dof = int(model.jnt_dofadr[world_joint])
    equality = object_id(
        mujoco,
        model,
        mujoco.mjtObj.mjOBJ_EQUALITY,
        f"{buoy_name}_magnet_weld",
    )
    start = buoy_center(data, buoy)
    observed: set[tuple[str, str]] = set()
    release_jump = -1.0
    release_velocity_jump = -1.0
    release_contact_hold = -1.0
    release_contact_peak = -1.0
    release_z = float(start[2])
    current_contact_hold = 0.0
    current_contact_peak = 0.0

    def hold_vehicle(position: np.ndarray, steps: int, step_delta: np.ndarray | None = None) -> None:
        nonlocal release_jump, release_velocity_jump, release_contact_hold, release_contact_peak, release_z
        nonlocal current_contact_hold, current_contact_peak
        delta = np.zeros(3, dtype=np.float64) if step_delta is None else step_delta
        for step in range(steps):
            data.qpos[vehicle_qpos : vehicle_qpos + 3] = position + step * delta
            data.qpos[vehicle_qpos + 3 : vehicle_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[vehicle_dof : vehicle_dof + 6] = 0.0
            mujoco.mj_forward(model, data)
            observed.update(contact_pairs(mujoco, model, data))
            rake_contact = buoy.body_id in runtime._release_probe_contacted_buoy_body_ids()
            if rake_contact:
                current_contact_hold += float(model.opt.timestep)
                current_contact_peak = max(
                    current_contact_peak,
                    runtime._contact_force_norm(
                        buoy, vehicle_geom_ids=runtime.vehicle_release_probe_geom_ids
                    ),
                )
            else:
                current_contact_hold = 0.0
                current_contact_peak = 0.0
            before_pos = buoy_center(data, buoy)
            before_vel = np.asarray(data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6]).copy()
            was_detached = buoy.detached
            runtime.apply(float(model.opt.timestep))
            if buoy.detached and not was_detached:
                release_jump = float(np.linalg.norm(buoy_center(data, buoy) - before_pos))
                release_velocity_jump = float(
                    np.linalg.norm(data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6] - before_vel)
                )
                release_contact_hold = current_contact_hold
                release_contact_peak = current_contact_peak
                release_z = float(buoy_center(data, buoy)[2])
            mujoco.mj_step(model, data)

    gap_y = -0.100
    hold_vehicle(start + np.array([-0.390, -gap_y, -0.070]), 30)
    require(not buoy.detached, "passing between rake tines released the magnet")
    # Continue driving the rake through the stem. Holding the base at the
    # first-contact pose lets the compliant weld move the stem out of contact
    # and does not represent an actual forward capture attempt.
    root_steps = int(math.ceil(0.65 / float(model.opt.timestep)))
    hold_vehicle(
        start + np.array([-0.340, -gap_y, -0.070]),
        root_steps,
        np.array([0.45 * float(model.opt.timestep), 0.0, 0.0]),
    )
    require(has_physical_rake_contact(observed, buoy.name), "PVC never contacted a physical rake geom")
    require(buoy.detached and int(data.eq_active[equality]) == 0, "rake contact did not release magnet weld")
    require(release_contact_peak >= 0.0, "rake contact force sample is invalid")
    require(release_jump <= 1.0e-9, f"release changed position by {release_jump:.9f} m")
    require(release_velocity_jump <= 1.0e-9, f"release changed velocity by {release_velocity_jump:.9f} m/s")
    require(
        release_contact_hold <= float(model.opt.timestep) + 1.0e-9,
        f"magnet did not release on the first contact step: {release_contact_hold:.4f} s",
    )

    data.qpos[vehicle_qpos : vehicle_qpos + 3] = start + np.array([-3.0, 0.0, 0.0])
    data.qvel[vehicle_dof : vehicle_dof + 6] = 0.0
    mujoco.mj_forward(model, data)
    for _ in range(1200):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
    rise = float(buoy_center(data, buoy)[2]) - release_z
    require(rise > 0.10, f"released buoy did not rise under buoyancy: {rise:.3f} m")
    return release_jump, release_velocity_jump, release_contact_hold, release_contact_peak, rise


def check_low_force_rake_contact_releases(mujoco) -> float:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_yellow_1")
    world_joint = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    vehicle_qpos = int(model.jnt_qposadr[world_joint])
    vehicle_dof = int(model.jnt_dofadr[world_joint])
    start = buoy_center(data, buoy)
    dt = float(model.opt.timestep)
    root = start + np.array([-0.340, 0.100, -0.070], dtype=np.float64)
    peak_rake_force_n = 0.0
    observed: set[tuple[str, str]] = set()
    steps = int(math.ceil(0.65 / dt))

    for step in range(steps):
        data.qpos[vehicle_qpos : vehicle_qpos + 3] = root + np.array([0.30 * dt * step, 0.0, 0.0])
        data.qpos[vehicle_qpos + 3 : vehicle_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[vehicle_dof : vehicle_dof + 6] = 0.0
        mujoco.mj_forward(model, data)
        observed.update(contact_pairs(mujoco, model, data))
        if buoy.body_id in runtime._release_probe_contacted_buoy_body_ids():
            peak_rake_force_n = max(
                peak_rake_force_n,
                runtime._contact_force_norm(
                    buoy, vehicle_geom_ids=runtime.vehicle_release_probe_geom_ids
                ),
            )
        runtime.apply(dt)
        mujoco.mj_step(model, data)

    require(
        has_physical_rake_contact(observed, buoy.name),
        f"fixture never reached a physical rake geom; observed={sorted(observed)}",
    )
    require(
        math.isfinite(peak_rake_force_n) and peak_rake_force_n >= 0.0,
        "rake-contact force diagnostic is invalid",
    )
    require(
        peak_rake_force_n < runtime.break_force_n,
        f"subthreshold fixture unexpectedly exceeded break load: {peak_rake_force_n:.3f} N",
    )
    require(buoy.detached, f"physical rake contact did not release at {peak_rake_force_n:.3f} N")
    return peak_rake_force_n


def check_runtime_timestep_rake_release(mujoco) -> dict[str, tuple[float, float]]:
    """Verify the 8 ms GUI/viewer physics cadence, not only MuJoCo's 2 ms default."""

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = 0.008
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    world_joint = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    vehicle_qpos = int(model.jnt_qposadr[world_joint])
    vehicle_dof = int(model.jnt_dofadr[world_joint])
    dt = float(model.opt.timestep)
    results: dict[str, tuple[float, float]] = {}

    for name in ("course_buoy_a_yellow_1", "course_buoy_a_orange_1"):
        buoy = next(item for item in runtime.buoys if item.name == name)
        start = buoy_center(data, buoy)
        root = start + np.array([-0.340, 0.100, -0.070], dtype=np.float64)
        peak_rake_force_n = 0.0
        contact_hold_s = 0.0
        release_hold_s = -1.0
        release_qpos_jump = -1.0
        release_qvel_jump = -1.0
        observed: set[tuple[str, str]] = set()
        steps = int(math.ceil(0.65 / dt))

        for step in range(steps):
            data.qpos[vehicle_qpos : vehicle_qpos + 3] = root + np.array([0.35 * dt * step, 0.0, 0.0])
            data.qpos[vehicle_qpos + 3 : vehicle_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[vehicle_dof : vehicle_dof + 6] = 0.0
            mujoco.mj_forward(model, data)
            observed.update(contact_pairs(mujoco, model, data))
            rake_contact = buoy.body_id in runtime._release_probe_contacted_buoy_body_ids()
            if rake_contact:
                contact_hold_s += dt
                peak_rake_force_n = max(
                    peak_rake_force_n,
                    runtime._contact_force_norm(
                        buoy, vehicle_geom_ids=runtime.vehicle_release_probe_geom_ids
                    ),
                )
            else:
                contact_hold_s = 0.0
                peak_rake_force_n = 0.0
            before_qpos = np.asarray(
                data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7], dtype=np.float64
            ).copy()
            before_qvel = np.asarray(
                data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6], dtype=np.float64
            ).copy()
            was_detached = buoy.detached
            runtime.apply(dt)
            if buoy.detached and not was_detached:
                release_hold_s = contact_hold_s
                release_qpos_jump = float(
                    np.linalg.norm(data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7] - before_qpos)
                )
                release_qvel_jump = float(
                    np.linalg.norm(data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6] - before_qvel)
                )
                require(not buoy.collisions_suppressed, f"8 ms {name} disabled buoy collisions at release")
                for geom_id, contype, conaffinity in buoy.geom_collision_bits:
                    require(
                        int(model.geom_contype[geom_id]) == contype
                        and int(model.geom_conaffinity[geom_id]) == conaffinity,
                        f"8 ms {name} changed collision mask at release",
                    )
            mujoco.mj_step(model, data)
            if buoy.detached:
                break

        require(
            has_physical_rake_contact(observed, name),
            f"8 ms {name} never contacted a physical rake geom",
        )
        require(buoy.detached, f"8 ms {name} did not release on physical rake contact")
        require(
            release_hold_s <= dt + 1.0e-9,
            f"8 ms {name} did not release on first contact step: {release_hold_s:.3f} s",
        )
        require(release_qpos_jump <= 1.0e-6, f"8 ms {name} qpos jumped {release_qpos_jump:.9f}")
        require(release_qvel_jump <= 1.0e-6, f"8 ms {name} qvel jumped {release_qvel_jump:.9f}")
        results[name] = (peak_rake_force_n, release_hold_s)

    return results


def check_released_buoy_preserves_rigid_collisions(mujoco) -> tuple[float, float]:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = 0.008
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    runtime.collector_net_enable = False
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_yellow_1")
    base = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    collector = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "front_open_buoy_collector")
    require(int(model.body_dofnum[collector]) == 0, "collector must be rigidly attached to base_link")

    data.xfrc_applied[buoy.body_id, 2] += 16.0
    runtime.apply(float(model.opt.timestep))
    data.xfrc_applied[buoy.body_id, 2] -= 16.0
    require(buoy.detached, "external break load did not release the buoy")
    require(not buoy.collisions_suppressed, "release created a collision ghost window")
    require(
        float(runtime._buoy_center_world(buoy)[2]) < runtime._surface_target_center_z(buoy) - 1.0,
        "collision continuity fixture must release below the surface",
    )
    for geom_id, contype, conaffinity in buoy.geom_collision_bits:
        require(int(model.geom_contype[geom_id]) == contype, "release changed buoy contype")
        require(int(model.geom_conaffinity[geom_id]) == conaffinity, "release changed buoy conaffinity")

    # Advance several live 8 ms steps: masks must remain physical continuously,
    # rather than reappearing after a delayed grace period.
    for _ in range(4):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(not buoy.collisions_suppressed, "released buoy entered a delayed collision ghost window")
        for geom_id, contype, conaffinity in buoy.geom_collision_bits:
            require(
                int(model.geom_contype[geom_id]) == contype
                and int(model.geom_conaffinity[geom_id]) == conaffinity,
                "released buoy collision continuity was not preserved",
            )

    world_joint = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    vehicle_qpos = int(model.jnt_qposadr[world_joint])
    vehicle_dof = int(model.jnt_dofadr[world_joint])
    held_base_pose = np.asarray(data.qpos[vehicle_qpos : vehicle_qpos + 7], dtype=np.float64).copy()
    place_body_center(
        mujoco,
        model,
        data,
        buoy,
        np.asarray(data.xpos[base], dtype=np.float64) + np.array([-0.240, 0.0, 0.355]),
    )
    data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6] = 0.0
    data.qvel[buoy.free_dofadr] = -0.50
    back_contact = tuple(sorted((f"{buoy.name}_float_geom", "collector_back_net_proxy")))
    observed_contact = False
    min_local_x = float("inf")
    for _ in range(max(1, int(np.ceil(1.2 / float(model.opt.timestep))))):
        data.qpos[vehicle_qpos : vehicle_qpos + 7] = held_base_pose
        data.qvel[vehicle_dof : vehicle_dof + 6] = 0.0
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        observed_contact = observed_contact or back_contact in contact_pairs(mujoco, model, data)
        local = np.asarray(data.xmat[base], dtype=np.float64).reshape(3, 3).T @ (
            buoy_center(data, buoy) - np.asarray(data.xpos[base], dtype=np.float64)
        )
        min_local_x = min(min_local_x, float(local[0]))
        require(np.all(np.isfinite(data.qpos)), "rigid net collision produced non-finite qpos")
        require(np.all(np.isfinite(data.qvel)), "rigid net collision produced non-finite qvel")
    require(observed_contact, "restored buoy never contacted the rigid collector back net")
    require(min_local_x > -0.30, f"released buoy ghosted through the rigid back net: x={min_local_x:.3f}")
    return 0.0, min_local_x


def check_physical_collector(mujoco, model) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    base = object_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    rotation = np.asarray(data.xmat[base], dtype=np.float64).reshape(3, 3)
    target = np.asarray(data.xpos[base], dtype=np.float64) + rotation @ np.array([-0.265, 0.0, 0.230])
    place_body_center(mujoco, model, data, buoy, target)
    expected = tuple(sorted((f"{buoy.name}_float_geom", "collector_back_net_proxy")))
    require(expected in contact_pairs(mujoco, model, data), "buoy did not contact physical collector back net")

    before_qpos = np.asarray(data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7]).copy()
    before_qvel = np.asarray(data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6]).copy()
    wrench = runtime._collector_net_wrench(buoy)
    require(np.array_equal(wrench, np.zeros(6)), "collector pulled a buoy that never crossed the mouth")
    require(np.array_equal(before_qpos, data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7]), "collector moved buoy qpos")
    require(np.array_equal(before_qvel, data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6]), "collector changed buoy qvel")


def check_scene_contract_mirror() -> None:
    require(FSM_SCENE.exists(), f"FSM scene mirror is missing: {FSM_SCENE}")

    def contracts(path: Path) -> tuple[dict[str, dict[str, str]], dict[str, dict[str, str]]]:
        root = ET.parse(path).getroot()
        equalities = {
            str(element.get("name")): dict(element.attrib)
            for element in root.findall("./equality/*")
            if str(element.get("name", "")).endswith(("_collector_weld", "_magnet_weld"))
        }
        geoms = {
            str(element.get("name")): dict(element.attrib)
            for element in root.iter("geom")
            if str(element.get("name", "")).startswith(
                ("collector_", "mission_port_rake_", "mission_starboard_rake_", "course_buoy_")
            )
        }
        return equalities, geoms

    current_contract = contracts(SCENE)
    fsm_contract = contracts(FSM_SCENE)
    require(current_contract[0] == fsm_contract[0], "current/FSM buoy equality contracts differ")
    require(current_contract[1] == fsm_contract[1], "current/FSM collector/rake/buoy geom contracts differ")


def check_all_buoys_idle_stability(mujoco, model) -> tuple[float, float, float]:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    initial = {buoy.name: buoy_center(data, buoy) for buoy in runtime.buoys}
    previous = {name: value.copy() for name, value in initial.items()}
    max_surface_step = 0.0
    for _ in range(2000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        require(np.all(np.isfinite(data.qpos)), "non-finite qpos during idle buoy simulation")
        require(np.all(np.isfinite(data.qvel)), "non-finite qvel during idle buoy simulation")
        for buoy in runtime.buoys:
            if buoy.has_magnet:
                continue
            current = buoy_center(data, buoy)
            max_surface_step = max(max_surface_step, float(np.linalg.norm(current - previous[buoy.name])))
            previous[buoy.name] = current

    max_attached_motion = 0.0
    max_surface_drift = 0.0
    for buoy in runtime.buoys:
        motion = float(np.linalg.norm(buoy_center(data, buoy) - initial[buoy.name]))
        if buoy.has_magnet:
            require(not buoy.detached, f"{buoy.name} detached without rake/external force")
            require(buoy.eq_id >= 0 and bool(data.eq_active[buoy.eq_id]), f"{buoy.name} magnet weld became inactive")
            max_attached_motion = max(max_attached_motion, motion)
        else:
            require(float(buoy_center(data, buoy)[2]) > -0.05, f"surface buoy sank: {buoy.name}")
            max_surface_drift = max(max_surface_drift, motion)
    require(max_attached_motion < 0.005, f"attached buoy drifted {max_attached_motion:.4f} m")
    require(max_surface_step < 0.020, f"surface buoy jumped {max_surface_step:.4f} m in one step")
    require(max_surface_drift < 0.050, f"surface buoy drifted {max_surface_drift:.4f} m while idle")
    return max_attached_motion, max_surface_step, max_surface_drift


def check_viewer_sync_preserves_surface_buoyancy(mujoco) -> float:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = 0.008
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def env_float(name: str, default: float) -> float:
        return 10.0 if name == "UUV_COURSE_BUOY_UPDATE_HZ" else default

    def env_flag(name: str, default: bool) -> bool:
        return False if name == "UUV_COURSE_BUOY_TRACK_CSV_ENABLE" else default

    runtime = CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=0.0,
        env_float=env_float,
        env_flag=env_flag,
        log=lambda _message: None,
    )
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
    start = buoy_center(data, buoy)

    class ClearingViewer:
        @staticmethod
        def sync(*, state_only: bool) -> None:
            require(state_only, "regression viewer must use state-only sync")
            data.xfrc_applied[:] = 0.0
            data.qfrc_applied[:] = 0.0

    old_value = os.environ.get("UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC")
    os.environ["UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC"] = "1"
    max_drift = 0.0
    try:
        for step in range(625):
            runtime.apply(float(model.opt.timestep))
            mujoco.mj_step(model, data)
            max_drift = max(max_drift, float(np.linalg.norm(buoy_center(data, buoy) - start)))
            if step % 4 == 3:
                _sync_viewer_preserving_applied_wrenches(runtime, ClearingViewer())
    finally:
        if old_value is None:
            os.environ.pop("UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC", None)
        else:
            os.environ["UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC"] = old_value

    require(max_drift < 0.005, f"viewer sync cleared buoyancy wrench; drift={max_drift:.4f} m")
    return max_drift


def check_throttled_logic_keeps_stepwise_hydrodynamics(mujoco) -> tuple[float, float, float]:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = 0.008
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data, update_hz=10.0)
    buoy = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")

    start_z = -0.8
    place_body_center(mujoco, model, data, buoy, np.array([*buoy_center(data, buoy)[:2], start_z]))
    buoy.surface_on_waterline = False
    previous_z = float(buoy_center(data, buoy)[2])
    max_step = 0.0
    max_z = previous_z
    for _ in range(1000):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        current_z = float(buoy_center(data, buoy)[2])
        max_step = max(max_step, abs(current_z - previous_z))
        max_z = max(max_z, current_z)
        previous_z = current_z

    target_z = float(runtime._surface_target_center_z(buoy))
    final_z = float(buoy_center(data, buoy)[2])
    final_vz = float(data.qvel[buoy.free_dofadr + 2])
    require(max_step < 0.020, f"10 Hz logic caused a per-step buoy jump: {max_step:.4f} m")
    require(max_z <= target_z + 0.015, f"10 Hz logic overshot waterline: {max_z:.4f} m")
    require(abs(final_z - target_z) < 0.010, f"10 Hz logic did not settle at waterline: {final_z:.4f} m")
    require(abs(final_vz) < 0.050, f"10 Hz logic left buoy moving at waterline: {final_vz:.4f} m/s")
    return final_z - start_z, max_step, max(0.0, max_z - target_z)


def main() -> int:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    check_compiled_contract(mujoco, model, runtime)
    low_force_contact_peak = check_low_force_rake_contact_releases(mujoco)
    runtime_timestep_release = check_runtime_timestep_rake_release(mujoco)
    collision_ghost_time, rigid_net_min_x = check_released_buoy_preserves_rigid_collisions(mujoco)
    position_jump, velocity_jump, contact_hold, contact_peak, rise = check_rake_release_and_rise(
        mujoco, model, data, runtime
    )
    check_physical_collector(mujoco, model)
    check_scene_contract_mirror()
    max_attached_motion, max_surface_step, max_surface_drift = check_all_buoys_idle_stability(mujoco, model)
    viewer_surface_drift = check_viewer_sync_preserves_surface_buoyancy(mujoco)
    throttled_rise, throttled_max_step, throttled_overshoot = check_throttled_logic_keeps_stepwise_hydrodynamics(
        mujoco
    )
    print(
        "buoy physics: PASS "
        f"floats={len(runtime.buoys)} magnet_welds=15 collector_welds=25 "
        "mass=0.010kg net_buoyancy=1.000N "
        f"rake_low_force_contact_peak={low_force_contact_peak:.3f}N "
        f"rake_contact_hold={contact_hold:.3f}s rake_contact_peak={contact_peak:.3f}N "
        f"runtime8_yellow={runtime_timestep_release['course_buoy_a_yellow_1'][0]:.3f}N/"
        f"{runtime_timestep_release['course_buoy_a_yellow_1'][1]:.3f}s "
        f"runtime8_orange={runtime_timestep_release['course_buoy_a_orange_1'][0]:.3f}N/"
        f"{runtime_timestep_release['course_buoy_a_orange_1'][1]:.3f}s "
        f"collision_ghost={collision_ghost_time:.3f}s rigid_net_min_x={rigid_net_min_x:.3f}m "
        f"release_position_jump={position_jump:.9f}m "
        f"release_velocity_jump={velocity_jump:.9f}m/s rise_1.2s={rise:.3f}m "
        f"max_attached_motion_2s={max_attached_motion:.6f}m "
        f"max_surface_step={max_surface_step:.6f}m max_surface_drift={max_surface_drift:.6f}m "
        f"viewer_surface_drift={viewer_surface_drift:.6f}m "
        f"logic10_rise={throttled_rise:.3f}m logic10_max_step={throttled_max_step:.6f}m "
        f"logic10_overshoot={throttled_overshoot:.6f}m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
