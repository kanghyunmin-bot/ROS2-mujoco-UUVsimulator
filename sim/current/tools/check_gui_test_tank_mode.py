#!/usr/bin/env python3
"""Verify GUI test-tank config, generated MJCF, and MuJoCo load contract."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.test_tank_layout_model import (  # noqa: E402
    COURSE_MODE_TEST_TANK,
    TEST_TANK_DEPTH_M,
    TEST_TANK_LENGTH_M,
    TEST_TANK_NYLON_LENGTH_M,
    TEST_TANK_PINGER_ID,
    TEST_TANK_PINGER_SITE_NAME,
    TEST_TANK_PINGER_Z_M,
    TEST_TANK_WIDTH_M,
    TEST_TANK_YELLOW_BUOY_PREFIX,
    TEST_TANK_YELLOW_ID,
    default_course_layout_config,
    prepare_active_course_runtime,
    save_course_layout_config,
)
from bridge.ros2_publish_course_buoys import (  # noqa: E402
    _course_buoy_body_ids,
    _parse_buoy_float_name,
)
from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402
from tools.check_buoy_physics_contract import check_rake_release_and_rise  # noqa: E402


BASE_SCENE = ROOT / "scenes" / "tank_current_scene.xml"


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _floats(value: str | None) -> tuple[float, ...]:
    return tuple(float(part) for part in str(value or "").split())


def _require_close(actual: float, expected: float, label: str) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9):
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def _runtime(model: mujoco.MjModel, data: mujoco.MjData) -> CourseBuoyRuntime:
    def env_float(name: str, default: float) -> float:
        return 0.0 if name == "UUV_COURSE_BUOY_UPDATE_HZ" else float(default)

    def env_flag(name: str, default: bool) -> bool:
        return False if name == "UUV_COURSE_BUOY_TRACK_CSV_ENABLE" else bool(default)

    return CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=0.0,
        env_float=env_float,
        env_flag=env_flag,
        log=lambda _message: None,
    )


def _check_physical_yellow_buoy(root: ET.Element, model: mujoco.MjModel) -> None:
    prefix = TEST_TANK_YELLOW_BUOY_PREFIX
    float_name = f"{prefix}_float"
    composite = next(
        (
            element
            for element in root.findall(".//composite")
            if element.get("prefix") == f"{prefix}_flex_line_"
        ),
        None,
    )
    if composite is None:
        raise AssertionError("test-tank physical yellow buoy has no nylon cable")
    values = _floats(composite.get("vertex"))
    if len(values) < 6 or len(values) % 3:
        raise AssertionError(f"invalid test-tank nylon vertices: {values}")
    points = np.asarray(values, dtype=np.float64).reshape(-1, 3)
    cable_length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    _require_close(cable_length, TEST_TANK_NYLON_LENGTH_M, "test-tank nylon length")

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, float_name)
    magnet_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_magnet_base")
    cob_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{prefix}_cob_site")
    magnet_eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{prefix}_magnet_weld")
    collector_eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{prefix}_collector_weld")
    cable_bottom_eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{prefix}_flex_line_bottom_connect")
    cable_top_eq = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"{prefix}_flex_line_top_connect")
    if min(body_id, magnet_body_id, cob_site_id, magnet_eq, collector_eq, cable_bottom_eq, cable_top_eq) < 0:
        raise AssertionError("test-tank physical yellow buoy is missing body/site/equality contracts")
    published_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, candidate)
        for candidate in _course_buoy_body_ids(model)
    ]
    if published_names != [float_name]:
        raise AssertionError(f"test-tank ROS status discovery mismatch: {published_names}")
    if _parse_buoy_float_name(float_name) != ("test_tank", "yellow", 1):
        raise AssertionError(f"test-tank ROS status name parse mismatch: {float_name}")
    _require_close(float(model.body_mass[body_id]), 0.010, "test-tank yellow mass")
    if not np.allclose(model.body_ipos[body_id], [0.0, 0.0, -0.035], atol=1.0e-12, rtol=0.0):
        raise AssertionError(f"test-tank yellow CoM mismatch: {model.body_ipos[body_id]}")
    if not np.allclose(model.site_pos[cob_site_id], [0.0, 0.0, 0.035], atol=1.0e-12, rtol=0.0):
        raise AssertionError(f"test-tank yellow CoB mismatch: {model.site_pos[cob_site_id]}")
    if int(model.eq_active0[magnet_eq]) != 1 or int(model.eq_active0[collector_eq]) != 0:
        raise AssertionError("test-tank yellow magnet/collector initial equality state is wrong")

    model.opt.timestep = 0.008
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = _runtime(model, data)
    if len(runtime.buoys) != 1 or runtime.buoys[0].name != prefix:
        raise AssertionError(f"test-tank runtime did not discover exactly one physical buoy: {runtime.buoys}")
    if not math.isclose(runtime.contact_release_hold_s, 0.0, rel_tol=0.0, abs_tol=1.0e-12):
        raise AssertionError(f"test-tank rake contact is not immediate: {runtime.contact_release_hold_s}")
    buoy = runtime.buoys[0]
    data.time = 0.0
    if not runtime._contact_release_sustained(buoy, 0.1):
        raise AssertionError("test-tank rake contact did not release on the first sample")
    runtime._reset_contact_release_sample(buoy)
    data.time = 0.0
    qpos_before = np.asarray(data.qpos, dtype=np.float64).copy()
    qvel_before = np.asarray(data.qvel, dtype=np.float64).copy()
    data.xfrc_applied[body_id, 2] += 16.0
    runtime.apply(float(model.opt.timestep))
    data.xfrc_applied[body_id, 2] -= 16.0
    if not buoy.detached or int(data.eq_active[magnet_eq]) != 0 or int(data.eq_active[cable_top_eq]) != 0:
        raise AssertionError("test-tank yellow did not detach magnet and nylon top at 16 N")
    if int(data.eq_active[cable_bottom_eq]) != 1:
        raise AssertionError("test-tank nylon must remain anchored after float release")
    if float(np.max(np.abs(np.asarray(data.qpos) - qpos_before))) > 1.0e-6:
        raise AssertionError("test-tank yellow position jumped at release")
    if float(np.max(np.abs(np.asarray(data.qvel) - qvel_before))) > 1.0e-6:
        raise AssertionError("test-tank yellow velocity jumped at release")

    surface_target_z = runtime._surface_target_center_z(buoy)
    surface_deadline_s = 6.0
    surface_reached_s: float | None = None
    max_cob_z = float(data.site_xpos[cob_site_id, 2])
    for _ in range(int(math.ceil(surface_deadline_s / float(model.opt.timestep)))):
        runtime.apply(float(model.opt.timestep))
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise AssertionError("test-tank physical yellow buoy produced non-finite state")
        cob_z = float(data.site_xpos[cob_site_id, 2])
        max_cob_z = max(max_cob_z, cob_z)
        if surface_reached_s is None and cob_z >= surface_target_z - 0.01:
            surface_reached_s = float(data.time)
    final_cob_z = float(data.site_xpos[cob_site_id, 2])
    if surface_reached_s is None or final_cob_z < surface_target_z - 0.01:
        raise AssertionError(
            "released test-tank yellow did not settle at the surface by "
            f"{surface_deadline_s:.1f}s: z={final_cob_z:.3f} target={surface_target_z:.3f}"
        )
    if max_cob_z > surface_target_z + 0.005:
        raise AssertionError(
            "released test-tank yellow overshot the surface equilibrium: "
            f"max_z={max_cob_z:.3f} target={surface_target_z:.3f}"
        )

    rake_data = mujoco.MjData(model)
    mujoco.mj_forward(model, rake_data)
    rake_runtime = _runtime(model, rake_data)
    position_jump, velocity_jump, hold_s, peak_n, rise_m = check_rake_release_and_rise(
        mujoco,
        model,
        rake_data,
        rake_runtime,
        buoy_name=prefix,
    )
    if hold_s > float(model.opt.timestep) + 1.0e-9 or position_jump > 1.0e-6 or velocity_jump > 1.0e-6 or rise_m < 0.60:
        raise AssertionError(
            "test-tank yellow rake-release contract failed: "
            f"hold={hold_s:.3f}s qpos={position_jump:.3e} qvel={velocity_jump:.3e} rise={rise_m:.3f}m"
        )
    print(
        "test_tank_yellow "
        f"nylon={TEST_TANK_NYLON_LENGTH_M:.3f}m rake={peak_n:.3f}N/{hold_s:.3f}s "
        f"qpos_jump={position_jump:.3e} qvel_jump={velocity_jump:.3e} rise={rise_m:.3f}m "
        f"surface={surface_reached_s:.3f}s/{surface_target_z:.3f}m"
    )


def _check_collector_geometry(model: mujoco.MjModel) -> None:
    for name in ("collector_left_net_proxy", "collector_right_net_proxy"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0 or int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            raise AssertionError(f"test-tank {name} is not the trapezoidal rigid mesh panel")
        mesh_name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_MESH,
            int(model.geom_dataid[geom_id]),
        )
        if mesh_name != "front_open_buoy_collector_side_proxy_v1":
            raise AssertionError(f"test-tank {name} uses unexpected mesh {mesh_name}")
    roof_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "collector_top_net_proxy")
    if roof_id < 0:
        raise AssertionError("test-tank collector roof is missing")
    _require_close(float(model.geom_pos[roof_id][2]), 0.405, "test-tank collector roof z")
    expected_quat = np.asarray([0.99875, 0.0, -0.04994, 0.0], dtype=np.float64)
    expected_quat /= np.linalg.norm(expected_quat)
    if not np.allclose(model.geom_quat[roof_id], expected_quat, atol=1.0e-6, rtol=0.0):
        raise AssertionError(f"test-tank collector roof angle mismatch: {model.geom_quat[roof_id]}")


def main() -> int:
    source_digest = _digest(BASE_SCENE)
    with tempfile.TemporaryDirectory(prefix="test_tank_contract_", dir=ROOT) as temp_dir:
        temp = Path(temp_dir)
        config_path = temp / "course_layout.json"
        scene_path = temp / "test_tank_scene.xml"
        config_path.write_text(json.dumps(default_course_layout_config()), encoding="utf-8")
        save_course_layout_config(
            config_path,
            mode=COURSE_MODE_TEST_TANK,
            robot_xy=(-1.25, 0.15),
            yellow_xy=(0.35, 0.55),
            pinger_xy=(1.65, -0.60),
        )
        selection = prepare_active_course_runtime(
            config_path=config_path,
            competition_scene_path=BASE_SCENE,
            test_tank_scene_path=scene_path,
        )
        if selection.mode != COURSE_MODE_TEST_TANK or selection.scene_path != scene_path:
            raise AssertionError(f"wrong runtime selection: {selection}")
        if selection.pinger_site_name != TEST_TANK_PINGER_SITE_NAME:
            raise AssertionError(f"wrong pinger site: {selection.pinger_site_name}")

        root = ET.parse(scene_path).getroot()
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise AssertionError("generated scene has no worldbody")
        names = {str(element.get("name")) for element in worldbody.iter() if element.get("name")}
        for required in (
            "base_link",
            f"{TEST_TANK_YELLOW_BUOY_PREFIX}_magnet_base",
            f"{TEST_TANK_YELLOW_BUOY_PREFIX}_float",
            TEST_TANK_PINGER_ID,
            TEST_TANK_PINGER_SITE_NAME,
        ):
            if required not in names:
                raise AssertionError(f"generated scene missing {required}")
        leaked = sorted(
            name
            for name in names
            if name.startswith("course_buoy_") and not name.startswith(TEST_TANK_YELLOW_BUOY_PREFIX)
        )
        if leaked:
            raise AssertionError(f"competition buoy body leaked into test-tank scene: {leaked[:3]}")

        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pool_floor")
        wall_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pool_wall_px")
        wall_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pool_wall_py")
        pinger_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TEST_TANK_PINGER_SITE_NAME)
        if min(floor_id, wall_x_id, wall_y_id, pinger_site_id) < 0:
            raise AssertionError("generated model lacks required pool geom or pinger site")
        _require_close(float(model.geom_size[floor_id][0]) * 2.0, TEST_TANK_LENGTH_M, "tank length")
        _require_close(float(model.geom_size[floor_id][1]) * 2.0, TEST_TANK_WIDTH_M, "tank width")
        floor_top = float(model.geom_pos[floor_id][2] + model.geom_size[floor_id][2])
        _require_close(-floor_top, TEST_TANK_DEPTH_M, "tank depth")
        _require_close(float(data.site_xpos[pinger_site_id][2]), TEST_TANK_PINGER_Z_M, "pinger depth prior")
        _require_close(float(model.geom_pos[wall_x_id][0] - model.geom_size[wall_x_id][0]), TEST_TANK_LENGTH_M / 2.0, "inner +X wall")
        _require_close(float(model.geom_pos[wall_y_id][1] - model.geom_size[wall_y_id][1]), TEST_TANK_WIDTH_M / 2.0, "inner +Y wall")
        _check_collector_geometry(model)
        _check_physical_yellow_buoy(root, model)

        for _ in range(100):
            mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise AssertionError("test-tank model produced non-finite state")

    if _digest(BASE_SCENE) != source_digest:
        raise AssertionError("test-tank generation modified the competition scene")
    print("gui_test_tank_mode=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
