"""MuJoCo model lookup helpers for the ROS2 bridge.

Keep object/sensor name contracts in one place so OS-specific runtime issues
stay separate from the bridge publish loop.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


ROS2_SENSOR_NAMES = (
    "imu_quat",
    "imu_gyro",
    "imu_acc",
    "dvl_vel_body",
    "dvl_altitude",
    "depth_pos",
)


@dataclass(frozen=True)
class Ros2MujocoIds:
    sensor_ids: dict[str, int]
    base_id: int
    imu_site_id: int
    bar30_site_id: int
    dvl_site_id: int
    cam_left_site_id: int
    cam_right_site_id: int
    bar30_depth_source: str


def _object_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    return int(mujoco.mj_name2id(model, obj_type, name))


def site_id(model: mujoco.MjModel, name: str) -> int:
    return _object_id(model, mujoco.mjtObj.mjOBJ_SITE, name)


def lookup_ros2_mujoco_ids(model: mujoco.MjModel) -> Ros2MujocoIds:
    sensor_ids = {
        name: sid
        for name in ROS2_SENSOR_NAMES
        if (sid := _object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)) >= 0
    }
    bar30_site_id = site_id(model, "bar30_site")
    if "depth_pos" in sensor_ids:
        bar30_depth_source = "depth_pos"
    elif bar30_site_id >= 0:
        bar30_depth_source = "bar30_site"
    else:
        bar30_depth_source = "base_link_fallback"
    return Ros2MujocoIds(
        sensor_ids=sensor_ids,
        base_id=_object_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link"),
        imu_site_id=site_id(model, "imu_site"),
        bar30_site_id=bar30_site_id,
        dvl_site_id=site_id(model, "dvl_site"),
        cam_left_site_id=site_id(model, "cam_left_site"),
        cam_right_site_id=site_id(model, "cam_right_site"),
        bar30_depth_source=bar30_depth_source,
    )


def sensor_slice(
    model: mujoco.MjModel,
    sensor_ids: dict[str, int],
    name: str,
    data: mujoco.MjData,
) -> np.ndarray | None:
    sid = sensor_ids.get(name, -1)
    if sid < 0:
        return None
    adr = int(model.sensor_adr[sid])
    dim = int(model.sensor_dim[sid])
    return np.array(data.sensordata[adr : adr + dim], dtype=np.float64)
