"""Model actuator, sensor, site, and video IO setup for the MuJoCo runner."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from .model_ids import SensorDataReader, keyed_object_ids, named_object_ids
from .qgc_video import QgcVideoRuntime


@dataclass
class RuntimeModelIoSetup:
    actuator_ids: dict[str, int]
    ctrlrange: Any
    thruster_params_path: Path
    yaw_names: list[str]
    horizontal_order: list[str]
    vertical_names: list[str]
    all_thruster_names: list[str]
    sensor_ids: dict[str, int]
    camera_ids: dict[str, int]
    qgc_video: QgcVideoRuntime
    sensor_site_ids: dict[str, int]
    sensor_value: Callable[[str], Any]


def create_runtime_model_io_setup(
    *,
    args,
    model,
    data,
    mujoco_module,
    config_dir: Path,
    ros_bridge,
    vertical_thrusters: list[str],
    yaw_thrusters: list[str],
    yaw_channel_order: list[str],
) -> RuntimeModelIoSetup:
    """Lookup stable MuJoCo actuator/sensor IDs and optional QGC video runtime."""

    actuator_ids = {
        mujoco_module.mj_id2name(model, mujoco_module.mjtObj.mjOBJ_ACTUATOR, i): i
        for i in range(model.nu)
    }
    ctrlrange = model.actuator_ctrlrange.copy()
    thruster_params_path = Path(config_dir) / "thruster_params.json"

    yaw_names = list(yaw_thrusters)
    horizontal_order = list(yaw_channel_order)
    vertical_names = list(vertical_thrusters)
    all_thruster_names = list(vertical_thrusters + yaw_thrusters)

    sensor_ids = named_object_ids(
        model,
        mujoco_module,
        mujoco_module.mjtObj.mjOBJ_SENSOR,
        ["imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude", "depth_pos"],
    )
    camera_ids = named_object_ids(
        model,
        mujoco_module,
        mujoco_module.mjtObj.mjOBJ_CAMERA,
        ("course_overview", "course_side", "stereo_left", "stereo_right", "top_up"),
    )

    qgc_video = QgcVideoRuntime.create(
        args=args,
        model=model,
        mujoco_module=mujoco_module,
        ros_bridge=ros_bridge,
        camera_ids=camera_ids,
    )

    sensor_site_ids = keyed_object_ids(
        model,
        mujoco_module,
        mujoco_module.mjtObj.mjOBJ_SITE,
        {
            "imu": "imu_site",
            "bar30": "bar30_site",
            "dvl": "dvl_site",
            "ping360": "ping360_site",
            "cam_left": "cam_left_site",
            "cam_right": "cam_right_site",
            "cam_top": "cam_top_site",
        },
    )
    sensor_value = SensorDataReader(model=model, data=data, sensor_ids=sensor_ids).value

    return RuntimeModelIoSetup(
        actuator_ids=actuator_ids,
        ctrlrange=ctrlrange,
        thruster_params_path=thruster_params_path,
        yaw_names=yaw_names,
        horizontal_order=horizontal_order,
        vertical_names=vertical_names,
        all_thruster_names=all_thruster_names,
        sensor_ids=sensor_ids,
        camera_ids=camera_ids,
        qgc_video=qgc_video,
        sensor_site_ids=sensor_site_ids,
        sensor_value=sensor_value,
    )


__all__ = ["RuntimeModelIoSetup", "create_runtime_model_io_setup"]
