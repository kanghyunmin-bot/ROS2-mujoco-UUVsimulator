"""MuJoCo sensor and Ping360 setup for Ros2Bridge construction."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .ping360_sim import Ping360Config, Ping360Simulator
from .ros2_hydrophone_sim import configure_hydrophone_runtime
from .ros2_mujoco_model import lookup_ros2_mujoco_ids, site_id
from .ros2_ping360_messages import Ping360ImageRenderer
from .sitl_env import env_to_float


def configure_mujoco_sensor_runtime(
    bridge: Any,
    *,
    enable_ping360: bool,
    ping360_config_path: str,
    ping360_overrides: dict | None,
) -> None:
    """Resolve MuJoCo IDs and optional Ping360 runtime state."""

    mujoco_ids = lookup_ros2_mujoco_ids(bridge.model)
    bridge.sensor_ids = mujoco_ids.sensor_ids
    bridge._base_id = mujoco_ids.base_id
    bridge._imu_site_id = mujoco_ids.imu_site_id
    bridge._bar30_site_id = mujoco_ids.bar30_site_id
    bridge._bar30_depth_source = mujoco_ids.bar30_depth_source
    print(
        "[sensor] Bar30 source: "
        f"{bridge._bar30_depth_source}, water_surface_z={bridge._water_surface_z:.3f}m",
        flush=True,
    )
    bridge._dvl_site_id = mujoco_ids.dvl_site_id
    bridge._cam_left_site_id = mujoco_ids.cam_left_site_id
    bridge._cam_right_site_id = mujoco_ids.cam_right_site_id
    bridge._cam_top_site_id = mujoco_ids.cam_top_site_id

    ping360_default_config = Path(__file__).resolve().parents[1] / "config" / "ping360.json"
    bridge._ping360_config = Ping360Config.from_file(
        ping360_config_path or ping360_default_config,
        ping360_overrides,
    )
    if not enable_ping360:
        bridge._ping360_config.enabled = False
    bridge._ping360_site_id = site_id(bridge.model, bridge._ping360_config.site_name)
    bridge._ping360 = (
        Ping360Simulator(bridge.model, bridge._ping360_config)
        if bridge._ping360_config.enabled
        else None
    )
    bridge._ping360_image_renderer = Ping360ImageRenderer()
    bridge._ping360_status_period_s = float(
        np.clip(env_to_float("ROS2_UUV_PING360_STATUS_HZ", 2.0), 0.2, 20.0)
    )
    bridge._ping360_status_period_s = 1.0 / bridge._ping360_status_period_s
    bridge._ping360_status_next_t = 0.0
    bridge._ping360_publish_hz = float(
        np.clip(env_to_float("ROS2_UUV_PING360_PUBLISH_HZ", 10.0), 0.2, 30.0)
    )
    configure_hydrophone_runtime(bridge)


__all__ = ["configure_mujoco_sensor_runtime"]
