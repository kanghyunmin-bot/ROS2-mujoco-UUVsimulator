"""Static TF specs for onboard sensor frames."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ros2_static_tf_types import TfSpec
from .ros2_tf_geometry import site_local_pose


def build_sensor_frame_specs(
    *,
    model: Any,
    imu_site_id: int,
    bar30_site_id: int,
    dvl_site_id: int,
    ping360_site_id: int,
    hydrophone_site_id: int,
    ping360_frame_id: str,
    zero: np.ndarray,
    ident: np.ndarray,
    x180: np.ndarray,
) -> list[TfSpec]:
    imu_pos, imu_quat = site_local_pose(
        model,
        imu_site_id,
        fallback_pos=np.array([0.13135, 0.0, 0.08541], dtype=np.float64),
    )
    bar30_pos, bar30_quat = site_local_pose(
        model,
        bar30_site_id,
        fallback_pos=np.array([-0.17364, -0.03034, 0.0536], dtype=np.float64),
    )
    dvl_pos, dvl_site_quat = site_local_pose(
        model,
        dvl_site_id,
        fallback_pos=np.array([-0.00488, 0.0, -0.03910], dtype=np.float64),
        fallback_quat=x180,
    )
    ping360_pos, ping360_quat = site_local_pose(
        model,
        ping360_site_id,
        fallback_pos=np.array([0.0, 0.0, 0.205], dtype=np.float64),
        fallback_quat=ident,
    )
    hydrophone_pos, hydrophone_quat = site_local_pose(
        model,
        hydrophone_site_id,
        fallback_pos=np.array([0.2150, 0.0, -0.0350], dtype=np.float64),
        fallback_quat=ident,
    )
    return [
        ("base_link", "fcu_link", imu_pos, imu_quat),
        ("fcu_link", "imu_link", zero, ident),
        ("base_link", "bar30_link", bar30_pos, bar30_quat),
        ("base_link", "depth_link", bar30_pos, bar30_quat),
        ("base_link", "dvl_link", dvl_pos, dvl_site_quat),
        ("dvl_link", "dvl", zero, ident),
        ("base_link", ping360_frame_id, ping360_pos, ping360_quat),
        ("base_link", "hydrophone_link", hydrophone_pos, hydrophone_quat),
    ]


__all__ = ["build_sensor_frame_specs"]
