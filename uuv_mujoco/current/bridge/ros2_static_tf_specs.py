"""Static ROS2 TF spec assembly facade for the MuJoCo vehicle model."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ros2_static_tf_cameras import build_camera_frame_specs
from .ros2_static_tf_core import build_core_frame_specs
from .ros2_static_tf_sensors import build_sensor_frame_specs
from .ros2_static_tf_types import TfSpec
from .ros2_tf_geometry import camera_optical_quat, quat_identity, quat_x_180


def build_static_tf_specs(
    *,
    model: Any,
    imu_site_id: int,
    bar30_site_id: int,
    dvl_site_id: int,
    ping360_site_id: int,
    hydrophone_site_id: int,
    ping360_frame_id: str,
    cam_left_site_id: int,
    cam_right_site_id: int,
) -> list[TfSpec]:
    zero = np.zeros(3, dtype=np.float64)
    ident = quat_identity()
    x180 = quat_x_180()
    specs = build_core_frame_specs(zero, ident, x180)
    specs.extend(
        build_sensor_frame_specs(
            model=model,
            imu_site_id=imu_site_id,
            bar30_site_id=bar30_site_id,
            dvl_site_id=dvl_site_id,
            ping360_site_id=ping360_site_id,
            hydrophone_site_id=hydrophone_site_id,
            ping360_frame_id=ping360_frame_id,
            zero=zero,
            ident=ident,
            x180=x180,
        )
    )
    specs.extend(
        build_camera_frame_specs(
            model=model,
            cam_left_site_id=cam_left_site_id,
            cam_right_site_id=cam_right_site_id,
            zero=zero,
            ident=ident,
            optical_quat=camera_optical_quat(),
        )
    )
    return specs


__all__ = [
    "TfSpec",
    "build_static_tf_specs",
]
