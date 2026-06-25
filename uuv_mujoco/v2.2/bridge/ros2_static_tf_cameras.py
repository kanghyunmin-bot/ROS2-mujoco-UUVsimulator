"""Static TF specs for stereo camera frames."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ros2_static_tf_types import TfSpec
from .ros2_tf_geometry import site_local_pose


def build_camera_frame_specs(
    *,
    model: Any,
    cam_left_site_id: int,
    cam_right_site_id: int,
    zero: np.ndarray,
    ident: np.ndarray,
    optical_quat: np.ndarray,
) -> list[TfSpec]:
    cam_left_pos, _ = site_local_pose(
        model,
        cam_left_site_id,
        fallback_pos=np.array([0.1493, -0.0225, -0.02], dtype=np.float64),
    )
    cam_right_pos, _ = site_local_pose(
        model,
        cam_right_site_id,
        fallback_pos=np.array([0.1493, 0.0225, -0.02], dtype=np.float64),
    )
    camera_link_pos = 0.5 * (cam_left_pos + cam_right_pos)
    return [
        ("base_link", "camera_link", camera_link_pos, ident),
        ("camera_link", "stereo_left", cam_left_pos - camera_link_pos, ident),
        ("camera_link", "stereo_right", cam_right_pos - camera_link_pos, ident),
        ("stereo_left", "stereo_left_optical", zero, optical_quat),
        ("stereo_right", "stereo_right_optical", zero, optical_quat),
    ]


__all__ = ["build_camera_frame_specs"]
