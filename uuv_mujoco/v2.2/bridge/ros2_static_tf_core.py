"""Static TF specs for fixed world/body frame aliases."""

from __future__ import annotations

import numpy as np

from .ros2_static_tf_types import TfSpec


def build_core_frame_specs(zero: np.ndarray, ident: np.ndarray, x180: np.ndarray) -> list[TfSpec]:
    return [
        ("map", "map_ned", zero, x180),
        ("odom", "odom_ned", zero, x180),
        ("base_link", "base_link_frd", zero, x180),
        ("base_link", "auv_link", zero, ident),
    ]


__all__ = ["build_core_frame_specs"]
