"""Odometry and TF ROS message builders for publish jobs."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .ros2_publish_odometry_messages import (
    build_local_odom_msg,
    build_rovio_odom_msg,
    build_sim_odom_msg,
)
from .ros2_publish_odometry_tf import build_odometry_tf_msg, rovio_orientation_quat

if TYPE_CHECKING:
    from .ros2_publish_state import RosPublishState


def _lazy_message(cache: dict[str, object], key: str, factory):
    def get_message():
        if key not in cache:
            cache[key] = factory()
        return cache[key]

    return get_message


def build_odometry_publish_builders(self, stamp, state: "RosPublishState") -> dict[str, object]:
    zero_vel = np.zeros(3, dtype=np.float64)
    cache: dict[str, object] = {}
    quat_rovio = rovio_orientation_quat(self, state)

    return {
        "odom_local": _lazy_message(cache, "odom_local", lambda: build_local_odom_msg(self, stamp, state, zero_vel)),
        "rovio_odom": _lazy_message(
            cache,
            "rovio_odom",
            lambda: build_rovio_odom_msg(self, stamp, state, quat_rovio, zero_vel),
        ),
        "sim_odom": _lazy_message(cache, "sim_odom", lambda: build_sim_odom_msg(self, stamp, state)),
        "tf": _lazy_message(cache, "tf", lambda: build_odometry_tf_msg(self, stamp, state)),
    }
