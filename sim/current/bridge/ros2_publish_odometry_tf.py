"""Odometry TF construction helpers for ROS publish jobs."""

from __future__ import annotations

from .ros2_math import rotmat_to_quat_wxyz
from .ros2_tf_messages import build_tf_message, quat_identity


def rovio_orientation_quat(bridge, state):
    return rotmat_to_quat_wxyz(state.rot_world_body @ bridge._base_to_rovio)


def odometry_tf_specs(bridge, state):
    map_to_odom = state.base_pos_enu - bridge._odom_pos
    return [
        ("map", "odom", map_to_odom, quat_identity()),
        ("odom", "base_link", bridge._odom_pos, state.quat_ros),
    ]


def build_odometry_tf_msg(bridge, stamp, state):
    return build_tf_message(
        bridge.TFMessage,
        bridge.TransformStamped,
        stamp,
        odometry_tf_specs(bridge, state),
    )


__all__ = ["build_odometry_tf_msg", "odometry_tf_specs", "rovio_orientation_quat"]
