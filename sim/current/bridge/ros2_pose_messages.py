"""Pose and odometry ROS2 message builders."""

from __future__ import annotations

from typing import Any

import numpy as np


def build_pose_msg(pose_stamped_type: type, stamp: Any, frame_id: str, pos: np.ndarray, quat: np.ndarray) -> Any:
    msg = pose_stamped_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pos[0])
    msg.pose.position.y = float(pos[1])
    msg.pose.position.z = float(pos[2])
    msg.pose.orientation.w = float(quat[0])
    msg.pose.orientation.x = float(quat[1])
    msg.pose.orientation.y = float(quat[2])
    msg.pose.orientation.z = float(quat[3])
    return msg


def build_depth_pose_msg(pose_cov_type: type, stamp: Any, depth_m: float) -> Any:
    msg = pose_cov_type()
    msg.header.stamp = stamp
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    # Real depth node uses z-up odom convention; positive depth is -z.
    msg.pose.pose.position.z = -float(max(0.0, depth_m))
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.0
    msg.pose.covariance[7] = 0.0
    msg.pose.covariance[14] = 0.05
    return msg


def build_odom_msg(
    odometry_type: type,
    stamp: Any,
    frame_id: str,
    child_frame: str,
    pos: np.ndarray,
    quat: np.ndarray,
    vel: np.ndarray,
    *,
    angular_vel: np.ndarray | None = None,
) -> Any:
    msg = odometry_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame
    msg.pose.pose.position.x = float(pos[0])
    msg.pose.pose.position.y = float(pos[1])
    msg.pose.pose.position.z = float(pos[2])
    msg.pose.pose.orientation.w = float(quat[0])
    msg.pose.pose.orientation.x = float(quat[1])
    msg.pose.pose.orientation.y = float(quat[2])
    msg.pose.pose.orientation.z = float(quat[3])
    msg.twist.twist.linear.x = float(vel[0])
    msg.twist.twist.linear.y = float(vel[1])
    msg.twist.twist.linear.z = float(vel[2])
    if angular_vel is not None:
        msg.twist.twist.angular.x = float(angular_vel[0])
        msg.twist.twist.angular.y = float(angular_vel[1])
        msg.twist.twist.angular.z = float(angular_vel[2])
    return msg


__all__ = ["build_pose_msg", "build_depth_pose_msg", "build_odom_msg"]
