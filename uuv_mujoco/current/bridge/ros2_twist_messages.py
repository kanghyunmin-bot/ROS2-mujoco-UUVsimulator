"""Twist ROS2 message builders."""

from __future__ import annotations

from typing import Any

import numpy as np


def build_twist_msg(twist_stamped_type: type, stamp: Any, frame_id: str, linear: np.ndarray) -> Any:
    msg = twist_stamped_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.twist.linear.x = float(linear[0])
    msg.twist.linear.y = float(linear[1])
    msg.twist.linear.z = float(linear[2])
    return msg


def build_twist_cov_msg(
    twist_cov_type: type,
    stamp: Any,
    frame_id: str,
    linear: np.ndarray,
    *,
    angular: np.ndarray | None = None,
    linear_cov_diag: tuple[float, float, float] | None = None,
    angular_cov_diag: tuple[float, float, float] | None = None,
) -> Any:
    msg = twist_cov_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.twist.twist.linear.x = float(linear[0])
    msg.twist.twist.linear.y = float(linear[1])
    msg.twist.twist.linear.z = float(linear[2])
    ang = np.zeros(3, dtype=np.float64) if angular is None else np.asarray(angular, dtype=np.float64)
    msg.twist.twist.angular.x = float(ang[0])
    msg.twist.twist.angular.y = float(ang[1])
    msg.twist.twist.angular.z = float(ang[2])
    if linear_cov_diag is not None:
        msg.twist.covariance[0] = float(linear_cov_diag[0])
        msg.twist.covariance[7] = float(linear_cov_diag[1])
        msg.twist.covariance[14] = float(linear_cov_diag[2])
    if angular_cov_diag is not None:
        msg.twist.covariance[21] = float(angular_cov_diag[0])
        msg.twist.covariance[28] = float(angular_cov_diag[1])
        msg.twist.covariance[35] = float(angular_cov_diag[2])
    return msg


__all__ = ["build_twist_msg", "build_twist_cov_msg"]
