"""Pose/odometry value helpers for GUI telemetry callbacks."""

from __future__ import annotations

import math
import time
from typing import Any


def depth_topic_stale(snapshot: object, last_wall: dict[str, float], *, max_age_s: float = 1.0) -> bool:
    # No dedicated depth sample means pose-derived depth is immediately
    # eligible.  Using +inf here produced -inf age and froze depth forever.
    depth_age = time.monotonic() - last_wall.get("depth", -math.inf)
    return not math.isfinite(snapshot.depth_m) or depth_age > max_age_s


def depth_from_pose_z(z: float) -> float:
    return max(0.0, -float(z))


def pose_xyz(pose: Any) -> tuple[float, float, float]:
    position = pose.position
    return float(position.x), float(position.y), float(position.z)


def pose_msg_xyz(msg: Any) -> tuple[float, float, float]:
    return pose_xyz(msg.pose)


def odom_msg_xyz_velocity(msg: Any) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    pose = msg.pose.pose
    twist = msg.twist.twist
    velocity = (float(twist.linear.x), float(twist.linear.y), float(twist.linear.z))
    return pose_xyz(pose), velocity


__all__ = [
    "depth_from_pose_z",
    "depth_topic_stale",
    "odom_msg_xyz_velocity",
    "pose_msg_xyz",
    "pose_xyz",
]
