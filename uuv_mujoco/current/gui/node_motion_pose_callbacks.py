"""Pose and odometry callbacks for UuvGuiNode."""

from __future__ import annotations

import math
import time

from .node_motion_pose_state import depth_from_pose_z, depth_topic_stale, odom_msg_xyz_velocity, pose_msg_xyz


def _depth_topic_stale(self) -> bool:
    return depth_topic_stale(self._snapshot, self._last_wall)


def _on_pose(self, msg: PoseStamped) -> None:
    self._touch("pose")
    x, y, z = pose_msg_xyz(msg)
    with self._lock:
        self._snapshot.position_xyz = (x, y, z)
        if _depth_topic_stale(self):
            self._snapshot.depth_m = depth_from_pose_z(z)
            self._snapshot.depth_source = "local_position.pose.z"


def _on_odom(self, msg: Odometry, source: str) -> None:
    now = time.monotonic()
    if source != "/sim/odom" and now - self._last_wall.get("sim_pose", -math.inf) < 0.5:
        return
    if (
        source not in {"/sim/odom", "/odometry/filtered"}
        and now - self._last_wall.get("filtered_pose", -math.inf) < 0.5
    ):
        return
    if source == "/sim/odom":
        self._touch("sim_pose")
    elif source == "/odometry/filtered":
        self._touch("filtered_pose")
    self._touch("pose")
    position_xyz, velocity_xyz = odom_msg_xyz_velocity(msg)
    with self._lock:
        self._snapshot.position_xyz = position_xyz
        self._snapshot.velocity_xyz = velocity_xyz
        self._snapshot.velocity_source = source
        if _depth_topic_stale(self):
            self._snapshot.depth_m = depth_from_pose_z(position_xyz[2])
            self._snapshot.depth_source = f"{source}.pose.z"


def _on_local_odom(self, msg: Odometry) -> None:
    self._on_odom(msg, self._topic("local_position/odom"))


def _on_filtered_odom(self, msg: Odometry) -> None:
    self._on_odom(msg, "/odometry/filtered")


def _on_rovio_odom(self, msg: Odometry) -> None:
    self._on_odom(msg, "/rovio/odometry")


def _on_dvl_odom(self, msg: Odometry) -> None:
    self._on_odom(msg, "/dvl/odometry")


def _on_ground_truth_pose(self, msg: PoseStamped) -> None:
    self._touch("pose")
    x, y, z = pose_msg_xyz(msg)
    with self._lock:
        self._snapshot.position_xyz = (x, y, z)
        if _depth_topic_stale(self):
            self._snapshot.depth_m = depth_from_pose_z(z)
            self._snapshot.depth_source = "/mujoco/ground_truth/pose.z"


__all__ = [
    "_on_dvl_odom",
    "_on_filtered_odom",
    "_on_ground_truth_pose",
    "_on_local_odom",
    "_on_odom",
    "_on_pose",
    "_on_rovio_odom",
]
