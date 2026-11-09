"""Depth and pressure subscriptions for the GUI ROS node."""

from __future__ import annotations

from geometry_msgs.msg import PoseWithCovarianceStamped

from .runtime import Float32


def initialize_depth_subscriptions(self, *, best_effort_qos) -> None:
    self.create_subscription(PoseWithCovarianceStamped, "/depth/pose", self._on_depth_pose, best_effort_qos)
    self.create_subscription(Float32, "/depth", self._on_depth, best_effort_qos)
    self.create_subscription(Float32, "/bar30/pressure_pa", self._on_bar30_pressure, best_effort_qos)


__all__ = ["initialize_depth_subscriptions"]
