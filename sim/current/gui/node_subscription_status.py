"""Status and Ping360 subscriptions for the GUI ROS node."""

from __future__ import annotations

from .runtime import String


def initialize_status_subscriptions(self) -> None:
    self._ping360_config_pub = self.create_publisher(String, "/ping360/config", 10)
    self.create_subscription(String, "/ping360/status", self._on_ping360_status, 10)
    self.create_subscription(String, "/mujoco/real_start_state/status", self._on_real_start_status, 10)
    self.create_subscription(
        String,
        "/uuv_mujoco/sitl/mavlink_telemetry_status",
        self._on_sitl_mavlink_telemetry_status,
        10,
    )


__all__ = ["initialize_status_subscriptions"]
