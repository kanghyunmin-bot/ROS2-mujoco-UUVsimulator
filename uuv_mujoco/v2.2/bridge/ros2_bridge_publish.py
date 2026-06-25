"""`Ros2Bridge.publish` implementation."""

from __future__ import annotations

from .ros2_bridge_publish_ros import publish_ros_snapshot
from .ros2_bridge_publish_timing import publish_time_due
from .ros2_bridge_sitl_poll import poll_sitl_servo_if_enabled


def publish(self, data) -> None:
    sim_t = float(data.time)
    if not publish_time_due(self, sim_t):
        return

    poll_sitl_servo_if_enabled(self)
    snapshot = self._build_and_send_sitl_sensor_snapshot(data)
    if snapshot is None:
        return
    publish_ros_snapshot(self, data, sim_t, snapshot)


__all__ = ["publish"]
