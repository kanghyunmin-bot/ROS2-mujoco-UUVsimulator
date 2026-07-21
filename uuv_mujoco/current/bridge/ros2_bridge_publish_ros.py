"""ROS publication steps for a prepared Ros2Bridge sensor snapshot."""

from __future__ import annotations

from .ros2_bridge_publish_stamp import acquire_ros_stamp


def publish_ros_snapshot(self, data, sim_t: float, snapshot) -> None:
    if not self._enable_ros or not self._ros_ok:
        return
    stamp = acquire_ros_stamp(self, sim_t)
    if stamp is None:
        return
    if not self._publish_static_context(stamp, sim_t):
        return
    self._flush_ros_publish_jobs(data, stamp, snapshot)


__all__ = ["publish_ros_snapshot"]
