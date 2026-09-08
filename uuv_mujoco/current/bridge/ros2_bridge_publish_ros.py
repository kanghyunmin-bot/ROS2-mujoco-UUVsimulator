"""ROS publication steps for a prepared Ros2Bridge sensor snapshot."""

from __future__ import annotations

from .ros2_bridge_publish_stamp import acquire_ros_stamp
from .ros2_publish_state import prepare_ros_publish_state
from .ros2_publish_runtime import flush_sensor_packet_jobs


def publish_ros_snapshot(self, data, sim_t: float, snapshot, *, general_due: bool = True) -> None:
    # Device-protocol sensors are part of the simulated plant boundary, not a
    # ROS publisher side effect. Advance them even when direct ROS is disabled
    # or an unrelated ROS topic has failed.
    state = prepare_ros_publish_state(self, data, snapshot)
    if not self._enable_ros:
        return
    # A device-mode A50 driver uses /clock for its host-receipt headers. Keep
    # the authoritative clocks moving even after an optional ROS publisher
    # has tripped the general publication-health latch.
    stamp = acquire_ros_stamp(self, sim_t)
    if stamp is None or not self._ros_ok:
        return
    if not self._publish_static_context(stamp, sim_t):
        return
    if general_due:
        self._flush_ros_publish_jobs(data, stamp, state)
    else:
        # Deliver modeled packets on arrival, without waiting for a display or
        # telemetry tick and adding another unmodeled transport delay.
        flush_sensor_packet_jobs(self, stamp, state)


__all__ = ["publish_ros_snapshot"]
