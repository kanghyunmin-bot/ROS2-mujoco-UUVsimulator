"""Timestamp acquisition for Ros2Bridge publish path."""

from __future__ import annotations

import math


def acquire_ros_stamp(self, sim_t: float):
    """Publish the authoritative MuJoCo clock and return the same ROS stamp.

    Building sensor stamps directly from ``data.time`` avoids a one-cycle
    race between publishing ``/clock`` and the bridge's own ROS clock
    subscription.  Every sensor snapshot and ``/clock`` therefore carries
    exactly the same simulation timestamp.
    """
    try:
        if not math.isfinite(sim_t) or sim_t < 0.0:
            raise ValueError(f"invalid MuJoCo simulation time: {sim_t!r}")
        total_nanoseconds = int(round(float(sim_t) * 1_000_000_000.0))
        clock_msg = self.Clock()
        clock_msg.clock.sec = total_nanoseconds // 1_000_000_000
        clock_msg.clock.nanosec = total_nanoseconds % 1_000_000_000
        if not self._safe_publish(self.pub_clock, clock_msg, "/clock"):
            return None
        if not self._safe_publish(
            self.pub_uuv_mujoco_clock, clock_msg, "/uuv_mujoco/clock"
        ):
            return None
        return clock_msg.clock
    except Exception as exc:
        if not self._ros_error_reported:
            self._ros_error_reported = True
            print(f"[ros2_bridge] timestamp acquisition failed: {exc}", flush=True)
        self._ros_ok = False
        return None


__all__ = ["acquire_ros_stamp"]
