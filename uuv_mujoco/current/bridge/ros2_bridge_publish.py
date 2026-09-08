"""`Ros2Bridge.publish` implementation."""

from __future__ import annotations

import os
import time

from .ros2_bridge_publish_ros import publish_ros_snapshot
from .ros2_bridge_publish_timing import publish_time_due
from .ros2_bridge_sitl_poll import poll_sitl_servo_if_enabled


def publish(self, data) -> None:
    sim_t = float(data.time)
    if getattr(self, "enable_sitl", False):
        last = getattr(self, "_fcu_last_pub_t", -1.0)
        if sim_t < last:
            self.last_pub_t = -1.0
            last = -1.0
        period = 1.0 / float(os.environ.get("SITL_SCHED_LOOP_RATE", "400"))
        if last >= 0.0 and sim_t + 1e-9 < last + period:
            return
        self._fcu_last_pub_t = sim_t
        poll_sitl_servo_if_enabled(self)
        snapshot = self._build_and_send_sitl_sensor_snapshot(data)
        if snapshot is None:
            return
        general_due = publish_time_due(self, sim_t)
    else:
        if not publish_time_due(self, sim_t):
            return
        poll_sitl_servo_if_enabled(self)
        snapshot = self._build_and_send_sitl_sensor_snapshot(data)
        if snapshot is None:
            return
        general_due = True
    try:
        publish_ros_snapshot(self, data, sim_t, snapshot, general_due=general_due)
    except Exception as exc:
        # The JSON sensor feed above is part of the flight-control loop.  A
        # stale ROS message overlay or one optional topic must never tear that
        # loop down and leave ArduSub without sensors/heartbeat.  Keep the
        # bridge (and its SITL poll thread) alive while disabling only ROS
        # topic publication for the remainder of this run.
        self._ros_ok = False
        print(
            "[ros2] topic publish failed; ROS publishers disabled while "
            f"SITL sensor transport remains active: {exc}",
            flush=True,
        )


def _debug_publish_stage(self, sim_t: float, stage: str) -> None:
    if os.environ.get("ROS2_UUV_PUBLISH_DEBUG", "0") != "1":
        return
    now = time.monotonic()
    next_wall = float(getattr(self, "_publish_debug_next_wall", 0.0))
    if now + 1.0e-9 < next_wall:
        return
    self._publish_debug_next_wall = now + 1.0
    print(
        "[ros2_bridge_debug] "
        f"stage={stage} sim_t={sim_t:.3f} "
        f"last_pub_t={float(getattr(self, 'last_pub_t', -1.0)):.3f} "
        f"sensor_dt={float(getattr(self, 'sensor_dt', 0.0)):.3f} "
        f"enable_ros={int(bool(getattr(self, '_enable_ros', False)))} "
        f"ros_ok={int(bool(getattr(self, '_ros_ok', False)))}",
        flush=True,
    )


__all__ = ["publish"]
