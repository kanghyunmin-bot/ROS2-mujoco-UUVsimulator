"""ROS topic publish-queue runtime helpers for Ros2Bridge."""

from __future__ import annotations

import os
import time

from .ros2_bridge_runtime import PublishQueue
from .ros2_publish_builders import build_ros_publish_builders
from .ros2_publish_schedule import schedule_ros_publish_jobs
from .ros2_publish_state import prepare_ros_publish_state
from .ros2_sitl_sensor_feed import Ros2SensorSnapshot


def flush_ros_publish_jobs(self, data, stamp, snapshot: Ros2SensorSnapshot) -> bool:
    state = prepare_ros_publish_state(self, data, snapshot)
    sim_t = state.sim_t
    dvl_altitude_m = state.dvl_altitude_m
    dvl_vel_body_ros = state.dvl_vel_body_ros

    jobs = PublishQueue(self._publisher_demand, sim_t)

    def add_rate_limited(publisher, label: str, builder, hz: float, *, on_demand: bool = False) -> None:
        # Real-robot contract topics must publish deterministically at their
        # configured rate. Subscriber discovery/demand caching can lag during
        # replay startup and silently drop due samples if these are gated.
        if self._ros_topic_due(label, sim_t, hz):
            jobs.add(publisher, label, builder, on_demand=on_demand)

    builders = build_ros_publish_builders(self, data, stamp, state)

    schedule_ros_publish_jobs(
        self,
        jobs,
        add_rate_limited,
        sim_t,
        builders=builders,
        dvl_vel_body_ros=dvl_vel_body_ros,
        dvl_altitude_m=dvl_altitude_m,
    )

    _debug_publish_jobs(self, sim_t, jobs)
    if not jobs.flush(self._safe_publish):
        return

    return True


def _debug_publish_jobs(self, sim_t: float, jobs: PublishQueue) -> None:
    if os.environ.get("ROS2_UUV_PUBLISH_DEBUG", "0") != "1":
        return
    now = time.monotonic()
    next_wall = float(getattr(self, "_publish_jobs_debug_next_wall", 0.0))
    if now + 1.0e-9 < next_wall:
        return
    self._publish_jobs_debug_next_wall = now + 1.0
    labels = [str(label) for _, _, label in getattr(jobs, "_jobs", [])]
    preview = ",".join(labels[:8])
    if len(labels) > 8:
        preview += f",+{len(labels) - 8}"
    print(
        "[ros2_bridge_debug] "
        f"stage=jobs sim_t={sim_t:.3f} count={len(labels)} labels={preview or '-'}",
        flush=True,
    )
