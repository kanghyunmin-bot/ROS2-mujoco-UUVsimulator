"""ROS topic publish-queue runtime helpers for Ros2Bridge."""

from __future__ import annotations

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

    if not jobs.flush(self._safe_publish):
        return

    return True
