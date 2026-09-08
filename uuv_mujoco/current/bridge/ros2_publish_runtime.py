"""ROS topic publish-queue runtime helpers for Ros2Bridge."""

from __future__ import annotations

import os
import time

from .ros2_bridge_runtime import PublishQueue
from .ros2_publish_builders import build_ros_publish_builders
from .ros2_publish_schedule import schedule_ros_publish_jobs
from .ros2_publish_state import RosPublishState
from .ros2_publish_builder_core import build_core_publish_builders
from .ros2_publish_builder_mavros import build_mavros_publish_builders


def flush_sensor_packet_jobs(self, stamp, state: RosPublishState) -> bool:
    """Publish modeled IMU and pressure arrivals between telemetry ticks."""
    if not state.imu_sensor_deliveries and not state.bar30_sensor_deliveries:
        return True
    jobs = PublishQueue(self._publisher_demand, state.sim_t)
    builders = build_core_publish_builders(self, stamp, state)
    builders.update(build_mavros_publish_builders(self, stamp, state))
    entries = []
    if not self._real_pkg_compat:
        if self._imu_sensor_model_enabled:
            entries.append((self.pub_imu, "/imu/data", "imu_batch"))
        if self._bar30_sensor_model_enabled:
            entries.extend((
                (self.pub_depth, "/depth", "depth_batch"),
                (self.pub_depth_pose, "/depth/pose", "depth_pose_batch"),
                (self.pub_bar30_pressure, "/bar30/pressure_pa", "baro_batch"),
            ))
    if self._mavros_surface_enabled or getattr(self, "_strict_sitl_sensor_transport", False):
        if self._imu_sensor_model_enabled:
            if self._mavros_surface_enabled:
                entries.append((self.pub_mavros_imu_data, "/mavros/imu/data", "mavros_imu_batch"))
            entries.append((self.pub_mavros_imu_data_raw, "/mavros/imu/data_raw", "mavros_imu_raw_batch"))
        if self._bar30_sensor_model_enabled and self._static_pressure_source == "external":
            entries.append((self.pub_mavros_imu_static_pressure, "/mavros/imu/static_pressure",
                            "mavros_static_pressure_batch"))
    for publisher, label, key in entries:
        for message in builders[key]():
            jobs.add(publisher, label, message)
    return jobs.flush(self._safe_publish)


def flush_ros_publish_jobs(self, data, stamp, state: RosPublishState) -> bool:
    sim_t = state.sim_t
    dvl_altitude_m = state.dvl_altitude_m
    dvl_vel_body_ros = state.dvl_vel_body_ros

    jobs = PublishQueue(self._publisher_demand, sim_t)

    def add_rate_limited(
        publisher,
        label: str,
        builder,
        hz: float,
        *,
        on_demand: bool = False,
    ) -> None:
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
