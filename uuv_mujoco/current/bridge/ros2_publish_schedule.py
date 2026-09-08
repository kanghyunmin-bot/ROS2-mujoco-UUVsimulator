"""Publish-queue scheduling for the ROS2 bridge."""

from __future__ import annotations

from .ros2_publish_schedule_core import schedule_core_ros_jobs
from .ros2_publish_schedule_dvl import schedule_dvl_ros_jobs, schedule_real_dvl_compat_jobs
from .ros2_publish_schedule_mavros import schedule_mavros_ros_jobs
from .ros2_publish_schedule_odometry import schedule_odometry_ros_jobs
from .ros2_publish_schedule_ping360 import schedule_ping360_ros_jobs
from .ros2_hydrophone_sim import schedule_hydrophone_jobs
from .ros2_stereo_image import schedule_stereo_image_jobs


def schedule_ros_publish_jobs(
    self,
    jobs,
    add_rate_limited,
    sim_t: float,
    *,
    builders: dict[str, object],
    dvl_vel_body_ros,
    dvl_altitude_m,
) -> None:
    """Add all due ROS topic jobs without constructing unused messages."""

    schedule_core_ros_jobs(self, jobs, add_rate_limited, builders=builders)
    schedule_ping360_ros_jobs(self, jobs, add_rate_limited, builders=builders)
    schedule_hydrophone_jobs(self, jobs, add_rate_limited, builders=builders)
    schedule_stereo_image_jobs(self, jobs, add_rate_limited, builders=builders)
    if not self._real_pkg_compat:
        schedule_dvl_ros_jobs(
            self,
            add_rate_limited,
            builders=builders,
            dvl_vel_body_ros=dvl_vel_body_ros,
            dvl_altitude_m=dvl_altitude_m,
        )
    schedule_mavros_ros_jobs(self, jobs, add_rate_limited, sim_t, builders=builders)
    schedule_odometry_ros_jobs(self, jobs, add_rate_limited, builders=builders)
    schedule_real_dvl_compat_jobs(
        self,
        jobs,
        add_rate_limited,
        builders=builders,
    )
    if not self._real_pkg_compat:
        jobs.add(self.pub_tf, "/tf", builders["tf"], on_demand=True)


__all__ = ["schedule_ros_publish_jobs"]
