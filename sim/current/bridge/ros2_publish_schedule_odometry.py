"""Odometry compatibility ROS topic scheduling."""

from __future__ import annotations


def schedule_odometry_ros_jobs(self, jobs, add_rate_limited, *, builders: dict[str, object]) -> None:
    if self._real_pkg_compat:
        jobs.add(self.pub_sim_odometry, "/sim/odom", builders["sim_odom"], on_demand=True)
        return
    add_rate_limited(self.pub_dvl_odometry, "/dvl/odometry", builders["odom_local"], self._ros_rate_dvl_position_hz)
    if self._mavros_surface_enabled:
        add_rate_limited(
            self.pub_mavros_local_odom,
            "/mavros/local_position/odom",
            builders["mavros_local_odom"],
            self._ros_rate_mavros_local_position_hz,
        )
    jobs.add(self.pub_rovio_odometry, "/rovio/odometry", builders["rovio_odom"], on_demand=True)
    jobs.add(self.pub_sim_odometry, "/sim/odom", builders["sim_odom"], on_demand=True)
    add_rate_limited(
        self.pub_odometry_filtered,
        "/odometry/filtered",
        builders["sim_odom"],
        self._ros_rate_dvl_position_hz,
    )


__all__ = ["schedule_odometry_ros_jobs"]
