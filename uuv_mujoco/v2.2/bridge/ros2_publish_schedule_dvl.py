"""DVL ROS topic scheduling."""

from __future__ import annotations


def schedule_dvl_ros_jobs(
    self,
    add_rate_limited,
    *,
    builders: dict[str, object],
    dvl_vel_body_ros,
    dvl_altitude_m,
) -> None:
    if dvl_vel_body_ros is not None:
        add_rate_limited(self.pub_dvl_velocity, "/dvl/velocity", builders["dvl_velocity"], self._ros_rate_dvl_twist_hz)
        add_rate_limited(self.pub_dvl_twist, "/dvl/twist", builders["dvl_twist"], self._ros_rate_dvl_twist_hz)
    if dvl_altitude_m is not None:
        add_rate_limited(self.pub_dvl_altitude, "/dvl/altitude", builders["dvl_altitude"], self._ros_rate_dvl_twist_hz)


def schedule_real_dvl_compat_jobs(self, add_rate_limited, *, builders: dict[str, object]) -> None:
    add_rate_limited(self.pub_dvl_data, "/dvl/data", builders["dvl_data"], self._ros_rate_dvl_twist_hz)
    add_rate_limited(self.pub_dvl_position, "/dvl/position", builders["dvl_position"], self._ros_rate_dvl_position_hz)


__all__ = ["schedule_dvl_ros_jobs", "schedule_real_dvl_compat_jobs"]
