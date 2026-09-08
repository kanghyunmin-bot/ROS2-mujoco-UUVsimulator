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
    delivery_driven_rate_hz = (
        0.0 if bool(getattr(self, "_dvl_sensor_model_enabled", False))
        else self._ros_rate_dvl_twist_hz
    )
    if dvl_vel_body_ros is not None:
        add_rate_limited(
            self.pub_dvl_velocity,
            "/dvl/velocity",
            builders["dvl_velocity"],
            delivery_driven_rate_hz,
        )
        add_rate_limited(
            self.pub_dvl_twist,
            "/dvl/twist",
            builders["dvl_twist"],
            delivery_driven_rate_hz,
        )
    if dvl_altitude_m is not None:
        add_rate_limited(
            self.pub_dvl_altitude,
            "/dvl/altitude",
            builders["dvl_altitude"],
            delivery_driven_rate_hz,
        )


def schedule_real_dvl_compat_jobs(
    self,
    jobs,
    add_rate_limited,
    *,
    builders: dict[str, object],
) -> None:
    if bool(getattr(self, "_dvl_sensor_model_enabled", False)):
        if self.pub_dvl_data is not None:
            for message in builders["dvl_data_batch"]():
                jobs.add(self.pub_dvl_data, "/dvl/data", message)
        if self.pub_dvl_position is not None:
            for message in builders["dvl_position_batch"]():
                jobs.add(self.pub_dvl_position, "/dvl/position", message)
        return
    data_rate_hz = self._ros_rate_dvl_twist_hz
    add_rate_limited(self.pub_dvl_data, "/dvl/data", builders["dvl_data"], data_rate_hz)
    position_rate_hz = self._ros_rate_dvl_position_hz
    add_rate_limited(
        self.pub_dvl_position,
        "/dvl/position",
        builders["dvl_position"],
        position_rate_hz,
    )


__all__ = ["schedule_dvl_ros_jobs", "schedule_real_dvl_compat_jobs"]
