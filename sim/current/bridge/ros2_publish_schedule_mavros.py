"""MAVROS-compatible ROS topic scheduling."""

from __future__ import annotations


def schedule_mavros_ros_jobs(self, jobs, add_rate_limited, sim_t: float, *, builders: dict[str, object]) -> None:
    if not self._mavros_surface_enabled:
        return
    _schedule_mavros_status_jobs(self, jobs, sim_t, builders=builders)
    _schedule_mavros_sensor_jobs(self, jobs, add_rate_limited, builders=builders)
    _schedule_mavros_local_position_jobs(self, add_rate_limited, builders=builders)
    _schedule_mavros_rc_jobs(self, jobs, add_rate_limited)


def _schedule_mavros_status_jobs(self, jobs, sim_t: float, *, builders: dict[str, object]) -> None:
    if self._mavros_surface_enabled and self._mavros_state_pub_hz > 0.0 and sim_t + 1e-9 >= self._mavros_state_next_t:
        self._mavros_state_next_t = sim_t + 1.0 / float(self._mavros_state_pub_hz)
        jobs.add(self.pub_mavros_state, "/mavros/state", builders["mavros_state"], on_demand=False)
    jobs.add(self.pub_mavros_vfr_hud, "/mavros/vfr_hud", builders["mavros_vfr_hud"], on_demand=True)


def _schedule_mavros_sensor_jobs(self, jobs, add_rate_limited, *, builders: dict[str, object]) -> None:
    add_rate_limited(
        self.pub_mavros_imu_data,
        "/mavros/imu/data",
        builders["mavros_imu"],
        self._ros_rate_mavros_imu_data_hz,
    )
    add_rate_limited(
        self.pub_mavros_imu_data_raw,
        "/mavros/imu/data_raw",
        builders["mavros_imu_raw"],
        self._ros_rate_mavros_imu_raw_hz,
    )
    add_rate_limited(
        self.pub_mavros_imu_static_pressure,
        "/mavros/imu/static_pressure",
        builders["mavros_static_pressure"],
        self._ros_rate_mavros_static_pressure_hz,
    )
    add_rate_limited(
        self.pub_mavros_imu_atm_pressure,
        "/mavros/imu/atm_pressure",
        builders["mavros_atm_pressure"],
        self._ros_rate_mavros_atm_pressure_hz,
    )
    jobs.add(self.pub_mavros_battery, "/mavros/battery", builders["mavros_battery"], on_demand=True)


def _schedule_mavros_local_position_jobs(self, add_rate_limited, *, builders: dict[str, object]) -> None:
    add_rate_limited(
        self.pub_mavros_local_pose,
        "/mavros/local_position/pose",
        builders["mavros_local_pose"],
        self._ros_rate_mavros_local_position_hz,
    )
    add_rate_limited(
        self.pub_mavros_local_vel,
        "/mavros/local_position/velocity_local",
        builders["mavros_local_vel"],
        self._ros_rate_mavros_local_position_hz,
    )
    add_rate_limited(
        self.pub_mavros_local_vel_body,
        "/mavros/local_position/velocity_body",
        builders["mavros_local_vel_body"],
        self._ros_rate_mavros_local_position_hz,
    )
    add_rate_limited(
        self.pub_mavros_local_vel_body_cov,
        "/mavros/local_position/velocity_body_cov",
        builders["mavros_local_vel_body_cov"],
        self._ros_rate_mavros_local_position_hz,
    )
    add_rate_limited(
        self.pub_mavros_vision_pose,
        "/mavros/vision_pose/pose",
        builders["mavros_vision_pose"],
        self._ros_rate_dvl_position_hz,
    )


def _schedule_mavros_rc_jobs(self, jobs, add_rate_limited) -> None:
    if self._mavros_last_rc_override is not None:
        jobs.add(self.pub_mavros_rc_in, "/mavros/rc/in", self._mavros_last_rc_override, on_demand=True)
    if self._mavros_last_rc_out is not None and self._mavros_rc_out_publish_mode == "rate_limited":
        add_rate_limited(
            self.pub_mavros_rc_out,
            "/mavros/rc/out",
            self._mavros_last_rc_out,
            self._ros_rate_mavros_rc_out_hz,
        )


__all__ = ["schedule_mavros_ros_jobs"]
