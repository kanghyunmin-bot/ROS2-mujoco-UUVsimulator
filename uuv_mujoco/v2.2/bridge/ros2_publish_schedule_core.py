"""Core ROS topic scheduling."""

from __future__ import annotations


def schedule_core_ros_jobs(self, jobs, add_rate_limited, *, builders: dict[str, object]) -> None:
    add_rate_limited(self.pub_imu, "/imu/data", builders["imu"], self._ros_rate_core_imu_hz)
    add_rate_limited(self.pub_depth, "/depth", builders["depth"], self._ros_rate_depth_hz)
    add_rate_limited(self.pub_depth_pose, "/depth/pose", builders["depth_pose"], self._ros_rate_depth_hz)
    add_rate_limited(self.pub_bar30_pressure, "/bar30/pressure_pa", builders["baro"], self._ros_rate_bar30_hz)
    jobs.add(self.pub_mujoco_sim_time, "/mujoco/sim_time", builders["sim_time"], on_demand=True)
    add_rate_limited(
        self.pub_sitl_sensor_replay_status,
        "/uuv_mujoco/sitl/sensor_replay_status",
        builders["sitl_sensor_replay_status"],
        10.0,
    )
    add_rate_limited(
        self.pub_sitl_mavlink_telemetry_status,
        "/uuv_mujoco/sitl/mavlink_telemetry_status",
        builders["sitl_mavlink_telemetry_status"],
        10.0,
    )
    jobs.add(self.pub_ground_truth, "/mujoco/ground_truth/pose", builders["ground_truth"], on_demand=True)


__all__ = ["schedule_core_ros_jobs"]
