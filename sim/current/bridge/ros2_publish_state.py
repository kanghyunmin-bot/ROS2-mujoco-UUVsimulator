"""Derived state preparation for ROS publish jobs."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ros2_math import quat_wxyz_to_rotmat, rotmat_to_quat_wxyz
from .ros2_sitl_sensor_feed import Ros2SensorSnapshot


@dataclass(frozen=True)
class RosPublishState:
    sim_t: float
    base_pos_enu: np.ndarray
    base_rot_enu: np.ndarray
    quat_base: np.ndarray
    base_vel_enu: np.ndarray
    gyro_bmj: np.ndarray | None
    acc_bmj: np.ndarray | None
    dvl_vel_body_bmj: np.ndarray | None
    dvl_altitude_m: float | None
    bar30_pressure_pa: float
    ros_depth_m: float
    quat_ros: np.ndarray
    base_vel_body_ros: np.ndarray
    gyro_ros: np.ndarray
    acc_ros: np.ndarray
    acc_ros_surface: np.ndarray
    dvl_vel_body_ros: np.ndarray | None
    dvl_vel_body_frd: np.ndarray | None
    dvl_vel_dvl_frd: np.ndarray | None
    static_pressure_pa: float
    rot_world_body: np.ndarray


def prepare_ros_publish_state(self, data, snapshot: Ros2SensorSnapshot) -> RosPublishState:
    sim_t = float(data.time)
    base_pos_enu = snapshot.base_pos_enu
    base_rot_enu = snapshot.base_rot_enu
    quat_base = snapshot.quat_base
    base_vel_enu = snapshot.base_vel_enu
    gyro_bmj = snapshot.gyro_bmj
    acc_bmj = snapshot.acc_bmj
    dvl_vel_body_bmj = snapshot.dvl_vel_body_bmj
    dvl_altitude_m = snapshot.dvl_altitude_m
    bar30_pressure_pa = snapshot.bar30_pressure_pa
    ros_depth_m = snapshot.ros_depth_m

    quat_ros = rotmat_to_quat_wxyz(base_rot_enu @ self._bmj_to_flu.T)
    base_vel_body_ros = self._bmj_to_flu @ (base_rot_enu.T @ base_vel_enu)
    gyro_ros = self._bmj_to_flu @ gyro_bmj if gyro_bmj is not None else np.zeros(3, dtype=np.float64)
    acc_ros = self._bmj_to_flu @ acc_bmj if acc_bmj is not None else np.zeros(3, dtype=np.float64)
    acc_ros_surface = self._ros_imu_accel_surface(acc_ros)
    dvl_vel_body_ros = self._bmj_to_flu @ dvl_vel_body_bmj if dvl_vel_body_bmj is not None else None
    dvl_vel_body_frd = self._bmj_to_frd @ dvl_vel_body_bmj if dvl_vel_body_bmj is not None else None
    dvl_vel_dvl_frd = self._dvl_body_frd_to_dvl_frd @ dvl_vel_body_frd if dvl_vel_body_frd is not None else None

    rot_world_body = quat_wxyz_to_rotmat(quat_base)
    if dvl_vel_body_bmj is not None:
        dt = self.sensor_dt if self._last_odom_time < 0.0 else float(np.clip(sim_t - self._last_odom_time, 1e-4, 0.2))
        self._last_odom_time = sim_t
        vel_world = rot_world_body @ dvl_vel_body_bmj
        self._odom_pos += vel_world * dt
    else:
        self._last_odom_time = -1.0

    self._apply_mavros_setpoint(base_pos_enu, base_rot_enu)

    # Decide what static_pressure should emulate.
    if self._static_pressure_source == "external":
        static_pressure_pa = bar30_pressure_pa
    else:
        static_pressure_pa = self._internal_pressure_pa

    return RosPublishState(
        sim_t=sim_t,
        base_pos_enu=base_pos_enu,
        base_rot_enu=base_rot_enu,
        quat_base=quat_base,
        base_vel_enu=base_vel_enu,
        gyro_bmj=gyro_bmj,
        acc_bmj=acc_bmj,
        dvl_vel_body_bmj=dvl_vel_body_bmj,
        dvl_altitude_m=dvl_altitude_m,
        bar30_pressure_pa=bar30_pressure_pa,
        ros_depth_m=ros_depth_m,
        quat_ros=quat_ros,
        base_vel_body_ros=base_vel_body_ros,
        gyro_ros=gyro_ros,
        acc_ros=acc_ros,
        acc_ros_surface=acc_ros_surface,
        dvl_vel_body_ros=dvl_vel_body_ros,
        dvl_vel_body_frd=dvl_vel_body_frd,
        dvl_vel_dvl_frd=dvl_vel_dvl_frd,
        static_pressure_pa=float(static_pressure_pa),
        rot_world_body=rot_world_body,
    )
