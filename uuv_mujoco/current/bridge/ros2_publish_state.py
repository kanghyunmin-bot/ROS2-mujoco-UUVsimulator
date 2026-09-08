"""Derived state preparation for ROS publish jobs."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .ros2_dvl_sensor_runtime import advance_dvl_sensor_runtime
from .ros2_dvl_beam_geometry import dvl_beam_geometry_if_capture_due
from .ros2_dvl_device_emulator_runtime import (
    forward_dvl_position_to_device_emulator,
    forward_dvl_delivery_to_device_emulator,
    synchronize_dvl_device_commands,
)
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
    dvl_beam_ranges_m: tuple[float | None, ...] | None
    dvl_beam_incidence_cosines: tuple[float, ...] | None
    static_pressure_pa: float
    rot_world_body: np.ndarray
    dvl_sensor_delivery: object | None = None
    dvl_position_delivery: object | None = None
    imu_sensor_delivery: object | None = None
    bar30_sensor_delivery: object | None = None
    imu_sensor_deliveries: tuple[object, ...] = ()
    bar30_sensor_deliveries: tuple[object, ...] = ()


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
    gyro_ros = (
        self._bmj_to_flu @ gyro_bmj
        if gyro_bmj is not None
        else np.zeros(3, dtype=np.float64)
    )
    acc_ros = self._bmj_to_flu @ acc_bmj if acc_bmj is not None else np.zeros(3, dtype=np.float64)
    acc_ros_surface = self._ros_imu_accel_surface(acc_ros)
    dvl_vel_body_ros = (
        self._bmj_to_flu @ dvl_vel_body_bmj
        if dvl_vel_body_bmj is not None
        else None
    )
    dvl_vel_body_frd = (
        self._bmj_to_frd @ dvl_vel_body_bmj
        if dvl_vel_body_bmj is not None
        else None
    )
    dvl_vel_dvl_frd = (
        self._dvl_body_frd_to_dvl_frd @ dvl_vel_body_frd
        if dvl_vel_body_frd is not None
        else None
    )
    dvl_beam_ranges_m, dvl_beam_incidence_cosines = dvl_beam_geometry_if_capture_due(
        self,
        data,
        sim_t,
    )

    rot_world_body = quat_wxyz_to_rotmat(quat_base)
    if not bool(getattr(self, "_dvl_sensor_model_enabled", False)):
        # Preserve the original ideal-DVL behavior when the explicit A50
        # model is disabled.  The modeled path integrates only delivered,
        # measured packets inside ``advance_dvl_sensor_runtime``.
        if dvl_vel_body_bmj is not None:
            dt = (
                self.sensor_dt
                if self._last_odom_time < 0.0
                else float(np.clip(sim_t - self._last_odom_time, 1.0e-4, 0.2))
            )
            self._last_odom_time = sim_t
            self._odom_pos += (rot_world_body @ dvl_vel_body_bmj) * dt
        else:
            self._last_odom_time = -1.0

    self._apply_mavros_setpoint(base_pos_enu, base_rot_enu)

    # Decide what static_pressure should emulate.
    if self._static_pressure_source == "external":
        static_pressure_pa = bar30_pressure_pa
    else:
        static_pressure_pa = self._internal_pressure_pa

    state = RosPublishState(
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
        dvl_beam_ranges_m=dvl_beam_ranges_m,
        dvl_beam_incidence_cosines=dvl_beam_incidence_cosines,
        static_pressure_pa=float(static_pressure_pa),
        rot_world_body=rot_world_body,
        imu_sensor_delivery=(
            snapshot.imu_sensor_deliveries[-1]
            if snapshot.imu_sensor_deliveries
            else None
        ),
        bar30_sensor_delivery=(
            snapshot.bar30_sensor_deliveries[-1]
            if snapshot.bar30_sensor_deliveries
            else None
        ),
        imu_sensor_deliveries=tuple(snapshot.imu_sensor_deliveries),
        bar30_sensor_deliveries=tuple(snapshot.bar30_sensor_deliveries),
    )
    synchronize_dvl_device_commands(
        self,
        sim_t=sim_t,
        rot_world_body=rot_world_body,
    )
    delivery = advance_dvl_sensor_runtime(self, state)
    _forward_dvl_device_deliveries(self)
    position_deliveries = getattr(
        self,
        "_dvl_sensor_new_position_deliveries",
        (),
    )
    return replace(
        state,
        dvl_sensor_delivery=delivery,
        dvl_position_delivery=(
            position_deliveries[-1] if position_deliveries else None
        ),
    )


def _forward_dvl_device_deliveries(bridge) -> None:
    """Forward independent DVL streams in simulated wire-arrival order."""

    arrivals = [
        (float(delivery.arrival_time_s), 0, index, delivery)
        for index, delivery in enumerate(
            getattr(bridge, "_dvl_sensor_new_deliveries", ())
        )
    ]
    arrivals.extend(
        (float(delivery.arrival_time_s), 1, index, delivery)
        for index, delivery in enumerate(
            getattr(bridge, "_dvl_sensor_new_position_deliveries", ())
        )
    )
    for _, stream, _, delivery in sorted(arrivals):
        if stream == 0:
            forward_dvl_delivery_to_device_emulator(bridge, delivery)
        else:
            forward_dvl_position_to_device_emulator(bridge, delivery)
