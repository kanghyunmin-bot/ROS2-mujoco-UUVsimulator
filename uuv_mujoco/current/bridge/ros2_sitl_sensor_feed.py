"""SITL sensor-feed snapshot builder for Ros2Bridge.publish."""

from __future__ import annotations

import mujoco

from .ros2_sitl_sensor_kinematics import build_base_kinematic_state
from .ros2_sitl_sensor_transport import send_sitl_sensor_state
from .ros2_sitl_sensor_types import Ros2SensorSnapshot
from .ros2_sitl_sensor_vectors import build_imu_dvl_state
from .ros2_sitl_sensor_vertical import build_bar30_vertical_state


def build_and_send_sitl_sensor_snapshot(self, data: mujoco.MjData) -> Ros2SensorSnapshot | None:
    """Build one MuJoCo sensor snapshot and feed SITL JSON sensors when enabled."""

    base = build_base_kinematic_state(self, data)
    if base is None:
        return None

    imu_dvl = build_imu_dvl_state(self, data, base)
    vertical = build_bar30_vertical_state(self, data, base)
    send_sitl_sensor_state(self, base, imu_dvl, vertical)

    return Ros2SensorSnapshot(
        base_pos_enu=base.base_pos_enu,
        base_rot_enu=base.base_rot_enu,
        quat_base=base.quat_base,
        base_vel_enu=base.base_vel_enu,
        gyro_bmj=imu_dvl.gyro_bmj,
        acc_bmj=imu_dvl.acc_bmj,
        dvl_vel_body_bmj=imu_dvl.dvl_vel_body_bmj,
        dvl_altitude_m=imu_dvl.dvl_altitude_m,
        bar30_pressure_pa=vertical.bar30_pressure_pa,
        ros_depth_m=vertical.ros_depth_m,
    )


__all__ = ["Ros2SensorSnapshot", "build_and_send_sitl_sensor_snapshot"]
