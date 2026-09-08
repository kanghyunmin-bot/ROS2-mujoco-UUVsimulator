"""SITL JSON plant handoff for ROS2 bridge sensor snapshots.

The JSON backend requires one complete frame at the 400 Hz controller cadence,
so modeled IMU/Bar30 values are sample-and-held here.  In strict real-package
mode this function is deliberately not the public raw-sensor transport: the
delivery-driven ROS publishers consume the independent timed packet batches.
"""

from __future__ import annotations

import numpy as np

from .ros2_math import rotmat_to_quat_wxyz
from .ros2_sitl_sensor_types import Bar30VerticalState, BaseKinematicState, ImuDvlState


def send_sitl_sensor_state(
    self,
    base: BaseKinematicState,
    imu_dvl: ImuDvlState,
    vertical: Bar30VerticalState,
) -> None:
    """Send one mandatory FCU plant frame without creating a ROS capture."""
    if self._sitl_transport is None or imu_dvl.gyro_bmj is None or imu_dvl.acc_bmj is None:
        return

    gyro_frd = self._bmj_to_frd @ imu_dvl.gyro_bmj
    acc_frd = self._bmj_to_frd @ imu_dvl.acc_bmj
    rot_ned_bfrd = self._enu_to_ned @ base.base_rot_enu @ self._bmj_to_frd.T
    quat_ned_bfrd = rotmat_to_quat_wxyz(rot_ned_bfrd)
    rangefinder_distance_m = imu_dvl.dvl_altitude_m if self._sitl_dvl_rangefinder_enabled else None
    if bool(getattr(self._sitl_transport, "live_wall_external_nav", False)):
        self._sitl_transport.send_state(
            base.sim_t,
            gyro_frd,
            acc_frd,
            vertical.vertical_estimate,
            quat_ned_bfrd,
            rangefinder_distance_m=rangefinder_distance_m,
            pressure_pa=vertical.bar30_pressure_pa,
        )
    else:
        with self._sitl_transport_lock:
            self._sitl_transport.send_state(
                base.sim_t,
                gyro_frd,
                acc_frd,
                vertical.vertical_estimate,
                quat_ned_bfrd,
                rangefinder_distance_m=rangefinder_distance_m,
                pressure_pa=vertical.bar30_pressure_pa,
            )


__all__ = ["send_sitl_sensor_state"]
