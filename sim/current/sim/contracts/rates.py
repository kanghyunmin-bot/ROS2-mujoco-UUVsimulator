"""Real-robot surface rate contract."""

from __future__ import annotations


# April 1 real-robot rosbag native telemetry surface rates.  These are not the
# simulator integration rates; the ROS/MAVROS compatibility layer must publish
# each topic at its observed real surface cadence and hold values in between.
REAL_ROBOT_SENSOR_RATES_HZ: dict[str, float] = {
    "bar30_pressure": 2.0,
    "depth": 2.0,
    "dvl_twist": 10.0,
    "dvl_position": 4.45,
    "mavros_imu_data": 20.0,
    "mavros_imu_raw": 2.0,
    "mavros_static_pressure": 2.0,
    "mavros_atm_pressure": 10.0,
    "mavros_local_position": 3.0,
    "mavros_rc_out": 2.0,
    "core_imu": 20.0,
}


__all__ = ["REAL_ROBOT_SENSOR_RATES_HZ"]
