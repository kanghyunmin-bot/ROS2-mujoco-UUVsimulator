"""ROS/MAVROS sensor surface rate configuration and logging."""

from __future__ import annotations

from sim.contracts import REAL_ROBOT_SENSOR_RATES_HZ

_SENSOR_RATE_SPECS = (
    ("_ros_rate_depth_hz", "ROS2_UUV_DEPTH_HZ", "depth"),
    ("_ros_rate_bar30_hz", "ROS2_UUV_BAR30_HZ", "bar30_pressure"),
    ("_ros_rate_dvl_twist_hz", "ROS2_UUV_DVL_TWIST_HZ", "dvl_twist"),
    ("_ros_rate_dvl_position_hz", "ROS2_UUV_DVL_POSITION_HZ", "dvl_position"),
    ("_ros_rate_mavros_imu_data_hz", "ROS2_UUV_MAVROS_IMU_DATA_HZ", "mavros_imu_data"),
    ("_ros_rate_mavros_imu_raw_hz", "ROS2_UUV_MAVROS_IMU_RAW_HZ", "mavros_imu_raw"),
    (
        "_ros_rate_mavros_static_pressure_hz",
        "ROS2_UUV_MAVROS_STATIC_PRESSURE_HZ",
        "mavros_static_pressure",
    ),
    (
        "_ros_rate_mavros_atm_pressure_hz",
        "ROS2_UUV_MAVROS_ATM_PRESSURE_HZ",
        "mavros_atm_pressure",
    ),
    (
        "_ros_rate_mavros_local_position_hz",
        "ROS2_UUV_MAVROS_LOCAL_POSITION_HZ",
        "mavros_local_position",
    ),
    ("_ros_rate_mavros_rc_out_hz", "ROS2_UUV_MAVROS_RC_OUT_HZ", "mavros_rc_out"),
    ("_ros_rate_core_imu_hz", "ROS2_UUV_CORE_IMU_HZ", "core_imu"),
)


def configure_ros_sensor_rates(bridge: object) -> None:
    for attr, env_name, rate_key in _SENSOR_RATE_SPECS:
        setattr(
            bridge,
            attr,
            bridge._env_to_rate_hz(env_name, REAL_ROBOT_SENSOR_RATES_HZ[rate_key]),
        )


def log_ros_sensor_rates(bridge: object) -> None:
    print(
        "[sensor-rate] ROS/MAVROS surface rates: "
        f"bar30={bridge._ros_rate_bar30_hz:g}Hz, depth={bridge._ros_rate_depth_hz:g}Hz, "
        f"imu_data={bridge._ros_rate_mavros_imu_data_hz:g}Hz, "
        f"imu_raw={bridge._ros_rate_mavros_imu_raw_hz:g}Hz, "
        f"static_pressure={bridge._ros_rate_mavros_static_pressure_hz:g}Hz, "
        f"atm_pressure={bridge._ros_rate_mavros_atm_pressure_hz:g}Hz, "
        f"dvl_twist={bridge._ros_rate_dvl_twist_hz:g}Hz, "
        f"dvl_position={bridge._ros_rate_dvl_position_hz:g}Hz, "
        f"local_position={bridge._ros_rate_mavros_local_position_hz:g}Hz",
        flush=True,
    )


__all__ = ["configure_ros_sensor_rates", "log_ros_sensor_rates"]
