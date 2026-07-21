"""Vertical feedback contract configuration for Ros2Bridge."""

from __future__ import annotations

import os

from bridge.sitl_env import env_flag


def configure_vertical_feedback_contract(bridge: object) -> None:
    bridge._mavros_atm_pressure_value = bridge._env_to_clamped_float(
        "ROS2_UUV_MAVROS_ATM_PRESSURE_VALUE",
        0.24,
        0.0,
        120000.0,
    )
    bridge._sitl_depth_sensor_bias_m = bridge._env_to_clamped_float(
        "ROS2_UUV_SITL_DEPTH_SENSOR_BIAS_M",
        0.0,
        -10.0,
        10.0,
    )
    bridge._ros_depth_sensor_bias_m = bridge._env_to_clamped_float(
        "ROS2_UUV_DEPTH_SENSOR_BIAS_M",
        0.0,
        -10.0,
        10.0,
    )
    bridge._ros_bar30_depth_sensor_bias_m = bridge._env_to_clamped_float(
        "ROS2_UUV_BAR30_DEPTH_SENSOR_BIAS_M",
        bridge._ros_depth_sensor_bias_m,
        -10.0,
        10.0,
    )
    requested_vertical_source = str(
        os.environ.get("ROS2_UUV_SITL_VERTICAL_SOURCE", "bar30")
    ).strip().lower()
    if requested_vertical_source not in {"", "bar30"}:
        print(
            f"[sitl] ignoring ROS2_UUV_SITL_VERTICAL_SOURCE={requested_vertical_source!r}; "
            "SITL vertical feedback is fixed to Bar30 depth/down-velocity.",
            flush=True,
        )
    print(
        "[sitl] vertical feedback contract: Bar30 depth + base_link/Pixhawk IMU "
        f"(ekf_contract={bridge._ekf_contract}, "
        f"dvl_rangefinder={bridge._sitl_dvl_rangefinder_enabled})",
        flush=True,
    )
    print(
        "[sitl] AP_Baro JSON depth contract: "
        f"{bridge._sitl_baro_depth_contract}, "
        f"surface={bridge._bar30_surface_pressure_pa:.3f}Pa, "
        f"real_gnd={bridge._baro_real_ground_pressure_pa:.3f}Pa, "
        f"sitl_gnd={bridge._baro_sitl_ground_pressure_pa:.3f}Pa",
        flush=True,
    )
    bridge._sitl_bar30_prev_depth_m = None
    bridge._sitl_bar30_prev_t = None
    bridge._sitl_initial_depth_hold_active = False
    bridge._sitl_zero_vertical_feedback_while_disarmed = env_flag(
        "ROS2_UUV_SITL_ZERO_VERTICAL_FEEDBACK_WHILE_DISARMED",
        False,
    )
    bridge._sitl_zero_vertical_feedback_after_hold_release_s = bridge._env_to_clamped_float(
        "ROS2_UUV_SITL_ZERO_VERTICAL_FEEDBACK_AFTER_HOLD_RELEASE_S",
        0.0,
        0.0,
        5.0,
    )
    bridge._sitl_zero_vertical_feedback_until_wall = -1.0
    bridge._sitl_zero_vertical_feedback_last_log_wall = -1.0


__all__ = ["configure_vertical_feedback_contract"]
