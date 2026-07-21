"""DVL and frame-transform configuration for Ros2Bridge."""

from __future__ import annotations

import numpy as np

from bridge.ros2_math import rpy_deg_to_rotmat


def configure_dvl_and_frame_transforms(bridge: object) -> None:
    bridge._dvl_filter_alpha = bridge._env_to_clamped_float(
        "ROS2_UUV_DVL_LPF_ALPHA",
        1.0,
        0.0,
        1.0,
    )
    bridge._dvl_vel_body_filt = None
    bridge._dvl_frame_roll_deg = bridge._env_to_clamped_float(
        "ROS2_UUV_DVL_FRAME_ROLL_DEG",
        0.0,
        -20.0,
        20.0,
    )
    bridge._dvl_frame_pitch_deg = bridge._env_to_clamped_float(
        "ROS2_UUV_DVL_FRAME_PITCH_DEG",
        0.0,
        -20.0,
        20.0,
    )
    bridge._dvl_frame_yaw_deg = bridge._env_to_clamped_float(
        "ROS2_UUV_DVL_FRAME_YAW_DEG",
        0.0,
        -20.0,
        20.0,
    )
    bridge._dvl_body_frd_to_dvl_frd = rpy_deg_to_rotmat(
        bridge._dvl_frame_roll_deg,
        bridge._dvl_frame_pitch_deg,
        bridge._dvl_frame_yaw_deg,
    )
    bridge._odom_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    bridge._last_odom_time = -1.0
    # REP-103 world coordinates are ENU (x=east, y=north, z=up), while
    # ArduPilot's JSON/SITL state is NED (x=north, y=east, z=down).  A
    # diagonal sign flip would be NWU->NED and leaves every SITL attitude
    # yaw-shifted by 90 degrees after MAVROS converts it back to ENU.
    bridge._enu_to_ned = np.array(
        [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=np.float64,
    )
    bridge._bmj_to_frd = np.diag([1.0, -1.0, -1.0])
    bridge._bmj_to_flu = np.eye(3, dtype=np.float64)
    bridge._rovio_to_flu = np.array(
        [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    bridge._base_to_rovio = bridge._rovio_to_flu.T
