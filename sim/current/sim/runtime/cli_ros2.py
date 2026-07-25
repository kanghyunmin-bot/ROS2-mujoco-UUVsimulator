"""ROS2 bridge CLI options for the UUV MuJoCo runner."""

from __future__ import annotations

import argparse

from .cli_common import env_float


def add_ros2_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="Enable ROS2 transport topics (/cmd_vel as TwistStamped, /imu/data, /dvl/*)",
    )
    parser.add_argument(
        "--ros2-real-pkg-compat",
        action="store_true",
        help="Keep ROS2 sensor topics but reduce the simulator MAVROS surface to compat-only (/mavros/vfr_hud) for external MAVROS + kmu26_auv.",
    )
    parser.add_argument(
        "--ros2-images",
        action="store_true",
        help="Publish stereo_left/stereo_right MuJoCo camera frames as ROS2 Image topics",
    )
    parser.add_argument(
        "--ros2-image-width",
        type=int,
        default=1280,
        help="Stereo image width for ROS2 image topics",
    )
    parser.add_argument(
        "--ros2-image-height",
        type=int,
        default=720,
        help="Stereo image height for ROS2 image topics",
    )
    parser.add_argument(
        "--ros2-sensor-hz",
        type=float,
        default=env_float("UUV_ROS2_SENSOR_HZ", 60.0),
        help="ROS2 IMU/DVL publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-image-hz",
        type=float,
        default=30.0,
        help="ROS2 stereo image publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-camera-calib-left",
        type=str,
        default="",
        help="Path to left camera calibration YAML (camera_info format)",
    )
    parser.add_argument(
        "--ros2-camera-calib-right",
        type=str,
        default="",
        help="Path to right camera calibration YAML (camera_info format)",
    )
