#!/usr/bin/env python3
# Copyright (c) 2026, KMU Underwater Robot Team.
# SPDX-License-Identifier: MIT

"""Launch the focused camera, sonar, DVL, and localization RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the robot, A50 DVL, EKF odometry, camera, and sonar view."""
    package_share = get_package_share_directory("hit25_auv_ros2")
    rviz_config = os.path.join(
        package_share, "rviz", "sensor_localization.rviz"
    )
    camera_topic = LaunchConfiguration("camera_topic")
    sonar_topic = LaunchConfiguration("sonar_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    visualizer = Node(
        package="hit25_auv_ros2",
        executable="dvl_localization_visualizer.py",
        name="dvl_localization_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "dvl_topic": "/dvl/data",
                "odom_topic": "/odometry/filtered",
                "marker_topic": "/dvl/markers",
                "path_topic": "/localization/filtered_path",
                "velocity_scale_s": 5.0,
                "marker_lifetime_s": 0.35,
                "max_path_points": 2500,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="camera_sonar_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[
            ("/imx219/camera0/image_raw", camera_topic),
            ("/ping360/scan_image", sonar_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_topic", default_value="/imx219/camera0/image_raw"
            ),
            DeclareLaunchArgument(
                "sonar_topic", default_value="/ping360/scan_image"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            visualizer,
            rviz,
        ]
    )
