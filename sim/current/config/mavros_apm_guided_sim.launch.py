#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    pluginlists_yaml = os.path.join(
        workspace, "sim", "current", "config", "mavros_guided_sim_pluginlists.yaml"
    )
    mavros_share = get_package_share_directory("mavros")
    auv_share = get_package_share_directory("auv")
    apm_config_yaml = os.path.join(mavros_share, "launch", "apm_config.yaml")
    sim_time_yaml = os.path.join(auv_share, "config", "mavros_sim_time.yaml")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=LaunchConfiguration("namespace"),
        output=LaunchConfiguration("log_output"),
        parameters=[
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": LaunchConfiguration("tgt_system"),
                "tgt_component": LaunchConfiguration("tgt_component"),
                "fcu_protocol": LaunchConfiguration("fcu_protocol"),
            },
            pluginlists_yaml,
            apm_config_yaml,
            sim_time_yaml,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:57600"),
            DeclareLaunchArgument("gcs_url", default_value=""),
            DeclareLaunchArgument("tgt_system", default_value="1"),
            DeclareLaunchArgument("tgt_component", default_value="1"),
            DeclareLaunchArgument("fcu_protocol", default_value="v2.0"),
            DeclareLaunchArgument("namespace", default_value="mavros"),
            DeclareLaunchArgument("log_output", default_value="screen"),
            mavros_node,
        ]
    )
