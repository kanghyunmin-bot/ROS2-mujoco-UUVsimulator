#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Launch the physical A50 driver with an explicit ROS clock contract."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("ip_address", default_value="192.168.194.95"),
            DeclareLaunchArgument("velocity_frame_id", default_value="dvl_link"),
            DeclareLaunchArgument("position_frame_id", default_value="dvl_link"),
            DeclareLaunchArgument("configure_acoustic_on_startup", default_value="false"),
            DeclareLaunchArgument("startup_acoustic_enabled", default_value="true"),
            DeclareLaunchArgument("request_config_on_startup", default_value="true"),
            DeclareLaunchArgument("reconnect_interval_ms", default_value="2000"),
            DeclareLaunchArgument("connect_timeout_ms", default_value="1000"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock for host-receipt ROS header stamps.",
            ),
            Node(
                package="auv_dvl_a50",
                executable="auv_dvl_a50_sensor",
                parameters=[
                    {
                        "dvl_ip_address": LaunchConfiguration("ip_address"),
                        "velocity_frame_id": LaunchConfiguration("velocity_frame_id"),
                        "position_frame_id": LaunchConfiguration("position_frame_id"),
                        "configure_acoustic_on_startup": ParameterValue(
                            LaunchConfiguration("configure_acoustic_on_startup"),
                            value_type=bool,
                        ),
                        "startup_acoustic_enabled": ParameterValue(
                            LaunchConfiguration("startup_acoustic_enabled"),
                            value_type=bool,
                        ),
                        "request_config_on_startup": ParameterValue(
                            LaunchConfiguration("request_config_on_startup"),
                            value_type=bool,
                        ),
                        "reconnect_interval_ms": ParameterValue(
                            LaunchConfiguration("reconnect_interval_ms"),
                            value_type=int,
                        ),
                        "connect_timeout_ms": ParameterValue(
                            LaunchConfiguration("connect_timeout_ms"),
                            value_type=int,
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
