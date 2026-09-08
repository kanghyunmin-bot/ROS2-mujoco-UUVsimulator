#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("hit25_auv_ros2")

    mavros_share = get_package_share_directory("mavros")
    # Simulation only needs the control/telemetry surface used by the GUI and
    # mission controllers. Loading the full ArduPilot plugin set starts dozens
    # of plugins that never receive a SITL message. The physical launch remains
    # untouched and continues to use MAVROS' full ArduPilot plugin list.
    pluginlists_yaml = os.path.join(
        package_share, "config", "mavros_sim_pluginlists.yaml")
    apm_config_yaml = os.path.join(mavros_share, "launch", "apm_config.yaml")
    sim_time_yaml = os.path.join(package_share, "config", "mavros_sim_time.yaml")

    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    fcu_protocol = LaunchConfiguration("fcu_protocol")
    namespace = LaunchConfiguration("namespace")
    log_output = LaunchConfiguration("log_output")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=namespace,
        output=log_output,
        parameters=[
            {
                "fcu_url": fcu_url,
                "gcs_url": gcs_url,
                "tgt_system": tgt_system,
                "tgt_component": tgt_component,
                "fcu_protocol": fcu_protocol,
            },
            # Ordering is contractual: simulation clock overrides come last.
            pluginlists_yaml,
            apm_config_yaml,
            sim_time_yaml,
        ],
        # MAVROS retains the FCU AHRS owner for /mavros/imu/data. MuJoCo owns
        # only the strict raw IMU and Bar30 delivery boundary, so quarantine
        # the JSON-backend sample-and-hold copies of those two topics.
        remappings=[
            (
                "/mavros/imu/data_raw",
                "/uuv_mujoco/mavros_fcu_passthrough/imu/data_raw",
            ),
            (
                "/mavros/imu/static_pressure",
                "/uuv_mujoco/mavros_fcu_passthrough/imu/static_pressure",
            ),
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
