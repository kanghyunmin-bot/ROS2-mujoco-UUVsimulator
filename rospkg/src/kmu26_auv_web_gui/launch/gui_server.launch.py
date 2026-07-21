#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    dronecan_python_default = os.path.expanduser("~/miniconda3/envs/auv_ros2/bin/python")
    if not os.path.exists(dronecan_python_default):
        dronecan_python_default = "python3"

    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    robot_package = LaunchConfiguration("robot_package")
    robot_launch = LaunchConfiguration("robot_launch")
    start_dronecan_allocator = LaunchConfiguration("start_dronecan_allocator")
    dronecan_can_interface = LaunchConfiguration("dronecan_can_interface")
    dronecan_allocator_node_id = LaunchConfiguration("dronecan_allocator_node_id")
    dronecan_allocator_db = LaunchConfiguration("dronecan_allocator_db")
    dronecan_python = LaunchConfiguration("dronecan_python")
    pinger_package = LaunchConfiguration("pinger_package")
    pinger_launch = LaunchConfiguration("pinger_launch")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("robot_package", default_value="hit25_auv_ros2"),
            DeclareLaunchArgument("robot_launch", default_value="localization_test.launch.py"),
            DeclareLaunchArgument("start_dronecan_allocator", default_value="true"),
            DeclareLaunchArgument("dronecan_can_interface", default_value="can0"),
            DeclareLaunchArgument("dronecan_allocator_node_id", default_value="126"),
            DeclareLaunchArgument("dronecan_allocator_db", default_value=""),
            DeclareLaunchArgument("dronecan_python", default_value=dronecan_python_default),
            DeclareLaunchArgument("pinger_package", default_value="kmu26_pinger_homing"),
            DeclareLaunchArgument("pinger_launch", default_value="pinger_homing_real.launch.py"),
            Node(
                package="kmu26_auv_web_gui",
                executable="server",
                name="kmu26_auv_web_gui_server",
                output="screen",
                arguments=[
                    "--host",
                    host,
                    "--port",
                    port,
                    "--robot-package",
                    robot_package,
                    "--robot-launch",
                    robot_launch,
                    "--start-dronecan-allocator",
                    start_dronecan_allocator,
                    "--dronecan-can-interface",
                    dronecan_can_interface,
                    "--dronecan-allocator-node-id",
                    dronecan_allocator_node_id,
                    "--dronecan-allocator-db",
                    dronecan_allocator_db,
                    "--dronecan-python",
                    dronecan_python,
                    "--pinger-package",
                    pinger_package,
                    "--pinger-launch",
                    pinger_launch,
                ],
            ),
        ]
    )
