from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("kmu26_auv_vla_policy")) / "config/sim_policy.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        Node(package="kmu26_auv_vla_policy", executable="policy",
             parameters=[LaunchConfiguration("config")], output="screen"),
    ])
