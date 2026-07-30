import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vision_share = get_package_share_directory("auv_buoy_vision_control")
    hydrophone_share = get_package_share_directory("hydrophone_ctrl")
    vision_control_launch = os.path.join(
        vision_share, "launch", "underwater_pinger_vision_control.launch.py"
    )
    hydrophone_launch = os.path.join(
        hydrophone_share, "launch", "competition_snr_homing.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_immediately", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_control_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "force_control_grant": "false",
                    "work_depth_m": "8.65",
                    "max_depth_m": "10.5",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hydrophone_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "start_immediately": LaunchConfiguration(
                        "start_immediately"
                    ),
                    "vision_handoff_enabled": "true",
                }.items(),
            ),
        ]
    )
