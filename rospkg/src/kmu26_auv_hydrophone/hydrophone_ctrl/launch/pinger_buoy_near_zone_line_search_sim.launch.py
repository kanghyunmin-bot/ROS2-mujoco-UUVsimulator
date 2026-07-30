from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory("hydrophone_ctrl")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    package_share
                    + "/launch/pinger_buoy_region_local_homing_sim.launch.py"
                ),
                launch_arguments={"controller_mode": "line_search"}.items(),
            )
        ]
    )
