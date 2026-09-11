from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("kmu26_auv_vla_data_collector")
    default_config = f"{package_share}/config/collector.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "buoy_release_image_topic",
                default_value="/imx219/camera1/image_raw/compressed",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="kmu26_auv_vla_data_collector",
                executable="collector",
                name="vla_data_collector",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "buoy_release_image_topic": LaunchConfiguration(
                            "buoy_release_image_topic"
                        ),
                    },
                ],
            ),
        ]
    )
