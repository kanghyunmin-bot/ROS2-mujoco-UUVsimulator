from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "odometry_topic", default_value="/homing/sim_odom"
            ),
            DeclareLaunchArgument("depth_pose_topic", default_value="/depth/pose"),
            DeclareLaunchArgument("start_frame_topic", default_value="/start_frame"),
            DeclareLaunchArgument(
                "arena_config_topic", default_value="/mission/arena_config"
            ),
            DeclareLaunchArgument(
                "surface_start_topic", default_value="/mission/surface_start"
            ),
            DeclareLaunchArgument(
                "surface_complete_topic", default_value="/mission/surface_complete"
            ),
            DeclareLaunchArgument(
                "front_bbox_topic", default_value="/vision/surface/front/buoy_bbox"
            ),
            DeclareLaunchArgument(
                "top_bbox_topic", default_value="/vision/surface/top/buoy_bbox"
            ),
            DeclareLaunchArgument(
                "score_release_topic", default_value="/mission/score_release"
            ),
            DeclareLaunchArgument(
                "rc_override_topic", default_value="/mavros/rc/override"
            ),
            Node(
                package="kmu26_auv_surface_buoy_mission",
                executable="surface_buoy_mission_node",
                name="surface_buoy_mission_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "odometry_topic": LaunchConfiguration("odometry_topic"),
                        "depth_pose_topic": LaunchConfiguration("depth_pose_topic"),
                        "start_frame_topic": LaunchConfiguration("start_frame_topic"),
                        "arena_config_topic": LaunchConfiguration("arena_config_topic"),
                        "surface_start_topic": LaunchConfiguration("surface_start_topic"),
                        "surface_complete_topic": LaunchConfiguration(
                            "surface_complete_topic"
                        ),
                        "surface_front_bbox_topic": LaunchConfiguration(
                            "front_bbox_topic"
                        ),
                        "surface_top_bbox_topic": LaunchConfiguration("top_bbox_topic"),
                        "score_release_topic": LaunchConfiguration(
                            "score_release_topic"
                        ),
                        "rc_override_topic": LaunchConfiguration("rc_override_topic"),
                    }
                ],
            ),
        ]
    )
