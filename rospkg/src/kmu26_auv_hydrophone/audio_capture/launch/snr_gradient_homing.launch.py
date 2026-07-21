from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# ros2 launch audio_capture snr_gradient_homing_control.launch.py \
#   particle_start_corner:=bottom_right

# ros2 launch audio_capture snr_gradient_homing.launch.py \
#   particle_area_width_m:=15.0 \
#   particle_area_height_m:=16.0 \
#   particle_start_corner:=bottom_left

# [SNR homing launch 구성] 오디오 분석기와 gradient/PF 추정기를 하나의 container로 구성한다.
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    audio_topic = LaunchConfiguration("audio_topic")
    audio_stamped_topic = LaunchConfiguration("audio_stamped_topic")
    use_stamped_audio = LaunchConfiguration("use_stamped_audio")
    odometry_topic = LaunchConfiguration("odometry_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    reference_frequency_hz = LaunchConfiguration("reference_frequency_hz")
    audio_input_latency_s = LaunchConfiguration("audio_input_latency_s")
    output_frame = LaunchConfiguration("output_frame")
    direction_source = LaunchConfiguration("direction_source")
    particle_count = LaunchConfiguration("particle_count")
    particle_area_width_m = LaunchConfiguration("particle_area_width_m")
    particle_area_height_m = LaunchConfiguration("particle_area_height_m")
    particle_start_corner = LaunchConfiguration("particle_start_corner")
    particle_area_yaw_rad = LaunchConfiguration("particle_area_yaw_rad")
    particle_roughening_std_m = LaunchConfiguration("particle_roughening_std_m")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("audio_topic", default_value="/audio"),
            DeclareLaunchArgument("audio_stamped_topic", default_value="/audio_stamped"),
            DeclareLaunchArgument("use_stamped_audio", default_value="false"),
            DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
            DeclareLaunchArgument(
                "reference_frequency_hz",
                default_value="21164.0",
                description="Expected pinger frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "audio_input_latency_s",
                default_value="0.0",
                description="Known transport/capture latency in seconds.",
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value="base_link",
                description="Direction output frame; base_link is the safe controller-compatible default.",
            ),
            DeclareLaunchArgument(
                "direction_source",
                default_value="blend",
                description="Direction source: gradient, particle, or blend.",
            ),
            DeclareLaunchArgument(
                "particle_area_width_m",
                default_value="15.0",
                description="Planar arena width from left to right in metres.",
            ),
            DeclareLaunchArgument(
                "particle_area_height_m",
                default_value="16.0",
                description="Planar arena length from bottom to top in metres.",
            ),
            DeclareLaunchArgument("particle_count", default_value="500"),
            DeclareLaunchArgument(
                "particle_start_corner",
                default_value="bottom_left",
                description="AUV start corner: bottom_left or bottom_right.",
            ),
            DeclareLaunchArgument(
                "particle_area_yaw_rad",
                default_value="0.0",
                description="Arena left-to-right axis yaw in the odometry frame.",
            ),
            DeclareLaunchArgument("particle_roughening_std_m", default_value="0.20"),
            ComposableNodeContainer(
                name="snr_gradient_homing_pipeline",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="audio_capture",
                        plugin="audio_capture::AudioPhaseEstimatorNode",
                        name="audio_phase_estimator",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "audio_topic": audio_topic,
                                "audio_stamped_topic": audio_stamped_topic,
                                "use_stamped_audio": ParameterValue(
                                    use_stamped_audio, value_type=bool
                                ),
                                "odometry_topic": odometry_topic,
                                "depth_topic": depth_topic,
                                "reference_frequency_hz": ParameterValue(
                                    reference_frequency_hz, value_type=float
                                ),
                                "initial_demodulation_frequency_hz": ParameterValue(
                                    reference_frequency_hz, value_type=float
                                ),
                                "audio_input_latency_s": ParameterValue(
                                    audio_input_latency_s, value_type=float
                                ),
                                "publish_homing_direction": False,
                            }
                        ],
                    ),
                    ComposableNode(
                        package="audio_capture",
                        plugin="audio_capture::SnrGradientHomingNode",
                        name="snr_gradient_homing",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                                "odometry_topic": odometry_topic,
                                "depth_topic": depth_topic,
                                "output_frame": output_frame,
                                "direction_source": direction_source,
                                "particle_count": ParameterValue(
                                    particle_count, value_type=int
                                ),
                                "particle_area_width_m": ParameterValue(
                                    particle_area_width_m, value_type=float
                                ),
                                "particle_area_height_m": ParameterValue(
                                    particle_area_height_m, value_type=float
                                ),
                                "particle_start_corner": particle_start_corner,
                                "particle_area_yaw_rad": ParameterValue(
                                    particle_area_yaw_rad, value_type=float
                                ),
                                "particle_roughening_std_m": ParameterValue(
                                    particle_roughening_std_m, value_type=float
                                ),
                            }
                        ],
                    ),
                ],
                output="screen",
            ),
        ]
    )
