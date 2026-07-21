from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    reference_frequency_hz = LaunchConfiguration("reference_frequency_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reference_frequency_hz",
                default_value="21164.0",
                description="Expected pinger frequency in Hz",
            ),
            ComposableNodeContainer(
                name="pinger_audio_pipeline",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="audio_capture",
                        plugin="audio_capture::AudioFrequencyDetectorNode",
                        name="audio_frequency_detector",
                    ),
                    ComposableNode(
                        package="audio_capture",
                        plugin="audio_capture::AudioPhaseEstimatorNode",
                        name="audio_phase_estimator",
                        parameters=[{"reference_frequency_hz": reference_frequency_hz}],
                    ),
                ],
                output="screen",
            ),
        ]
    )
