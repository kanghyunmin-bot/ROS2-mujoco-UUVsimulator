from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bbox_topic", default_value="/vision/buoy_bbox"),
            DeclareLaunchArgument("depth_topic", default_value="/auv/depth"),
            DeclareLaunchArgument("depth_pose_topic", default_value="/depth/pose"),
            DeclareLaunchArgument("depth_pose_scale", default_value="-1.0"),
            DeclareLaunchArgument("depth_pose_offset_m", default_value="0.0"),
            DeclareLaunchArgument("enable_topic", default_value="/mission/control_enable"),
            DeclareLaunchArgument("state_topic", default_value="/mission/state"),
            DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
            DeclareLaunchArgument("rc_monitor_topic", default_value="/mission/rc_command"),
            DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("throttle_channel", default_value="3"),
            DeclareLaunchArgument("yaw_channel", default_value="4"),
            DeclareLaunchArgument("forward_channel", default_value="5"),
            DeclareLaunchArgument("neutral_pwm", default_value="1500"),
            DeclareLaunchArgument(
                "min_pwm",
                default_value="1300",
                description="Minimum thruster PWM command.",
            ),
            DeclareLaunchArgument(
                "max_pwm",
                default_value="1700",
                description="Maximum thruster PWM command.",
            ),
            # 아래 기본값은 mission_state_machine_node.cpp declare_parameter 와 동기화
            DeclareLaunchArgument("max_yaw_delta", default_value="180"),
            DeclareLaunchArgument(
                "forward_pwm",
                default_value="1700",
                description="Maps to approach_forward_pwm (max approach forward).",
            ),
            DeclareLaunchArgument("approach_forward_min_pwm", default_value="1560"),
            DeclareLaunchArgument("search_yaw_pwm", default_value="1600"),
            DeclareLaunchArgument("yaw_invert", default_value="false"),
            DeclareLaunchArgument("vertical_positive_is_up", default_value="true"),
            DeclareLaunchArgument("work_depth_m", default_value="0.4"),
            DeclareLaunchArgument("surface_depth_m", default_value="0.1"),
            DeclareLaunchArgument("max_depth_m", default_value="1.5"),
            DeclareLaunchArgument("buoyancy_hold_delta_pwm", default_value="40"),
            DeclareLaunchArgument("lpf_tau_sec", default_value="0.3"),
            DeclareLaunchArgument("buoy_class_id", default_value="0"),
            DeclareLaunchArgument("stick_class_id", default_value="1"),
            DeclareLaunchArgument("min_detection_hits", default_value="5"),
            DeclareLaunchArgument("approach_area_ratio", default_value="0.30"),
            DeclareLaunchArgument("approach_vision_throttle_weight", default_value="0.4"),
            DeclareLaunchArgument("fork_target_x", default_value="0.30"),
            DeclareLaunchArgument("fork_target_y", default_value="0.70"),
            DeclareLaunchArgument("stick_deadband_x", default_value="0.06"),
            DeclareLaunchArgument("stick_deadband_y", default_value="0.08"),
            DeclareLaunchArgument("align_stable_sec", default_value="0.7"),
            DeclareLaunchArgument("insert_pwm", default_value="1560"),
            DeclareLaunchArgument("insert_duration_sec", default_value="0.8"),
            DeclareLaunchArgument("detach_pwm", default_value="1620"),
            DeclareLaunchArgument("detach_duration_sec", default_value="0.3"),
            DeclareLaunchArgument("backoff_pwm", default_value="1420"),
            DeclareLaunchArgument("backoff_duration_sec", default_value="0.5"),
            DeclareLaunchArgument("search_timeout_sec", default_value="40.0"),
            DeclareLaunchArgument("area_verify_sec", default_value="12.0"),
            Node(
                package="auv_buoy_vision_control",
                executable="mission_state_machine_node",
                name="mission_state_machine_node",
                output="screen",
                parameters=[
                    {
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "depth_pose_topic": LaunchConfiguration("depth_pose_topic"),
                        "depth_pose_scale": ParameterValue(
                            LaunchConfiguration("depth_pose_scale"), value_type=float
                        ),
                        "depth_pose_offset_m": ParameterValue(
                            LaunchConfiguration("depth_pose_offset_m"), value_type=float
                        ),
                        "enable_topic": LaunchConfiguration("enable_topic"),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "rc_override_topic": LaunchConfiguration("rc_override_topic"),
                        "rc_monitor_topic": LaunchConfiguration("rc_monitor_topic"),
                        "control_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"), value_type=float
                        ),
                        "work_depth_m": ParameterValue(
                            LaunchConfiguration("work_depth_m"), value_type=float
                        ),
                        "surface_depth_m": ParameterValue(
                            LaunchConfiguration("surface_depth_m"), value_type=float
                        ),
                        "max_depth_m": ParameterValue(
                            LaunchConfiguration("max_depth_m"), value_type=float
                        ),
                        "buoy_class_id": ParameterValue(
                            LaunchConfiguration("buoy_class_id"), value_type=int
                        ),
                        "stick_class_id": ParameterValue(
                            LaunchConfiguration("stick_class_id"), value_type=int
                        ),
                        "approach_area_ratio": ParameterValue(
                            LaunchConfiguration("approach_area_ratio"), value_type=float
                        ),
                        "approach_vision_throttle_weight": ParameterValue(
                            LaunchConfiguration("approach_vision_throttle_weight"),
                            value_type=float,
                        ),
                        "min_detection_hits": ParameterValue(
                            LaunchConfiguration("min_detection_hits"), value_type=int
                        ),
                        "fork_target_x": ParameterValue(
                            LaunchConfiguration("fork_target_x"), value_type=float
                        ),
                        "fork_target_y": ParameterValue(
                            LaunchConfiguration("fork_target_y"), value_type=float
                        ),
                        "stick_deadband_x": ParameterValue(
                            LaunchConfiguration("stick_deadband_x"), value_type=float
                        ),
                        "stick_deadband_y": ParameterValue(
                            LaunchConfiguration("stick_deadband_y"), value_type=float
                        ),
                        "align_stable_sec": ParameterValue(
                            LaunchConfiguration("align_stable_sec"), value_type=float
                        ),
                        "insert_pwm": ParameterValue(
                            LaunchConfiguration("insert_pwm"), value_type=int
                        ),
                        "insert_duration_sec": ParameterValue(
                            LaunchConfiguration("insert_duration_sec"), value_type=float
                        ),
                        "detach_pwm": ParameterValue(
                            LaunchConfiguration("detach_pwm"), value_type=int
                        ),
                        "detach_duration_sec": ParameterValue(
                            LaunchConfiguration("detach_duration_sec"), value_type=float
                        ),
                        "backoff_pwm": ParameterValue(
                            LaunchConfiguration("backoff_pwm"), value_type=int
                        ),
                        "backoff_duration_sec": ParameterValue(
                            LaunchConfiguration("backoff_duration_sec"), value_type=float
                        ),
                        "search_timeout_sec": ParameterValue(
                            LaunchConfiguration("search_timeout_sec"), value_type=float
                        ),
                        "area_verify_sec": ParameterValue(
                            LaunchConfiguration("area_verify_sec"), value_type=float
                        ),
                        "buoyancy_hold_delta_pwm": ParameterValue(
                            LaunchConfiguration("buoyancy_hold_delta_pwm"), value_type=int
                        ),
                        "lpf_tau_sec": ParameterValue(
                            LaunchConfiguration("lpf_tau_sec"), value_type=float
                        ),
                        "throttle_channel": ParameterValue(
                            LaunchConfiguration("throttle_channel"), value_type=int
                        ),
                        "yaw_channel": ParameterValue(
                            LaunchConfiguration("yaw_channel"), value_type=int
                        ),
                        "forward_channel": ParameterValue(
                            LaunchConfiguration("forward_channel"), value_type=int
                        ),
                        "neutral_pwm": ParameterValue(
                            LaunchConfiguration("neutral_pwm"), value_type=int
                        ),
                        "min_pwm": ParameterValue(
                            LaunchConfiguration("min_pwm"), value_type=int
                        ),
                        "max_pwm": ParameterValue(
                            LaunchConfiguration("max_pwm"), value_type=int
                        ),
                        "max_yaw_delta": ParameterValue(
                            LaunchConfiguration("max_yaw_delta"), value_type=int
                        ),
                        "approach_forward_pwm": ParameterValue(
                            LaunchConfiguration("forward_pwm"), value_type=int
                        ),
                        "approach_forward_min_pwm": ParameterValue(
                            LaunchConfiguration("approach_forward_min_pwm"), value_type=int
                        ),
                        "search_yaw_pwm": ParameterValue(
                            LaunchConfiguration("search_yaw_pwm"), value_type=int
                        ),
                        "yaw_invert": ParameterValue(
                            LaunchConfiguration("yaw_invert"), value_type=bool
                        ),
                        "vertical_positive_is_up": ParameterValue(
                            LaunchConfiguration("vertical_positive_is_up"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
