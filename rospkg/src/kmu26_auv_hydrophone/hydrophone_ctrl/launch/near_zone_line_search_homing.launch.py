from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    string_names = [
        ("odometry_topic", "/odometry/filtered"),
        ("start_frame_topic", "/start_frame"),
        ("snr_topic", "/audio_frequency_detector/snr_db_stamped"),
        ("state_topic", "/homing/control_state"),
        ("waypoint_topic", "/homing/current_waypoint"),
        ("arena_frame_id", "arena"),
        ("peak_topic", "/homing/snr_peak_position"),
        ("vision_search_request_topic", "/homing/vision_search_active"),
        ("target_confirmed_topic", "/vision/target_confirmed"),
        ("vision_control_granted_topic", "/homing/vision_control_granted"),
        ("rc_override_topic", "/mavros/rc/override"),
        ("emergency_stop_topic", "/mission/emergency_stop"),
        ("emergency_stop_key", "s"),
        ("arena_start_corner", "bottom_left"),
    ]
    float_names = [
        ("arena_length_m", "15.0"),
        ("arena_width_m", "16.0"),
        ("arena_offset_x_m", "0.0"),
        ("arena_offset_y_m", "0.0"),
        ("arena_safety_margin_m", "0.5"),
        ("vision_near_zone_width_m", "2.0"),
        ("acoustic_timeout_s", "90.0"),
        ("target_depth_z_m", "-8.0"),
        ("line_search_start_depth_tolerance_m", "0.2"),
        ("waypoint_reach_tolerance_m", "0.15"),
        ("snr_sample_spacing_m", "0.15"),
        ("snr_drop_from_peak_db", "2.0"),
        ("snr_timeout_s", "1.0"),
        ("max_snr_odom_skew_s", "0.15"),
        ("odometry_timeout_s", "0.5"),
        ("forward_cruise", "0.5"),
        ("yaw_kp", "1.15"),
        ("yaw_ki", "0.15"),
        ("yaw_kd", "0.08"),
        ("yaw_integral_limit", "2.0"),
        ("yaw_limit", "0.72"),
        ("move_heading_tolerance_rad", "0.1745"),
        ("rc_pwm_span", "400.0"),
        ("rate_hz", "30.0"),
    ]
    int_names = [
        ("line_search_direction", "1"),
        ("snr_decline_count_limit", "5"),
        ("snr_median_window_size", "3"),
    ]
    bool_names = [
        ("use_sim_time", "false"),
        ("invert_rc_yaw", "true"),
        ("enable_keyboard_emergency_stop", "true"),
    ]
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in string_names + float_names + int_names + bool_names
    ]
    parameters = {
        name: LaunchConfiguration(name) for name, _ in string_names
    }
    parameters.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name, _ in float_names
    })
    parameters.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=int)
        for name, _ in int_names
    })
    parameters.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=bool)
        for name, _ in bool_names
    })

    return LaunchDescription(
        arguments
        + [
            Node(
                package="hydrophone_ctrl",
                executable="near_zone_line_search_controller",
                name="near_zone_line_search_controller",
                output="screen",
                emulate_tty=True,
                parameters=[parameters],
            )
        ]
    )
