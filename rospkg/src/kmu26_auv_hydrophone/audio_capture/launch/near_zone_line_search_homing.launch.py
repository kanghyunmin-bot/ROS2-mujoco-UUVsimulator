from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    string_names = [
        ("odometry_topic", "/odometry/filtered"),
        ("snr_topic", "/audio_frequency_detector/snr_db_stamped"),
        ("state_topic", "/homing/control_state"),
        ("waypoint_topic", "/waypoint"),  # odom 절대좌표 PositionTarget
        ("arena_start_frame_topic", "/guided/start_frame"),
        ("peak_topic", "/homing/snr_peak_position"),
        ("vision_search_request_topic", "/homing/vision_search_active"),
        ("target_confirmed_topic", "/vision/target_confirmed"),
        ("vision_control_granted_topic", "/homing/vision_control_granted"),
        ("guided_waypoint_enable_topic", "/guided/waypoint_enable"),
        ("guided_status_topic", "/guided/status"),
        ("fcu_state_topic", "/mavros/state"),
        ("set_mode_service", "/mavros/set_mode"),
        ("vision_mode_name", "STABILIZE"),
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
        ("target_depth_z_m", "-8.0"),
        ("waypoint_reach_tolerance_m", "0.15"),
        ("snr_sample_spacing_m", "0.15"),
        ("snr_drop_from_peak_db", "2.0"),
        ("snr_timeout_s", "1.0"),
        ("max_snr_odom_skew_s", "0.15"),
        ("odometry_timeout_s", "0.5"),
        ("fcu_state_timeout_s", "1.0"),
        ("handoff_hold_sec", "0.7"),
        ("handoff_max_speed_mps", "0.2"),
        ("mode_request_interval_s", "1.0"),
        ("rate_hz", "30.0"),
    ]
    int_names = [
        ("line_search_direction", "1"),
        ("snr_decline_count_limit", "5"),
        ("snr_median_window_size", "3"),
    ]
    bool_names = [
        ("use_sim_time", "false"),
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
                package="audio_capture",
                executable="near_zone_line_search_controller",
                name="near_zone_line_search_controller",
                output="screen",
                emulate_tty=True,
                parameters=[parameters],
            )
        ]
    )
