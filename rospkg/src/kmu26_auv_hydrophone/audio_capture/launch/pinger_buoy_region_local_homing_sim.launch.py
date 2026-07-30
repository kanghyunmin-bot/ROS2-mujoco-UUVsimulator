from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    raw_odometry_topic = LaunchConfiguration("raw_odometry_topic")
    homing_odometry_topic = LaunchConfiguration("homing_odometry_topic")
    signal_mode = LaunchConfiguration("signal_mode")
    noise_bag = LaunchConfiguration("noise_bag")
    frequency_hz = LaunchConfiguration("frequency_hz")
    pinger_x = LaunchConfiguration("pinger_x")
    pinger_y = LaunchConfiguration("pinger_y")
    pinger_z = LaunchConfiguration("pinger_z")
    source_amplitude = LaunchConfiguration("source_amplitude")
    clean_noise_amplitude = LaunchConfiguration("clean_noise_amplitude")
    minimum_distance_m = LaunchConfiguration("minimum_distance_m")
    attenuation_power = LaunchConfiguration("attenuation_power")
    sound_speed_mps = LaunchConfiguration("sound_speed_mps")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controller_mode = LaunchConfiguration("controller_mode")

    # 현재 활성값은 15 x 16 x 11 m 대회장용이다.
    #
    # 5.49 x 2.74 x 1.32 m 실험 수조로 바꿀 때 아래 값으로 교체한다.
    # AUV 시작 odom (0, 0)을 +X/-Y 벽에서 각각 0.30 m 안쪽으로 둔
    # bottom_left 기준이며, 수조 폭이 좁아서 초기 원형 탐색 반경은 1.0 m로 제한한다.
    #
    # ("arena_length_m", "5.49"),
    # ("arena_width_m", "2.74"),
    # ("arena_offset_x_m", "-0.30"),
    # ("arena_offset_y_m", "0.30"),
    # ("arena_safety_margin_m", "0.20"),
    # ("initial_scan_radius_m", "1.00"),
    # ("rescan_radius_m", "0.50"),
    # ("homing_waypoint_step_m", "0.50"),
    # ("homing_zigzag_offset_m", "0.15"),
    # ("vision_near_zone_width_m", "0.60"),
    # ("target_depth_z_m", "-0.65"),
    # ("pinger_x", "2.20"),
    # ("pinger_y", "-1.00"),
    # ("pinger_z", "-1.00"),
    #
    # 위 pinger 좌표는 scene.xml 실험 수조 프리셋의 pinger_source와 같다.
    # AUV의 시작 z도 실험 수조 범위인 -1.32~0 m 안으로 별도 이동해야 한다.
    homing_float_names = [

        # 대회장용
        # ("arena_length_m", "15.0"),
        # ("arena_width_m", "16.0"),
        # # 15 x 16 m 경기장의 +Y/-X 코너에서 0.55 m 안쪽인 AUV 시작점을
        # # odom (0,0)으로 변환한 경기장 경계 오프셋.
        # ("arena_offset_x_m", "-0.55"),
        # ("arena_offset_y_m", "0.55"),
        # ("arena_safety_margin_m", "0.55"),
        # ("initial_scan_radius_m", "1.50"),
        # ("rescan_radius_m", "0.70"),
        # ("homing_waypoint_step_m", "0.80"),
        # ("homing_zigzag_offset_m", "0.20"),
        # ("target_depth_z_m", "-8.00"),

        # 실험 수조용
        ("arena_length_m", "5.49"),
        ("arena_width_m", "2.74"),
        ("arena_offset_x_m", "-0.30"),
        ("arena_offset_y_m", "0.30"),
        ("arena_safety_margin_m", "0.40"),
        ("initial_scan_radius_m", "0.70"),
        ("rescan_radius_m", "0.50"),
        ("homing_waypoint_step_m", "0.50"),
        ("homing_zigzag_offset_m", "0.15"),
        ("target_depth_z_m", "-0.65"),

        ("rolling_gradient_alpha", "0.15"),
        ("rolling_gradient_conflict_angle_rad", "1.0472"),
        ("waypoint_reach_tolerance_m", "0.15"),
        ("scan_waypoint_lookahead_rad", "0.35"),
        ("region_sample_spacing_m", "0.15"),
        ("min_region_gradient_magnitude", "0.05"),
        ("min_region_lateral_spread_m", "0.10"),
        ("vision_near_zone_width_m", "0.60"),
        # ("vision_near_zone_width_m", "2.0"),
    ]
    homing_int_names = [
        ("homing_gradient_window_size", "12"),
        ("min_homing_gradient_samples", "8"),
        ("rolling_gradient_conflict_limit", "3"),
    ]
    homing_values = {
        name: LaunchConfiguration(name)
        for name, _ in homing_float_names + homing_int_names
    }
    line_search_float_names = [
        ("snr_sample_spacing_m", "0.15"),
        ("snr_drop_from_peak_db", "2.0"),
        ("snr_timeout_s", "1.0"),
        ("max_snr_odom_skew_s", "0.15"),
        ("odometry_timeout_s", "0.5"),
        ("rate_hz", "30.0"),
    ]
    line_search_int_names = [
        ("line_search_direction", "1"),
        ("snr_decline_count_limit", "5"),
        ("snr_median_window_size", "3"),
    ]

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "controller_mode",
            default_value="region",
            description="region 또는 line_search. 두 제어기는 동시에 실행하지 않는다.",
        ),
        DeclareLaunchArgument(
            "raw_odometry_topic", default_value="/odometry/filtered"
        ),
        DeclareLaunchArgument(
            "homing_odometry_topic", default_value="/homing/sim_odometry"
        ),
        DeclareLaunchArgument(
            "signal_mode",
            default_value="noisy",
            description="clean 또는 실측 배경 잡음이 섞인 noisy.",
        ),
        DeclareLaunchArgument(
            "noise_bag",
            default_value="/home/kim/new_hydrophone_ws/localization_20260719_185918",
        ),
        DeclareLaunchArgument("frequency_hz", default_value="21164.0"),
        # scene.xml 실험 수조 프리셋의 pinger_source world 좌표.
        DeclareLaunchArgument("pinger_x", default_value="2.20"),
        DeclareLaunchArgument("pinger_y", default_value="-1.00"),
        DeclareLaunchArgument("pinger_z", default_value="-0.65"),
        DeclareLaunchArgument("source_amplitude", default_value="0.03"),
        DeclareLaunchArgument("clean_noise_amplitude", default_value="0.001"),
        DeclareLaunchArgument("minimum_distance_m", default_value="0.5"),
        DeclareLaunchArgument("attenuation_power", default_value="2.0"),
        DeclareLaunchArgument("sound_speed_mps", default_value="1500.0"),
        DeclareLaunchArgument("arena_start_corner", default_value="bottom_left"),
        DeclareLaunchArgument(
            "arena_start_frame_topic", default_value="/guided/start_frame"
        ),
        DeclareLaunchArgument("vision_handoff_enabled", default_value="true"),
        DeclareLaunchArgument(
            "vision_search_request_topic",
            default_value="/homing/vision_search_active",
        ),
        DeclareLaunchArgument(
            "target_confirmed_topic", default_value="/vision/target_confirmed"
        ),
        # [ACOUSTIC-VISION HANDSHAKE] Acoustic RC 종료 후 Vision 제어를 승인한다.
        DeclareLaunchArgument(
            "vision_control_granted_topic",
            default_value="/homing/vision_control_granted",
        ),
        DeclareLaunchArgument(
            "rolling_gradient_topic", default_value="/homing/rolling_gradient"
        ),
        DeclareLaunchArgument(
            "homing_direction_topic", default_value="/homing/homing_direction"
        ),
        DeclareLaunchArgument(
            "emergency_stop_topic", default_value="/mission/emergency_stop"
        ),
        DeclareLaunchArgument("enable_keyboard_emergency_stop", default_value="true"),
        DeclareLaunchArgument("emergency_stop_key", default_value="s"),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="true이면 V3 SNR map/gradient RViz를 함께 실행한다.",
        ),
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in (
            homing_float_names + homing_int_names +
            line_search_float_names + line_search_int_names
        )
    ]

    pinger_audio = Node(
        package="audio_capture",
        executable="pinger_buoy_audio_sim.py",
        name="pinger_buoy_audio_sim",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "signal_mode": signal_mode,
                "noise_bag": noise_bag,
                "frequency_hz": ParameterValue(frequency_hz, value_type=float),
                "pinger_x": ParameterValue(pinger_x, value_type=float),
                "pinger_y": ParameterValue(pinger_y, value_type=float),
                "pinger_z": ParameterValue(pinger_z, value_type=float),
                "source_amplitude": ParameterValue(
                    source_amplitude, value_type=float
                ),
                "clean_noise_amplitude": ParameterValue(
                    clean_noise_amplitude, value_type=float
                ),
                "minimum_distance_m": ParameterValue(
                    minimum_distance_m, value_type=float
                ),
                "attenuation_power": ParameterValue(
                    attenuation_power, value_type=float
                ),
                "sound_speed_mps": ParameterValue(
                    sound_speed_mps, value_type=float
                ),
                "odometry_topic": raw_odometry_topic,
            }
        ],
    )

    odometry_rebaser = Node(
        package="audio_capture",
        executable="sim_odometry_rebaser",
        name="sim_odometry_rebaser",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "input_topic": raw_odometry_topic,
                "output_topic": homing_odometry_topic,
                "start_frame_input_topic": LaunchConfiguration(
                    "arena_start_frame_topic"
                ),
                "start_frame_output_topic": "/homing/sim_start_frame",
            }
        ],
    )

    package_share = get_package_share_directory("audio_capture")
    homing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_share + "/launch/region_local_gradient_homing.launch.py"
        ),
        condition=IfCondition(
            PythonExpression(["'", controller_mode, "' == 'region'"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odometry_topic": homing_odometry_topic,
            "arena_start_frame_topic": "/homing/sim_start_frame",
            "arena_start_corner": LaunchConfiguration("arena_start_corner"),
            "vision_handoff_enabled": LaunchConfiguration(
                "vision_handoff_enabled"
            ),
            "vision_search_request_topic": LaunchConfiguration(
                "vision_search_request_topic"
            ),
            "target_confirmed_topic": LaunchConfiguration(
                "target_confirmed_topic"
            ),
            "vision_control_granted_topic": LaunchConfiguration(
                "vision_control_granted_topic"
            ),
            "rolling_gradient_topic": LaunchConfiguration(
                "rolling_gradient_topic"
            ),
            "homing_direction_topic": LaunchConfiguration(
                "homing_direction_topic"
            ),
            "emergency_stop_topic": LaunchConfiguration("emergency_stop_topic"),
            "enable_keyboard_emergency_stop": LaunchConfiguration(
                "enable_keyboard_emergency_stop"
            ),
            "emergency_stop_key": LaunchConfiguration("emergency_stop_key"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            **homing_values,
        }.items(),
    )

    line_search_detector = Node(
        package="audio_capture",
        executable="audio_frequency_detector",
        name="audio_frequency_detector",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller_mode, "' == 'line_search'"])
        ),
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
        ],
    )

    line_search = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_share + "/launch/near_zone_line_search_homing.launch.py"
        ),
        condition=IfCondition(
            PythonExpression(["'", controller_mode, "' == 'line_search'"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odometry_topic": homing_odometry_topic,
            "arena_start_frame_topic": "/homing/sim_start_frame",
            "arena_start_corner": LaunchConfiguration("arena_start_corner"),
            "arena_length_m": LaunchConfiguration("arena_length_m"),
            "arena_width_m": LaunchConfiguration("arena_width_m"),
            "arena_offset_x_m": LaunchConfiguration("arena_offset_x_m"),
            "arena_offset_y_m": LaunchConfiguration("arena_offset_y_m"),
            "arena_safety_margin_m": LaunchConfiguration(
                "arena_safety_margin_m"
            ),
            "vision_near_zone_width_m": LaunchConfiguration(
                "vision_near_zone_width_m"
            ),
            "target_depth_z_m": LaunchConfiguration("target_depth_z_m"),
            "waypoint_reach_tolerance_m": LaunchConfiguration(
                "waypoint_reach_tolerance_m"
            ),
            "snr_sample_spacing_m": LaunchConfiguration(
                "snr_sample_spacing_m"
            ),
            "snr_drop_from_peak_db": LaunchConfiguration(
                "snr_drop_from_peak_db"
            ),
            "snr_timeout_s": LaunchConfiguration("snr_timeout_s"),
            "max_snr_odom_skew_s": LaunchConfiguration(
                "max_snr_odom_skew_s"
            ),
            "odometry_timeout_s": LaunchConfiguration("odometry_timeout_s"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "line_search_direction": LaunchConfiguration(
                "line_search_direction"
            ),
            "snr_decline_count_limit": LaunchConfiguration(
                "snr_decline_count_limit"
            ),
            "snr_median_window_size": LaunchConfiguration(
                "snr_median_window_size"
            ),
            "vision_search_request_topic": LaunchConfiguration(
                "vision_search_request_topic"
            ),
            "target_confirmed_topic": LaunchConfiguration(
                "target_confirmed_topic"
            ),
            "vision_control_granted_topic": LaunchConfiguration(
                "vision_control_granted_topic"
            ),
            "emergency_stop_topic": LaunchConfiguration("emergency_stop_topic"),
            "enable_keyboard_emergency_stop": LaunchConfiguration(
                "enable_keyboard_emergency_stop"
            ),
            "emergency_stop_key": LaunchConfiguration("emergency_stop_key"),
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_share + "/launch/region_local_gradient_rviz.launch.py"
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odometry_topic": homing_odometry_topic,
            "arena_length_m": LaunchConfiguration("arena_length_m"),
            "arena_width_m": LaunchConfiguration("arena_width_m"),
            "arena_offset_x_m": LaunchConfiguration("arena_offset_x_m"),
            "arena_offset_y_m": LaunchConfiguration("arena_offset_y_m"),
            "arena_safety_margin_m": LaunchConfiguration("arena_safety_margin_m"),
            "vision_near_zone_width_m": LaunchConfiguration(
                "vision_near_zone_width_m"
            ),
            "arena_start_corner": LaunchConfiguration("arena_start_corner"),
            "map_cell_size_m": LaunchConfiguration(
                "region_sample_spacing_m"
            ),
            "rolling_gradient_topic": LaunchConfiguration(
                "rolling_gradient_topic"
            ),
            "homing_direction_topic": LaunchConfiguration(
                "homing_direction_topic"
            ),
        }.items(),
    )

    return LaunchDescription(
        arguments + [
            pinger_audio,
            odometry_rebaser,
            homing,
            line_search_detector,
            line_search,
            rviz,
        ]
    )
