from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# 실험 수조버전 런치
# ros2 launch hydrophone_ctrl region_local_gradient_homing.launch.py \
#   use_sim_time:=false \
#   arena_start_corner:=bottom_right \
#   arena_length_m:=5.50 \
#   arena_width_m:=2.74 \
#   arena_offset_x_m:=-0.30 \
#   arena_offset_y_m:=-0.30 \
#   arena_safety_margin_m:=0.40 \
#   initial_scan_radius_m:=0.40 \
#   rescan_radius_m:=0.50 \
#   homing_waypoint_step_m:=0.50 \
#   homing_zigzag_offset_m:=0.15 \
#   vision_near_zone_width_m:=0.40 \
#   forward_cruise:=0.35 \
#   target_depth_z_m:=-0.60 \
#   depth_kp:=1.2 \
#   depth_ki:=0.15  

#   arena_offset_y_m:=-0.30 \ (bottom_left면 양수로, bottom_right면 음수로)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")  # 시뮬 시계
    odometry_topic = LaunchConfiguration("odometry_topic")  # odometry 입력
    start_frame_topic = LaunchConfiguration("start_frame_topic")
    snr_topic = LaunchConfiguration("snr_topic")  # SNR 입력
    state_topic = LaunchConfiguration("state_topic")  # 제어 상태
    waypoint_topic = LaunchConfiguration("waypoint_topic")  # 현재 waypoint
    scan_center_topic = LaunchConfiguration("scan_center_topic")  # 스캔 중심
    region_gradient_topic = LaunchConfiguration("region_gradient_topic")  # 스캔 그래디언트
    rolling_gradient_topic = LaunchConfiguration("rolling_gradient_topic")  # 롤링 그래디언트
    rc_override_topic = LaunchConfiguration("rc_override_topic")  # RC override 출력

    float_names = [
        ("arena_length_m", "15.0"),  # 수조 길이
        ("arena_width_m", "16.0"),  # 수조 너비
        ("arena_offset_x_m", "0.0"),  # 시작 좌표계 기준 수조 경계 X 오프셋
        ("arena_offset_y_m", "0.0"),  # 시작 좌표계 기준 수조 경계 Y 오프셋
        ("arena_safety_margin_m", "0.5"),  # 벽으로부터의 안전 여유
        ("initial_scan_radius_m", "1.5"),  # 최초 원형 스캔 반경
        ("rescan_radius_m", "0.7"),  # 재스캔 원형 반경
        ("homing_waypoint_step_m", "0.8"),  # 호밍 한 스텝 전진 거리
        ("homing_zigzag_offset_m", "0.2"),  # 호밍 좌우 지그재그 오프셋
        ("rolling_gradient_alpha", "0.15"),  # 롤링 그래디언트 방향 스무딩 비율
        ("rolling_gradient_conflict_angle_rad", "1.0472"),  # 방향 충돌 판정 각도 (60도)
        ("waypoint_reach_tolerance_m", "0.15"),  # waypoint 도착 판정 반경
        ("scan_radial_kp", "1.5"),  # 원형스캔 반경 오차 P 게인
        ("scan_radial_ki", "0.05"),  # 원형스캔 반경 오차 I 게인
        ("scan_radial_kd", "0.3"),  # 원형스캔 반경 오차 D 게인
        ("scan_radial_integral_limit", "1.0"),  # 반경 오차 적분 제한
        ("region_sample_spacing_m", "0.15"),  # SNR 샘플 최소 간격
        ("min_region_gradient_magnitude", "0.05"),  # 유효 그래디언트 최소 크기
        ("min_region_lateral_spread_m", "0.10"),  # 유효 피팅용 최소 횡방향 퍼짐
        ("vision_near_zone_width_m", "2.0"),  # 비전 인계용 근접 구간 폭
        ("acoustic_timeout_s", "90.0"),  # Acoustic 예산; 초과 시 Vision에 즉시 grant
        ("target_depth_z_m", "-2.05"),  # 목표 수심 (odom z)
        ("depth_tolerance_m", "0.10"),  # 목표 수심 도달 허용오차
        ("depth_kp", "0.8"),  # 수심 오차 P 게인
        ("depth_ki", "0.15"),  # 수심 오차 I 게인
        ("odometry_timeout_s", "0.5"),  # odometry 신선도 타임아웃
        ("max_snr_odom_skew_s", "0.15"),  # SNR-odometry 시각 허용 오차
        ("forward_cruise", "0.5"),  # 정렬 후 전진 RC 명령 크기
        ("yaw_kp", "1.15"),  # yaw 오차 비례 게인
        ("yaw_ki", "0.15"),  # 지속 yaw 오차 제거용 적분 게인
        ("yaw_kd", "0.08"),  # yaw 오차 변화 감쇠 게인
        ("yaw_integral_limit", "2.0"),  # yaw 적분 누적 제한 (rad*s)
        ("yaw_limit", "0.72"),  # yaw 명령 최대 크기
        ("move_heading_tolerance_rad", "0.1745"),  # 전진 허용 헤딩 오차
        ("vision_heading_tolerance_rad", "0.12"),  # Vision 확인 중 전진 허용 헤딩 오차
        ("rc_pwm_span", "400.0"),  # RC 중립±PWM 스팬
        ("rate_hz", "30.0"),  # 제어 루프 주기
    ]
    int_names = [
        ("homing_gradient_window_size", "12"),  # 롤링 그래디언트 윈도우 크기
        ("min_homing_gradient_samples", "8"),  # 롤링 그래디언트 피팅 최소 샘플 수
        ("rolling_gradient_conflict_limit", "3"),  # 큰 방향 충돌 연속 허용 횟수
    ]
    values = {name: LaunchConfiguration(name) for name, _ in float_names + int_names}

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),  # 시뮬 시계 사용 여부
        DeclareLaunchArgument(
            "odometry_topic", default_value="/odometry/filtered"
        ),  # 입력 odometry 토픽
        DeclareLaunchArgument(
            "start_frame_topic", default_value="/start_frame"
        ),
        DeclareLaunchArgument(
            "arena_frame_id", default_value="arena"
        ),
        DeclareLaunchArgument(
            "snr_topic", default_value="/audio_frequency_detector/snr_db_stamped"
        ),  # 입력 SNR 토픽
        DeclareLaunchArgument(
            "state_topic", default_value="/homing/control_state"
        ),  # 제어 상태 토픽
        DeclareLaunchArgument(
            "waypoint_topic", default_value="/homing/current_waypoint"
        ),  # 현재 목표 waypoint 토픽
        DeclareLaunchArgument(
            "scan_center_topic", default_value="/homing/scan_center"
        ),  # 원형 스캔 중심 토픽
        DeclareLaunchArgument(
            "vision_search_request_topic",
            default_value="/homing/vision_search_active",
        ),  # 비전 탐색 활성화 요청 토픽
        DeclareLaunchArgument(
            "target_confirmed_topic", default_value="/vision/target_confirmed"
        ),  # Vision 타깃 확정 응답 토픽
        # [ACOUSTIC-VISION HANDSHAKE] Vision은 이 승인 후에만 RC를 발행한다.
        DeclareLaunchArgument(
            "vision_control_granted_topic",
            default_value="/homing/vision_control_granted",
        ),  # 비전 RC 제어 승인 토픽
        DeclareLaunchArgument(
            "region_gradient_topic", default_value="/homing/region_gradient"
        ),  # 원형 스캔 그래디언트 토픽
        DeclareLaunchArgument(
            "rolling_gradient_topic", default_value="/homing/rolling_gradient"
        ),  # 호밍 중 롤링 그래디언트 토픽
        DeclareLaunchArgument(
            "homing_direction_topic", default_value="/homing/homing_direction"
        ),  # 실제 호밍 진행 방향 토픽
        DeclareLaunchArgument(
            "rc_override_topic", default_value="/mavros/rc/override"
        ),  # MAVROS RC override 출력 토픽
        DeclareLaunchArgument(
            "emergency_stop_topic", default_value="/mission/emergency_stop"
        ),
        DeclareLaunchArgument("enable_keyboard_emergency_stop", default_value="true"),
        DeclareLaunchArgument("emergency_stop_key", default_value="s"),
        DeclareLaunchArgument(
            "arena_start_corner",
            default_value="bottom_left",
            description=(
                "bottom_left (inside=-Y) or bottom_right (inside=+Y); "
                "initial heading is always arena +X."
            ),
        ),  # 수조 시작 코너 (안쪽 Y 부호 결정)
        DeclareLaunchArgument(
            "invert_rc_yaw", default_value="true"
        ),  # RC yaw 채널 부호 반전
        DeclareLaunchArgument(
            "invert_rc_lateral", default_value="true"
        ),  # ROS FLU +left를 ArduSub RC6 FRD +right로 변환
        DeclareLaunchArgument(
            "vision_handoff_enabled", default_value="true"
        ),  # 근접 구간에서 비전 인계 사용 여부
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in float_names + int_names
    ]

    common_topics = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "odometry_topic": odometry_topic,
        "start_frame_topic": start_frame_topic,
        "state_topic": state_topic,
        "region_gradient_topic": region_gradient_topic,
        "rolling_gradient_topic": rolling_gradient_topic,
        "arena_frame_id": LaunchConfiguration("arena_frame_id"),
    }
    estimator_parameters = {
        **common_topics,
        "snr_topic": snr_topic,
        "region_sample_spacing_m": ParameterValue(
            values["region_sample_spacing_m"], value_type=float
        ),
        "homing_gradient_window_size": ParameterValue(
            values["homing_gradient_window_size"], value_type=int
        ),
        "min_homing_gradient_samples": ParameterValue(
            values["min_homing_gradient_samples"], value_type=int
        ),
        "min_region_gradient_magnitude": ParameterValue(
            values["min_region_gradient_magnitude"], value_type=float
        ),
        "min_region_lateral_spread_m": ParameterValue(
            values["min_region_lateral_spread_m"], value_type=float
        ),
        "odometry_timeout_s": ParameterValue(
            values["odometry_timeout_s"], value_type=float
        ),
        "max_snr_odom_skew_s": ParameterValue(
            values["max_snr_odom_skew_s"], value_type=float
        ),
    }
    controller_parameters = {
        **common_topics,
        "waypoint_topic": waypoint_topic,
        "scan_center_topic": scan_center_topic,
        "vision_search_request_topic": LaunchConfiguration(
            "vision_search_request_topic"
        ),
        "target_confirmed_topic": LaunchConfiguration("target_confirmed_topic"),
        "vision_control_granted_topic": LaunchConfiguration(
            "vision_control_granted_topic"
        ),
        "homing_direction_topic": LaunchConfiguration("homing_direction_topic"),
        "rc_override_topic": rc_override_topic,
        "emergency_stop_topic": LaunchConfiguration("emergency_stop_topic"),
        "enable_keyboard_emergency_stop": ParameterValue(
            LaunchConfiguration("enable_keyboard_emergency_stop"), value_type=bool
        ),
        "emergency_stop_key": LaunchConfiguration("emergency_stop_key"),
        "arena_start_corner": LaunchConfiguration("arena_start_corner"),
        "invert_rc_yaw": ParameterValue(
            LaunchConfiguration("invert_rc_yaw"), value_type=bool
        ),
        "invert_rc_lateral": ParameterValue(
            LaunchConfiguration("invert_rc_lateral"), value_type=bool
        ),
        "vision_handoff_enabled": ParameterValue(
            LaunchConfiguration("vision_handoff_enabled"), value_type=bool
        ),
    }
    for name in [
        "arena_length_m",
        "arena_width_m",
        "arena_offset_x_m",
        "arena_offset_y_m",
        "arena_safety_margin_m",
        "initial_scan_radius_m",
        "rescan_radius_m",
        "homing_waypoint_step_m",
        "homing_zigzag_offset_m",
        "rolling_gradient_alpha",
        "rolling_gradient_conflict_angle_rad",
        "waypoint_reach_tolerance_m",
        "scan_radial_kp",
        "scan_radial_ki",
        "scan_radial_kd",
        "scan_radial_integral_limit",
        "vision_near_zone_width_m",
        "acoustic_timeout_s",
        "target_depth_z_m",
        "depth_tolerance_m",
        "depth_kp",
        "depth_ki",
        "odometry_timeout_s",
        "forward_cruise",
        "yaw_kp",
        "yaw_ki",
        "yaw_kd",
        "yaw_integral_limit",
        "yaw_limit",
        "move_heading_tolerance_rad",
        "vision_heading_tolerance_rad",
        "rc_pwm_span",
        "rate_hz",
    ]:
        controller_parameters[name] = ParameterValue(values[name], value_type=float)
    for name in [
        "rolling_gradient_conflict_limit",
    ]:
        controller_parameters[name] = ParameterValue(values[name], value_type=int)

    container = ComposableNodeContainer(
        name="region_local_gradient_homing_pipeline",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::AudioFrequencyDetectorNode",
                name="audio_frequency_detector",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
            ComposableNode(
                package="hydrophone_ctrl",
                plugin="audio_capture::RegionLocalGradientEstimatorNode",
                name="region_local_gradient_estimator",
                parameters=[estimator_parameters],
            ),
            ComposableNode(
                package="hydrophone_ctrl",
                plugin="audio_capture::WaypointHomingControllerNode",
                name="waypoint_homing_controller",
                parameters=[controller_parameters],
            ),
        ],
        output="screen",
    )
    return LaunchDescription(arguments + [container])
