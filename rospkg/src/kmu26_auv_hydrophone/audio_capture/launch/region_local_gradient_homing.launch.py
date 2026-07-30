from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# 실험 수조버전 런치
# ros2 launch audio_capture region_local_gradient_homing.launch.py \
#   use_sim_time:=false \
#   arena_start_corner:=bottom_left \
#   arena_length_m:=5.49 \
#   arena_width_m:=2.74 \
#   arena_offset_x_m:=-0.30 \
#   arena_offset_y_m:=0.30 \ (bottom_left면 양수로, bottom_right면 음수로)
#   arena_safety_margin_m:=0.30 \
#   initial_scan_radius_m:=1.00 \
#   rescan_radius_m:=0.50 \
#   homing_waypoint_step_m:=0.50 \
#   homing_zigzag_offset_m:=0.15 \
#   vision_near_zone_width_m:=0.60


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")  # 시뮬 시계
    odometry_topic = LaunchConfiguration("odometry_topic")  # odometry 입력
    snr_topic = LaunchConfiguration("snr_topic")  # SNR 입력
    state_topic = LaunchConfiguration("state_topic")  # 제어 상태
    waypoint_topic = LaunchConfiguration("waypoint_topic")  # 현재 waypoint
    scan_center_topic = LaunchConfiguration("scan_center_topic")  # 스캔 중심
    region_gradient_topic = LaunchConfiguration("region_gradient_topic")  # 스캔 그래디언트
    rolling_gradient_topic = LaunchConfiguration("rolling_gradient_topic")  # 롤링 그래디언트

    float_names = [
        ("arena_length_m", "15.0"),  # 수조 길이
        ("arena_width_m", "16.0"),  # 수조 너비
        ("arena_offset_x_m", "-0.30"),  # 수조 원점 X 오프셋
        ("arena_offset_y_m", "0.30"),  # 수조 원점 Y 오프셋
        ("arena_safety_margin_m", "0.5"),  # 벽으로부터의 안전 여유
        ("initial_scan_radius_m", "1.5"),  # 최초 원형 스캔 반경
        ("rescan_radius_m", "0.7"),  # 재스캔 원형 반경
        ("homing_waypoint_step_m", "0.8"),  # 호밍 한 스텝 전진 거리
        ("homing_zigzag_offset_m", "0.2"),  # 호밍 좌우 지그재그 오프셋
        ("rolling_gradient_alpha", "0.15"),  # 롤링 그래디언트 방향 스무딩 비율
        ("rolling_gradient_conflict_angle_rad", "1.0472"),  # 방향 충돌 판정 각도 (60도)
        ("waypoint_reach_tolerance_m", "0.15"),  # waypoint 도착 판정 반경
        ("scan_waypoint_lookahead_rad", "0.35"),  # 현재 각도보다 앞선 원주 목표각
        ("region_sample_spacing_m", "0.15"),  # SNR 샘플 최소 간격
        ("min_region_gradient_magnitude", "0.05"),  # 유효 그래디언트 최소 크기
        ("min_region_lateral_spread_m", "0.10"),  # 유효 피팅용 최소 횡방향 퍼짐
        ("vision_near_zone_width_m", "2.0"),  # 비전 인계용 근접 구간 폭
        ("target_depth_z_m", "-0.10"),  # 목표 수심 (odom z)
        ("odometry_timeout_s", "0.5"),  # odometry 신선도 타임아웃
        ("fcu_state_timeout_s", "1.0"),  # FCU 상태 신선도 타임아웃
        ("handoff_hold_sec", "0.7"),  # 인계 전 정지 상태 유지 시간
        ("handoff_max_speed_mps", "0.2"),  # 인계 허용 최대 속도
        ("mode_request_interval_s", "1.0"),  # STABILIZE 재요청 간격
        ("max_snr_odom_skew_s", "0.15"),  # SNR-odometry 시각 허용 오차
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
            "snr_topic", default_value="/audio_frequency_detector/snr_db_stamped"
        ),  # 입력 SNR 토픽
        DeclareLaunchArgument(
            "state_topic", default_value="/homing/control_state"
        ),  # 제어 상태 토픽
        DeclareLaunchArgument(
            "waypoint_topic", default_value="/waypoint"
        ),  # odom 절대좌표 PositionTarget waypoint 토픽
        DeclareLaunchArgument(
            "arena_start_frame_topic", default_value="/guided/start_frame"
        ),  # arena 원점과 +X 방향을 담은 odom pose
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
            "guided_waypoint_enable_topic",
            default_value="/guided/waypoint_enable",
        ),  # 외부 waypoint 제어 활성화 토픽
        DeclareLaunchArgument(
            "guided_status_topic", default_value="/guided/status"
        ),  # 외부 waypoint 제어기 상태 토픽
        DeclareLaunchArgument(
            "fcu_state_topic", default_value="/mavros/state"
        ),  # FCU 모드 확인 토픽
        DeclareLaunchArgument(
            "set_mode_service", default_value="/mavros/set_mode"
        ),  # FCU 모드 변경 서비스
        DeclareLaunchArgument(
            "vision_mode_name", default_value="STABILIZE"
        ),  # 비전 RC 제어용 FCU 모드
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
        "state_topic": state_topic,
        "region_gradient_topic": region_gradient_topic,
        "rolling_gradient_topic": rolling_gradient_topic,
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
        "arena_start_frame_topic": LaunchConfiguration(
            "arena_start_frame_topic"
        ),
        "scan_center_topic": scan_center_topic,
        "vision_search_request_topic": LaunchConfiguration(
            "vision_search_request_topic"
        ),
        "target_confirmed_topic": LaunchConfiguration("target_confirmed_topic"),
        "vision_control_granted_topic": LaunchConfiguration(
            "vision_control_granted_topic"
        ),
        "guided_waypoint_enable_topic": LaunchConfiguration(
            "guided_waypoint_enable_topic"
        ),
        "guided_status_topic": LaunchConfiguration("guided_status_topic"),
        "fcu_state_topic": LaunchConfiguration("fcu_state_topic"),
        "set_mode_service": LaunchConfiguration("set_mode_service"),
        "vision_mode_name": LaunchConfiguration("vision_mode_name"),
        "homing_direction_topic": LaunchConfiguration("homing_direction_topic"),
        "emergency_stop_topic": LaunchConfiguration("emergency_stop_topic"),
        "enable_keyboard_emergency_stop": ParameterValue(
            LaunchConfiguration("enable_keyboard_emergency_stop"), value_type=bool
        ),
        "emergency_stop_key": LaunchConfiguration("emergency_stop_key"),
        "arena_start_corner": LaunchConfiguration("arena_start_corner"),
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
        "scan_waypoint_lookahead_rad",
        "vision_near_zone_width_m",
        "target_depth_z_m",
        "odometry_timeout_s",
        "fcu_state_timeout_s",
        "handoff_hold_sec",
        "handoff_max_speed_mps",
        "mode_request_interval_s",
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
                package="audio_capture",
                plugin="audio_capture::RegionLocalGradientEstimatorNode",
                name="region_local_gradient_estimator",
                parameters=[estimator_parameters],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::WaypointHomingControllerNode",
                name="waypoint_homing_controller",
                parameters=[controller_parameters],
            ),
        ],
        output="screen",
    )
    return LaunchDescription(arguments + [container])
