from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_model_path() -> str:
    """설정값이 없으면 현재 저장소의 대회용 YOLO 모델을 찾는다."""
    configured = os.environ.get("UUV_YOLO_MODEL")
    if configured:
        return configured

    relative_model = Path("sim/current/assets/yolo/best.pt")
    for root in (Path.cwd(), *Path(__file__).resolve().parents):
        candidate = root / relative_model
        if candidate.is_file():
            return str(candidate)
    return str(Path.cwd() / relative_model)


def _detector_path() -> str:
    """ultralytics가 설치된 프로젝트 가상환경을 detector PATH 앞에 둔다."""
    configured = os.environ.get("UUV_YOLO_VENV")
    candidates = []
    if configured:
        candidates.append(Path(configured))
    candidates.extend(
        [
            Path.home() / "venvs" / "yolo26",
            Path.home() / ".venvs" / "uuv_mujoco_desktop",
            Path.home() / ".venvs" / "yolo26",
        ]
    )

    for venv in candidates:
        python = venv / "bin" / "python3"
        site_packages = tuple((venv / "lib").glob("python*/site-packages/ultralytics"))
        if python.is_file() and site_packages:
            return f"{venv / 'bin'}:{os.environ.get('PATH', '')}"
    return os.environ.get("PATH", "")


def generate_launch_description() -> LaunchDescription:
    detector_launch = os.path.join(
        get_package_share_directory("auv_buoy_vision_control"),
        "launch",
        "laptop_yolo_detection.launch.py",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    include_detectors = LaunchConfiguration("include_detectors")
    standalone_test = LaunchConfiguration("standalone_test")

    common_detector_arguments = {
        "model_path": LaunchConfiguration("model_path"),
        "target_class_id": "0",
        "target_class_name": "",
        "confidence_threshold": LaunchConfiguration("confidence_threshold"),
        "device": LaunchConfiguration("device"),
        "imgsz": "640",
        "show_preview": LaunchConfiguration("show_preview"),
        "publish_per_class": "true",
        "initially_enabled": "false",
        "pinger_marker_fallback": "false",
        "course_buoy_color_filter": "false",
        "associate_stick_with_buoy": "false",
    }

    return LaunchDescription(
        [
            # yolo_buoy_detector의 /usr/bin/env python3가 ultralytics 가상환경을
            # 자동 선택하도록 모든 detector include보다 먼저 PATH를 설정한다.
            SetEnvironmentVariable("PATH", _detector_path()),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("include_detectors", default_value="true"),
            DeclareLaunchArgument("standalone_test", default_value="true"),
            DeclareLaunchArgument("require_armed", default_value="true"),
            DeclareLaunchArgument("model_path", default_value=_default_model_path()),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("show_preview", default_value="false"),
            DeclareLaunchArgument(
                "surface_align_deadband_x",
                default_value="0.07",
                description="정면 bbox 중심 허용오차(정규화 화면 폭, 0.07=7%).",
            ),
            DeclareLaunchArgument(
                "surface_capture_min_bbox_height_ratio",
                default_value="0.10",
                description="포획 상태 진입 최소 bbox 높이 비율(기본 10%).",
            ),
            DeclareLaunchArgument(
                "surface_waypoint_yaw_kp_pwm_per_rad", default_value="450.0"
            ),
            DeclareLaunchArgument(
                "surface_align_yaw_kp_pwm_per_normalized_x", default_value="500.0"
            ),
            DeclareLaunchArgument(
                "surface_yaw_command_deadband_rad", default_value="0.03"
            ),
            DeclareLaunchArgument("surface_min_yaw_delta_pwm", default_value="55"),
            DeclareLaunchArgument("surface_max_yaw_delta_pwm", default_value="200"),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/sim/odom",
                description="수면 단독 시험은 시뮬레이터 world odometry를 직접 사용한다.",
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
                "front_image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "top_image_topic",
                default_value="/camera/top/color/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "front_bbox_topic", default_value="/vision/surface/front/buoy_bbox"
            ),
            DeclareLaunchArgument(
                "top_bbox_topic", default_value="/vision/surface/top/buoy_bbox"
            ),
            DeclareLaunchArgument(
                "front_enable_topic", default_value="/vision/surface/front/enabled"
            ),
            DeclareLaunchArgument(
                "top_enable_topic", default_value="/vision/surface/top/enabled"
            ),
            DeclareLaunchArgument(
                "front_ready_topic", default_value="/vision/surface/front/ready"
            ),
            DeclareLaunchArgument(
                "top_ready_topic", default_value="/vision/surface/top/ready"
            ),
            DeclareLaunchArgument(
                "score_release_topic", default_value="/mission/score_release"
            ),
            DeclareLaunchArgument(
                "rc_override_topic", default_value="/mavros/rc/override"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
                condition=IfCondition(include_detectors),
                launch_arguments={
                    **common_detector_arguments,
                    "node_name": "surface_front_yolo_detector",
                    "image_topic": LaunchConfiguration("front_image_topic"),
                    "bbox_topic": LaunchConfiguration("front_bbox_topic"),
                    "enable_topic": LaunchConfiguration("front_enable_topic"),
                    "ready_topic": LaunchConfiguration("front_ready_topic"),
                    "annotated_image_topic": (
                        "/vision/surface/front/annotated/compressed"
                    ),
                    "publish_annotated_image": "true",
                    "preview_window_name": "Surface Front YOLO",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
                condition=IfCondition(include_detectors),
                launch_arguments={
                    **common_detector_arguments,
                    "node_name": "surface_top_yolo_detector",
                    "image_topic": LaunchConfiguration("top_image_topic"),
                    "bbox_topic": LaunchConfiguration("top_bbox_topic"),
                    "enable_topic": LaunchConfiguration("top_enable_topic"),
                    "ready_topic": LaunchConfiguration("top_ready_topic"),
                    "annotated_image_topic": (
                        "/vision/surface/top/annotated/compressed"
                    ),
                    "publish_annotated_image": "true",
                    "preview_window_name": "Surface Top YOLO",
                }.items(),
            ),
            Node(
                package="kmu26_auv_surface_buoy_mission",
                executable="surface_test_starter_node",
                name="surface_test_starter_node",
                output="screen",
                condition=IfCondition(standalone_test),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "odometry_topic": LaunchConfiguration("odometry_topic"),
                        "depth_pose_topic": LaunchConfiguration("depth_pose_topic"),
                        "front_ready_topic": LaunchConfiguration("front_ready_topic"),
                        "top_ready_topic": LaunchConfiguration("top_ready_topic"),
                        "start_frame_topic": LaunchConfiguration("start_frame_topic"),
                        "arena_config_topic": LaunchConfiguration(
                            "arena_config_topic"
                        ),
                        "surface_start_topic": LaunchConfiguration(
                            "surface_start_topic"
                        ),
                        "require_vision": ParameterValue(
                            include_detectors, value_type=bool
                        ),
                        "require_armed": ParameterValue(
                            LaunchConfiguration("require_armed"), value_type=bool
                        ),
                    }
                ],
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
                        "surface_front_enable_topic": LaunchConfiguration(
                            "front_enable_topic"
                        ),
                        "surface_top_enable_topic": LaunchConfiguration(
                            "top_enable_topic"
                        ),
                        "score_release_topic": LaunchConfiguration(
                            "score_release_topic"
                        ),
                        "rc_override_topic": LaunchConfiguration("rc_override_topic"),
                        "surface_align_deadband_x": ParameterValue(
                            LaunchConfiguration("surface_align_deadband_x"),
                            value_type=float,
                        ),
                        "surface_capture_min_bbox_height_ratio": ParameterValue(
                            LaunchConfiguration(
                                "surface_capture_min_bbox_height_ratio"
                            ),
                            value_type=float,
                        ),
                        "surface_waypoint_yaw_kp_pwm_per_rad": ParameterValue(
                            LaunchConfiguration("surface_waypoint_yaw_kp_pwm_per_rad"),
                            value_type=float,
                        ),
                        "surface_align_yaw_kp_pwm_per_normalized_x": ParameterValue(
                            LaunchConfiguration(
                                "surface_align_yaw_kp_pwm_per_normalized_x"
                            ),
                            value_type=float,
                        ),
                        "surface_yaw_command_deadband_rad": ParameterValue(
                            LaunchConfiguration("surface_yaw_command_deadband_rad"),
                            value_type=float,
                        ),
                        "surface_min_yaw_delta_pwm": ParameterValue(
                            LaunchConfiguration("surface_min_yaw_delta_pwm"),
                            value_type=int,
                        ),
                        "surface_max_yaw_delta_pwm": ParameterValue(
                            LaunchConfiguration("surface_max_yaw_delta_pwm"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
