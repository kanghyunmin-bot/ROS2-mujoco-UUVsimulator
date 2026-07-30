import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _default_model_path() -> str:
    configured = os.environ.get("UUV_YOLO_MODEL")
    if configured:
        return configured

    relative_model = Path("sim/current/assets/yolo/best.pt")
    for root in (Path.cwd(), *Path(__file__).resolve().parents):
        candidate = root / relative_model
        if candidate.is_file():
            return str(candidate)
    return str(Path.cwd() / relative_model)


def generate_launch_description():
    package_share = get_package_share_directory("auv_buoy_vision_control")
    detector_launch = os.path.join(
        package_share, "launch", "laptop_yolo_detection.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=_default_model_path(),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("show_preview", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
                launch_arguments={
                    "model_path": LaunchConfiguration("model_path"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "bbox_topic": "/vision/buoy_bbox",
                    "annotated_image_topic": (
                        "/vision/yolo/underwater_pinger/annotated/compressed"
                    ),
                    "target_class_id": "-1",
                    "target_class_name": "",
                    "publish_per_class": "true",
                    "pinger_marker_fallback": "true",
                    "pinger_marker_disable_topic": (
                        "/vision/course_buoy_detached_id"
                    ),
                    "pinger_marker_target_id": (
                        "course_buoy_pinger_white_1_float"
                    ),
                    "course_buoy_color_filter": "true",
                    # Reject the intermittent 0.39-0.40, ~24x29 px pool-edge
                    # float false positive without raising the stick threshold.
                    "course_buoy_min_confidence": "0.45",
                    "associate_stick_with_buoy": "true",
                    "publish_annotated_image": "true",
                    "device": LaunchConfiguration("device"),
                    "show_preview": LaunchConfiguration("show_preview"),
                }.items(),
            ),
        ]
    )
