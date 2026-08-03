from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("node_name", default_value="yolo_buoy_detector"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
                description="Compressed camera topic received from the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "bbox_topic",
                default_value="/vision/buoy_bbox",
                description="BBox topic published back to the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "annotated_image_topic",
                default_value="/vision/yolo/annotated/compressed",
                description="Compressed image topic rendered from the completed YOLO inference frame.",
            ),
            DeclareLaunchArgument(
                "publish_annotated_image",
                default_value="true",
                description="Publish the completed YOLO inference frame with overlays.",
            ),
            DeclareLaunchArgument("annotated_jpeg_quality", default_value="80"),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Required .pt model path. Example: /home/user/models/yolo26m_underwater_batch4_last.pt",
            ),
            DeclareLaunchArgument(
                "target_class_name",
                default_value="",
                description="Target class name. Leave empty to accept every detected class.",
            ),
            DeclareLaunchArgument(
                "target_class_id",
                default_value="-1",
                description="Target class id. Overrides target_class_name when >= 0.",
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument(
                "device",
                default_value="auto",
                description="Inference device: auto, cpu, cuda:0, etc.",
            ),
            DeclareLaunchArgument("imgsz", default_value="640"),
            DeclareLaunchArgument(
                "show_preview",
                default_value="true",
                description="Show OpenCV preview window with live detections.",
            ),
            DeclareLaunchArgument(
                "preview_window_name",
                default_value="YOLO Buoy Detection",
                description="OpenCV window title for the preview UI.",
            ),
            DeclareLaunchArgument(
                "publish_per_class",
                default_value="true",
                description="Publish the best detection for every visible class in each frame.",
            ),
            DeclareLaunchArgument(
                "enable_topic",
                default_value="",
                description="Optional transient-local Bool topic that gates image inference.",
            ),
            DeclareLaunchArgument(
                "initially_enabled",
                default_value="true",
                description="Initial inference state before an enable-topic message arrives.",
            ),
            DeclareLaunchArgument(
                "ready_topic",
                default_value="",
                description="Optional transient-local Bool topic published after model setup.",
            ),
            DeclareLaunchArgument(
                "pinger_marker_fallback",
                default_value="false",
                description=(
                    "Use the competition underwater pinger blue marker only "
                    "when YOLO returns no detections."
                ),
            ),
            DeclareLaunchArgument("pinger_marker_disable_topic", default_value=""),
            DeclareLaunchArgument(
                "pinger_marker_target_id",
                default_value="course_buoy_pinger_white_1_float",
            ),
            DeclareLaunchArgument("course_buoy_color_filter", default_value="false"),
            DeclareLaunchArgument("course_buoy_min_confidence", default_value="0.0"),
            DeclareLaunchArgument("associate_stick_with_buoy", default_value="false"),
            Node(
                package="auv_buoy_vision_control",
                executable="yolo_buoy_detector",
                name=LaunchConfiguration("node_name"),
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "annotated_image_topic": LaunchConfiguration(
                            "annotated_image_topic"
                        ),
                        "publish_annotated_image": ParameterValue(
                            LaunchConfiguration("publish_annotated_image"), value_type=bool
                        ),
                        "annotated_jpeg_quality": ParameterValue(
                            LaunchConfiguration("annotated_jpeg_quality"), value_type=int
                        ),
                        "model_path": LaunchConfiguration("model_path"),
                        "target_class_name": LaunchConfiguration("target_class_name"),
                        "target_class_id": ParameterValue(LaunchConfiguration("target_class_id"), value_type=int),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "device": LaunchConfiguration("device"),
                        "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                        "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
                        "preview_window_name": LaunchConfiguration("preview_window_name"),
                        "publish_per_class": ParameterValue(
                            LaunchConfiguration("publish_per_class"), value_type=bool
                        ),
                        "enable_topic": LaunchConfiguration("enable_topic"),
                        "initially_enabled": ParameterValue(
                            LaunchConfiguration("initially_enabled"), value_type=bool
                        ),
                        "ready_topic": LaunchConfiguration("ready_topic"),
                        "pinger_marker_fallback": ParameterValue(
                            LaunchConfiguration("pinger_marker_fallback"),
                            value_type=bool,
                        ),
                        "pinger_marker_disable_topic": LaunchConfiguration(
                            "pinger_marker_disable_topic"
                        ),
                        "pinger_marker_target_id": LaunchConfiguration(
                            "pinger_marker_target_id"
                        ),
                        "course_buoy_color_filter": ParameterValue(
                            LaunchConfiguration("course_buoy_color_filter"),
                            value_type=bool,
                        ),
                        "course_buoy_min_confidence": ParameterValue(
                            LaunchConfiguration("course_buoy_min_confidence"),
                            value_type=float,
                        ),
                        "associate_stick_with_buoy": ParameterValue(
                            LaunchConfiguration("associate_stick_with_buoy"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
