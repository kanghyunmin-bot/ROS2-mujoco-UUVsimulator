"""Lazy ROS2 import loading for the MuJoCo bridge."""

from __future__ import annotations

from .ros2_bridge_runtime import optional_message_type
from .ros2_bridge_import_core import load_ros2_core_imports
from .ros2_bridge_import_messages import load_optional_ros2_message_imports, load_ros2_message_imports


def load_ros2_runtime_imports() -> dict[str, object]:
    try:
        core_imports = load_ros2_core_imports()
        message_imports = load_ros2_message_imports()
        optional_imports = load_optional_ros2_message_imports()
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 packages not found. Install rclpy + sensor_msgs + geometry_msgs + nav_msgs + mavros_msgs."
        ) from exc

    log_optional = lambda text: print(text, flush=True)
    return {
        **core_imports,
        **message_imports,
        "RCOut": optional_message_type("mavros_msgs/RCOut", optional_imports["RCOut"], log=log_optional),
        "DVLMsg": optional_message_type("dvl_msgs/DVL", optional_imports["DVLMsg"], log=log_optional),
        "DVLDRMsg": optional_message_type("dvl_msgs/DVLDR", optional_imports["DVLDRMsg"], log=log_optional),
        "SonarEcho": optional_message_type("ping360_sonar_msgs/SonarEcho", optional_imports["SonarEcho"], log=log_optional),
    }


__all__ = ["load_ros2_runtime_imports"]
