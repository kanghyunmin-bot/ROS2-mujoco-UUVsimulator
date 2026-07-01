"""ROS2 publisher endpoint construction."""

from __future__ import annotations

from .ros2_endpoint_core_publishers import create_core_publishers
from .ros2_endpoint_mavros_publishers import create_mavros_publishers
from .ros2_endpoint_misc_publishers import create_dvl_compat_publishers, create_tf_publishers
from .ros2_endpoint_ping360_publishers import create_ping360_publishers
from .ros2_stereo_image import create_stereo_image_publishers


def create_ros2_publishers(bridge, *, q10, q1, tf_qos, latched_qos) -> None:
    create_core_publishers(bridge, q10=q10)
    create_ping360_publishers(bridge, q10=q10, q1=q1)
    create_stereo_image_publishers(bridge, q1=q1)
    create_mavros_publishers(bridge, q10=q10)
    create_dvl_compat_publishers(bridge, q10=q10)
    create_tf_publishers(bridge, tf_qos=tf_qos, latched_qos=latched_qos)


__all__ = [
    "create_core_publishers",
    "create_mavros_publishers",
    "create_ping360_publishers",
    "create_stereo_image_publishers",
    "create_dvl_compat_publishers",
    "create_tf_publishers",
    "create_ros2_publishers",
]
