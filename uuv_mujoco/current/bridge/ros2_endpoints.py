"""ROS2 endpoint construction facade."""

from __future__ import annotations

from .ros2_endpoint_publishers import create_ros2_publishers
from .ros2_endpoint_services import create_ros2_services
from .ros2_endpoint_subscriptions import create_ros2_subscriptions


def create_ros2_endpoints(
    bridge,
    *,
    q10,
    q1,
    tf_qos,
    dvl_sensor_qos,
    latched_qos,
) -> None:
    """Create ROS2 endpoints and attach them to a Ros2Bridge instance."""
    create_ros2_publishers(
        bridge,
        q10=q10,
        q1=q1,
        tf_qos=tf_qos,
        dvl_sensor_qos=dvl_sensor_qos,
        latched_qos=latched_qos,
    )
    create_ros2_subscriptions(bridge, q10=q10)
    create_ros2_services(bridge)


__all__ = ["create_ros2_endpoints"]
