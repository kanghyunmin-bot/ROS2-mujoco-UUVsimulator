"""DVL compatibility and TF ROS2 publisher endpoint construction."""

from __future__ import annotations


def create_dvl_compat_publishers(bridge, *, q10) -> None:
    node = bridge.node
    bridge.pub_dvl_data = node.create_publisher(bridge.DVLMsg, "/dvl/data", q10) if bridge.DVLMsg else None
    bridge.pub_dvl_position = node.create_publisher(bridge.DVLDRMsg, "/dvl/position", q10) if bridge.DVLDRMsg else None


def create_tf_publishers(bridge, *, tf_qos, latched_qos) -> None:
    node = bridge.node
    bridge.pub_tf = node.create_publisher(bridge.TFMessage, "/tf", tf_qos)
    bridge.pub_tf_static = node.create_publisher(bridge.TFMessage, "/tf_static", latched_qos)
    bridge.pub_robot_description = node.create_publisher(bridge.String, "/robot_description", latched_qos)


__all__ = ["create_dvl_compat_publishers", "create_tf_publishers"]
