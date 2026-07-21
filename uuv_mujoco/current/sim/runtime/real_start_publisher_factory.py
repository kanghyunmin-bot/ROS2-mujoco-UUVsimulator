"""Factory helpers for the real-start status ROS publisher."""

from __future__ import annotations


def create_real_start_status_ros_publisher(ros_bridge):
    if ros_bridge is None or getattr(ros_bridge, "node", None) is None:
        return None, None
    try:
        from std_msgs.msg import String

        publisher = ros_bridge.node.create_publisher(
            String,
            "/mujoco/real_start_state/status",
            10,
        )
        return publisher, String
    except Exception as exc:
        print(f"[ros2] real start status publisher unavailable: {exc}", flush=True)
        return None, None


__all__ = ["create_real_start_status_ros_publisher"]
