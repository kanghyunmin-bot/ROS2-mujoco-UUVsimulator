"""Lazy core ROS2 import helpers for the MuJoCo bridge."""

from __future__ import annotations


def load_ros2_core_imports() -> dict[str, object]:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    try:
        from rclpy.signals import SignalHandlerOptions
    except Exception:
        SignalHandlerOptions = None

    return {
        "rclpy": rclpy,
        "SingleThreadedExecutor": SingleThreadedExecutor,
        "Node": Node,
        "Context": Context,
        "SignalHandlerOptions": SignalHandlerOptions,
        "QoSProfile": QoSProfile,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
        "ReliabilityPolicy": ReliabilityPolicy,
    }


__all__ = ["load_ros2_core_imports"]
