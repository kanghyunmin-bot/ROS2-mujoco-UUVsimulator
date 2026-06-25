"""Core ROS2 GUI imports and optional rosbag/std_srvs bindings."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, FluidPressure, Imu
from std_msgs.msg import Float32, String

try:
    from std_srvs.srv import Trigger

    HAVE_STD_SRVS = True
except ModuleNotFoundError:
    HAVE_STD_SRVS = False

    class Trigger:
        class Request:
            pass

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    HAVE_ROSBAG2_PY = True
except ModuleNotFoundError:
    HAVE_ROSBAG2_PY = False
    rosbag2_py = None
    deserialize_message = None


__all__ = [
    "BatteryState",
    "DurabilityPolicy",
    "Float32",
    "FluidPressure",
    "HAVE_ROSBAG2_PY",
    "HAVE_STD_SRVS",
    "HistoryPolicy",
    "Imu",
    "MultiThreadedExecutor",
    "Node",
    "Odometry",
    "PoseStamped",
    "QoSProfile",
    "ReliabilityPolicy",
    "String",
    "Trigger",
    "TwistStamped",
    "deserialize_message",
    "qos_profile_sensor_data",
    "rclpy",
    "rosbag2_py",
]
