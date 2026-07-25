"""Lazy ROS2 imports for the ALT_HOLD diagnostics logger."""

from __future__ import annotations

from types import SimpleNamespace


def load_ros_symbols() -> SimpleNamespace:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import ManualControl, RCIn, RCOut
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float32, String
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "ROS2/mavros Python modules are not available. Source the same "
            "ROS2 environment used by the simulator, then rerun this tool."
        ) from exc
    return SimpleNamespace(
        rclpy=rclpy,
        PoseStamped=PoseStamped,
        TwistStamped=TwistStamped,
        ManualControl=ManualControl,
        RCIn=RCIn,
        RCOut=RCOut,
        Odometry=Odometry,
        Node=Node,
        qos_profile_sensor_data=qos_profile_sensor_data,
        Float32=Float32,
        String=String,
    )
