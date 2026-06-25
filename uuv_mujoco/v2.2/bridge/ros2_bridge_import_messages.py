"""Lazy ROS message/service import helpers for the MuJoCo bridge."""

from __future__ import annotations


def load_ros2_message_imports() -> dict[str, object]:
    from geometry_msgs.msg import (
        PoseStamped,
        PoseWithCovarianceStamped,
        TransformStamped,
        TwistStamped,
        TwistWithCovarianceStamped,
    )
    from mavros_msgs.msg import ManualControl, OverrideRCIn, PositionTarget, RCIn, VfrHud
    from mavros_msgs.msg import State as MavrosState
    from mavros_msgs.srv import CommandBool as MavrosCommandBool
    from mavros_msgs.srv import CommandLong as MavrosCommandLong
    from mavros_msgs.srv import SetMode as MavrosSetMode
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState, FluidPressure, Image, Imu, LaserScan, Range
    from std_msgs.msg import Float32, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    return {
        "PoseStamped": PoseStamped,
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "TransformStamped": TransformStamped,
        "TwistStamped": TwistStamped,
        "TwistWithCovarianceStamped": TwistWithCovarianceStamped,
        "Odometry": Odometry,
        "BatteryState": BatteryState,
        "FluidPressure": FluidPressure,
        "Image": Image,
        "Imu": Imu,
        "LaserScan": LaserScan,
        "Range": Range,
        "Float32": Float32,
        "String": String,
        "Trigger": Trigger,
        "TFMessage": TFMessage,
        "ManualControl": ManualControl,
        "OverrideRCIn": OverrideRCIn,
        "RCIn": RCIn,
        "PositionTarget": PositionTarget,
        "VfrHud": VfrHud,
        "MavrosState": MavrosState,
        "MavrosCommandBool": MavrosCommandBool,
        "MavrosCommandLong": MavrosCommandLong,
        "MavrosSetMode": MavrosSetMode,
    }


def load_optional_ros2_message_imports() -> dict[str, object | None]:
    try:
        from mavros_msgs.msg import RCOut
    except Exception:
        RCOut = None

    try:
        from dvl_msgs.msg import DVL as DVLMsg
        from dvl_msgs.msg import DVLDR as DVLDRMsg
    except Exception:
        DVLMsg = None
        DVLDRMsg = None

    try:
        from ping360_sonar_msgs.msg import SonarEcho
    except Exception:
        SonarEcho = None

    return {
        "RCOut": RCOut,
        "DVLMsg": DVLMsg,
        "DVLDRMsg": DVLDRMsg,
        "SonarEcho": SonarEcho,
    }


__all__ = ["load_optional_ros2_message_imports", "load_ros2_message_imports"]
