"""Lazy ROS message/service import helpers for the MuJoCo bridge."""

from __future__ import annotations


def load_ros2_message_imports() -> dict[str, object]:
    from geometry_msgs.msg import (
        PoseStamped,
        PoseWithCovarianceStamped,
        TransformStamped,
        TwistStamped,
        TwistWithCovarianceStamped,
        Vector3Stamped,
    )
    from mavros_msgs.msg import ManualControl, OverrideRCIn, PositionTarget, RCIn, VfrHud
    from mavros_msgs.msg import State as MavrosState
    from mavros_msgs.srv import CommandBool as MavrosCommandBool
    from mavros_msgs.srv import CommandLong as MavrosCommandLong
    from mavros_msgs.srv import SetMode as MavrosSetMode
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, FluidPressure, Image, Imu, LaserScan, Range
    from std_msgs.msg import Float32, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    return {
        "PoseStamped": PoseStamped,
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "TransformStamped": TransformStamped,
        "TwistStamped": TwistStamped,
        "TwistWithCovarianceStamped": TwistWithCovarianceStamped,
        "Vector3Stamped": Vector3Stamped,
        "Odometry": Odometry,
        "Clock": Clock,
        "BatteryState": BatteryState,
        "CompressedImage": CompressedImage,
        "CameraInfo": CameraInfo,
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
        from auv_dvl_a50_msg.msg import DVL as DVLMsg
        from auv_dvl_a50_msg.msg import DVLDR as DVLDRMsg

        DVLPackage = "auv_dvl_a50_msg"
    except Exception:
        try:
            # Older local workspaces used the upstream package's historical
            # name. Keep it as a compatibility fallback while preferring the
            # package used by the physical KMU robot.
            from dvl_msgs.msg import DVL as DVLMsg
            from dvl_msgs.msg import DVLDR as DVLDRMsg

            DVLPackage = "dvl_msgs"
        except Exception:
            DVLMsg = None
            DVLDRMsg = None
            DVLPackage = None

    try:
        from ping360_sonar_msgs.msg import SonarEcho
    except Exception:
        SonarEcho = None

    try:
        from audio_common_msgs.msg import AudioData, AudioInfo
    except Exception:
        AudioData = None
        AudioInfo = None

    try:
        from hit25_auv_ros2_msg.msg import CollectorState
    except Exception:
        CollectorState = None

    return {
        "RCOut": RCOut,
        "DVLMsg": DVLMsg,
        "DVLDRMsg": DVLDRMsg,
        "DVLPackage": DVLPackage,
        "SonarEcho": SonarEcho,
        "AudioData": AudioData,
        "AudioInfo": AudioInfo,
        "CollectorState": CollectorState,
    }


__all__ = ["load_optional_ros2_message_imports", "load_ros2_message_imports"]
