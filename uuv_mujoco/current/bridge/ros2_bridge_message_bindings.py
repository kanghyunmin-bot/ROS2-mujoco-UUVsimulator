"""Bind lazily loaded ROS2 message/service imports to the bridge object."""

from __future__ import annotations


ROS2_IMPORT_ATTRS = (
    "rclpy",
    "SingleThreadedExecutor",
    "PoseStamped",
    "PoseWithCovarianceStamped",
    "TransformStamped",
    "TwistStamped",
    "TwistWithCovarianceStamped",
    "Vector3Stamped",
    "Odometry",
    "Clock",
    "QoSProfile",
    "DurabilityPolicy",
    "HistoryPolicy",
    "ReliabilityPolicy",
    "BatteryState",
    "CameraInfo",
    "CompressedImage",
    "FluidPressure",
    "Image",
    "Imu",
    "LaserScan",
    "Range",
    "Float32",
    "String",
    "Trigger",
    "TFMessage",
    "ManualControl",
    "OverrideRCIn",
    "RCIn",
    "PositionTarget",
    "RCOut",
    "VfrHud",
    "MavrosState",
    "MavrosCommandBool",
    "MavrosCommandLong",
    "MavrosSetMode",
    "DVLMsg",
    "DVLDRMsg",
    "DVLPackage",
    "SonarEcho",
    "AudioData",
    "AudioInfo",
    "CollectorState",
)


def bind_ros2_runtime_imports(self, imports: dict[str, object]) -> None:
    for attr in ROS2_IMPORT_ATTRS:
        setattr(self, attr, imports[attr])


__all__ = ["ROS2_IMPORT_ATTRS", "bind_ros2_runtime_imports"]
