"""ROS status publisher for real-start state."""

from __future__ import annotations

from sim.runtime.real_start_publisher_delivery import publish_and_log_real_start_status
from sim.runtime.real_start_publisher_factory import create_real_start_status_ros_publisher


class RealStartStatusPublisher:
    """ROS publisher wrapper for real-start state status payloads."""

    def __init__(self, publisher=None, string_type=None) -> None:
        self.publisher = publisher
        self.string_type = string_type
        self.logged_ok = False
        self.logged_released = False

    @classmethod
    def create(cls, ros_bridge) -> "RealStartStatusPublisher":
        """Create the status publisher if a ROS bridge node is available."""
        publisher, string_type = create_real_start_status_ros_publisher(ros_bridge)
        return cls(publisher=publisher, string_type=string_type)

    def publish(self, payload: dict[str, object]) -> None:
        """Publish and log the real-start status payload."""
        publish_and_log_real_start_status(self, payload)


__all__ = ["RealStartStatusPublisher"]
