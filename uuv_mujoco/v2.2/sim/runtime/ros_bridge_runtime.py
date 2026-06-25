"""Mutable ROS bridge lifecycle wrapper for the MuJoCo runner."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from sim.runtime.ros_bridge_runtime_failure import (
    disable_ros_bridge_after_failure,
    shutdown_ros_bridge_once,
)
from sim.runtime.ros_bridge_runtime_publish import (
    publish_qgc_video_once,
    publish_ros_bridge_once,
    spin_ros_bridge_once,
)


class RosBridgeRuntime:
    """Own optional ROS/SITL bridge publication, spin, and shutdown state."""

    def __init__(self, bridge: Any | None) -> None:
        self.bridge = bridge

    def get(self) -> Any | None:
        """Return the currently active bridge, if one is still enabled."""

        return self.bridge

    def shutdown(self) -> None:
        """Shutdown the active bridge exactly once."""

        shutdown_ros_bridge_once(self)

    def _disable_after_failure(self, *, label: str, exc: Exception) -> None:
        disable_ros_bridge_after_failure(self, label=label, exc=exc)

    def publish_once(
        self,
        *,
        data: Any,
        initial_depth_hold: Mapping[str, Any],
        real_start_status: Any,
    ) -> None:
        """Publish one ROS bridge sensor/status update with failure isolation."""

        publish_ros_bridge_once(
            self,
            data=data,
            initial_depth_hold=initial_depth_hold,
            real_start_status=real_start_status,
        )

    def publish_qgc_video_once(self, *, qgc_video: Any, data: Any) -> None:
        """Publish one optional QGC video frame using the active bridge if any."""

        publish_qgc_video_once(self, qgc_video=qgc_video, data=data)

    def spin_once(self) -> None:
        """Spin the active bridge once with the same failure policy as publish."""

        spin_ros_bridge_once(self)


__all__ = [
    "RosBridgeRuntime",
    "disable_ros_bridge_after_failure",
    "publish_qgc_video_once",
    "publish_ros_bridge_once",
    "shutdown_ros_bridge_once",
    "spin_ros_bridge_once",
]
