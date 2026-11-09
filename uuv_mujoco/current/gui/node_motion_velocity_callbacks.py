"""Velocity callbacks for UuvGuiNode."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from geometry_msgs.msg import TwistWithCovarianceStamped


def _on_dvl_twist(self, msg: TwistWithCovarianceStamped) -> None:
    """Display DVL velocity [m/s] in the message's declared coordinate frame."""
    linear = msg.twist.twist.linear
    velocity = (float(linear.x), float(linear.y), float(linear.z))
    if not all(math.isfinite(value) for value in velocity):
        return
    with self._lock:
        self._snapshot.velocity_xyz = velocity
        self._snapshot.velocity_source = "/dvl/twist"
        self._snapshot.velocity_frame = msg.header.frame_id
        self._touch("velocity")
        self._touch("dvl_velocity")


def _on_velocity(self, msg: TwistStamped, source: str) -> None:
    with self._lock:
        if time.monotonic() - self._last_wall.get("dvl_velocity", -math.inf) < 1.0:
            return
        self._snapshot.velocity_xyz = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        )
        self._snapshot.velocity_source = source
        self._snapshot.velocity_frame = msg.header.frame_id
        self._touch("velocity")


def _on_velocity_body(self, msg: TwistStamped) -> None:
    self._on_velocity(msg, self._topic("local_position/velocity_body"))


def _on_velocity_local(self, msg: TwistStamped) -> None:
    self._on_velocity(msg, self._topic("local_position/velocity_local"))


def _on_dvl_velocity(self, msg: TwistStamped) -> None:
    self._on_velocity(msg, "/dvl/velocity")


__all__ = ["_on_dvl_twist", "_on_dvl_velocity", "_on_velocity", "_on_velocity_body", "_on_velocity_local"]
