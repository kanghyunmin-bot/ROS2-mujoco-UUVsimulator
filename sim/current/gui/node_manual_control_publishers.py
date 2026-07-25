"""MANUAL_CONTROL publisher for UuvGuiNode."""

from __future__ import annotations

from .helpers import clamp_axis
from .runtime import ManualControl


def publish_manual_control(
    self,
    *,
    yaw: float,
    heave: float,
    forward: float,
    lateral: float,
) -> None:
    if self._manual_control_pub is None:
        return
    msg = ManualControl()
    msg.x = clamp_axis(forward)
    msg.y = clamp_axis(lateral)
    # The local bridge accepts normalized heave [-1, +1] and converts it to
    # MAVLink MANUAL_CONTROL z [0, 1000] with 500 as neutral.
    msg.z = clamp_axis(heave)
    msg.r = clamp_axis(yaw)
    msg.buttons = 0
    self._manual_control_pub.publish(msg)


__all__ = ["publish_manual_control"]
