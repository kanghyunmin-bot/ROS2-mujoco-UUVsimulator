"""MAVROS RC override mirror for GUI and diagnostics feedback."""

from __future__ import annotations

from collections.abc import Sequence

from .ros2_rc_override_frame import rc_override_forward_frame


def _mirror_rc_override_to_rc_in(self, channels: Sequence[int]) -> None:
    try:
        rc_in = self.RCIn()
        header = getattr(rc_in, "header", None)
        if header is not None and self.node is not None:
            header.stamp = self.node.get_clock().now().to_msg()
            if hasattr(header, "frame_id"):
                header.frame_id = "fcu"
        rc_in.channels = rc_override_forward_frame(channels)
        self._mavros_last_rc_override = rc_in
        # Publish the mirror immediately as well as through the periodic bridge
        # loop so GUI feedback is not one bridge cycle behind command forwarding.
        self._safe_publish(self.pub_mavros_rc_in, rc_in, "/mavros/rc/in")
    except Exception:
        self._mavros_last_rc_override = None


__all__ = ["_mirror_rc_override_to_rc_in"]
