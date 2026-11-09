"""Top-level MAVROS RC override callback for Ros2Bridge."""

from __future__ import annotations

from .ros2_rc_override_forward_cache import store_pending_rc_override
from .ros2_rc_override_frame import normalized_rc_override_axes, rc_override_channels_from_msg
from .ros2_rc_override_mirror import _mirror_rc_override_to_rc_in


def _on_mavros_rc_override(self, msg) -> None:
    channels = rc_override_channels_from_msg(msg)
    if channels is None:
        return

    fwd, sway, yaw, heave = normalized_rc_override_axes(self, channels)
    if self._sitl_transport is None or self._mavros_rc_override_local_fallback:
        self._handle_normalized_cmd(fwd, sway, yaw, heave)
    else:
        # The physics-step handoff is the sole RC sender. A callback must not
        # overtake an older cached frame being forwarded on another thread.
        store_pending_rc_override(self, channels)

    _mirror_rc_override_to_rc_in(self, channels)


__all__ = ["_on_mavros_rc_override"]
