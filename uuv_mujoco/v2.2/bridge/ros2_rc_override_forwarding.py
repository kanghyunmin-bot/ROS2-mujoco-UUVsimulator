"""SITL forwarding policy for MAVROS RC override input."""

from __future__ import annotations

from collections.abc import Sequence

from .ros2_rc_override_frame import normalized_rc_override_axes, rc_override_forward_frame


def forward_rc_override_to_sitl(
    self,
    channels: Sequence[int],
    *,
    axes: tuple[float, float, float, float] | None = None,
) -> bool:
    if self._sitl_transport is None:
        return False
    try:
        with self._sitl_transport_lock:
            if getattr(self, "_mavros_rc_override_backend", "manual_control") == "manual_control":
                fwd, sway, yaw, heave = axes or normalized_rc_override_axes(self, channels)
                return bool(
                    self._sitl_transport.send_manual_control(
                        x=fwd,
                        y=sway,
                        z=heave,
                        r=yaw,
                        buttons=0,
                    )
                )
            return bool(self._sitl_transport.send_rc_override(rc_override_forward_frame(channels)))
    except Exception:
        return False


__all__ = ["forward_rc_override_to_sitl"]
