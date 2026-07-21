"""Send-method wrappers for the low-level MAVLink command link."""

from __future__ import annotations

from .mavlink_command_senders import (
    send_arm_disarm_on_link,
    send_gcs_heartbeat_on_link,
    send_rc_channels_override_on_link,
)


class MavlinkCommandLinkSendMixin:
    """Public send methods backed by primitive MAVLink packet helpers."""

    def send_gcs_heartbeat(self, mavutil: object, *, force: bool = False, now_wall: float | None = None) -> bool:
        sent, last_send_wall = send_gcs_heartbeat_on_link(
            self.mav,
            mavutil,
            last_send_wall=self.last_heartbeat_send_wall,
            force=force,
            now_wall=now_wall,
        )
        self.last_heartbeat_send_wall = last_send_wall
        return sent

    def send_rc_channels_override(self, target_sys: int, target_comp: int, values: list[int]) -> int:
        """Send RC_CHANNELS_OVERRIDE, preferring MAVLink2 channels 1..18."""
        return send_rc_channels_override_on_link(self.mav, target_sys, target_comp, values)

    def send_arm_disarm(
        self,
        mavutil: object,
        target_sys: int,
        target_comp: int,
        arm_value: bool,
        *,
        force: bool = False,
    ) -> bool:
        return send_arm_disarm_on_link(
            self.mav,
            mavutil,
            target_sys,
            target_comp,
            arm_value,
            force=force,
        )


__all__ = ["MavlinkCommandLinkSendMixin"]
