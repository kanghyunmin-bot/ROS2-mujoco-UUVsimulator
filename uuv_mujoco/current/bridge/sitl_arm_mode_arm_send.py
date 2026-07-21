"""Arm/disarm MAVLink forwarding for SitlTransport."""

from __future__ import annotations


def _send_arm_disarm_mavlink(
    self,
    mav,
    target_sys: int,
    target_comp: int,
    arm_value: bool,
    *,
    force: bool = False,
) -> bool:
    if mav is None or self._sitl_mavutil is None:
        return False
    link = self._command_link_for_mav(mav)
    if link is not None:
        sent = link.send_arm_disarm(self._sitl_mavutil, target_sys, target_comp, arm_value, force=force)
        if self._sitl_cmd_debug:
            print(
                f"[sitl_transport] arm/disarm send link={link.endpoint or 'same'} "
                f"target={target_sys}:{target_comp} armed={bool(arm_value)} sent={int(bool(sent))}",
                flush=True,
            )
        return bool(sent)
    return False


__all__ = ["_send_arm_disarm_mavlink"]
