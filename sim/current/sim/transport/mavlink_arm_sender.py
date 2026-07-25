"""Low-level MAVLink arm/disarm command sender."""

from __future__ import annotations


def send_arm_disarm_on_link(
    mav: object | None,
    mavutil: object,
    target_sys: int,
    target_comp: int,
    arm_value: bool,
    *,
    force: bool = False,
) -> bool:
    if mav is None or mavutil is None:
        return False
    force_magic = 2989.0 if arm_value else 21196.0
    try:
        mav.mav.command_long_send(
            int(target_sys),
            int(target_comp),
            int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM),
            0,
            1.0 if arm_value else 0.0,
            force_magic if force else 0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        return True
    except Exception:
        return False


__all__ = ["send_arm_disarm_on_link"]
