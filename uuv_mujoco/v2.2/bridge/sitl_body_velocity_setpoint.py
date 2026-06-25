"""GUIDED body-frame velocity setpoint helper for SitlTransport."""

from __future__ import annotations


def send_body_velocity_setpoint(
    self,
    *,
    forward_mps: float,
    left_mps: float,
    up_mps: float,
    yaw_rate_rad_s: float,
) -> bool:
    """Forward ROS body FLU velocity command to ArduSub GUIDED velocity control."""
    mav = self._mav_for_commands()
    if mav is None or self._sitl_mavutil is None:
        return False
    target = self._resolve_mav_target(mav)
    if target is None:
        return False
    target_sys, target_comp = target
    mavlink_defs = self._sitl_mavutil.mavlink
    type_mask = int(
        mavlink_defs.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavlink_defs.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavlink_defs.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavlink_defs.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavlink_defs.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavlink_defs.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavlink_defs.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )
    try:
        mav.mav.set_position_target_local_ned_send(
            0,
            int(target_sys),
            int(target_comp),
            int(mavlink_defs.MAV_FRAME_BODY_NED),
            type_mask,
            0.0,
            0.0,
            0.0,
            float(forward_mps),
            float(-left_mps),
            float(-up_mps),
            0.0,
            0.0,
            0.0,
            0.0,
            float(-yaw_rate_rad_s),
        )
        return True
    except Exception as exc:
        print(f"[sitl_transport] body velocity setpoint send failed: {exc}", flush=True)
        return False


__all__ = ["send_body_velocity_setpoint"]
