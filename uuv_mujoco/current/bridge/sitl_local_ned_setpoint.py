"""Raw local-NED setpoint helper for SitlTransport."""

from __future__ import annotations


def send_position_target_local_ned(
    self,
    *,
    coordinate_frame: int,
    type_mask: int,
    x: float,
    y: float,
    z: float,
    vx: float,
    vy: float,
    vz: float,
    afx: float,
    afy: float,
    afz: float,
    yaw: float,
    yaw_rate: float,
) -> bool:
    """Forward MAVROS raw local setpoint to ArduSub SITL unchanged."""
    mav = self._mav_for_commands()
    if mav is None:
        return False
    target = self._resolve_mav_target(mav)
    if target is None:
        return False
    target_sys, target_comp = target
    try:
        mav.mav.set_position_target_local_ned_send(
            0,
            int(target_sys),
            int(target_comp),
            int(coordinate_frame),
            int(type_mask),
            float(x),
            float(y),
            float(z),
            float(vx),
            float(vy),
            float(vz),
            float(afx),
            float(afy),
            float(afz),
            float(yaw),
            float(yaw_rate),
        )
        return True
    except Exception as exc:
        print(f"[sitl_transport] raw local setpoint send failed: {exc}", flush=True)
        return False


__all__ = ["send_position_target_local_ned"]
