"""Servo-link HEARTBEAT target filtering."""

from __future__ import annotations


def _ardupilotmega_value(self) -> int:
    if self._sitl_mavutil is None:
        return -1
    try:
        return int(self._sitl_mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
    except Exception:
        return -1


def _servo_heartbeat_target_ok(
    self,
    src_sys: int,
    src_comp: int,
    target_sys: int,
    target_comp: int,
    now_wall: float,
) -> bool:
    if target_sys > 0 and src_sys != target_sys:
        _warn_target_mismatch(self, src_sys, src_comp, target_sys, target_comp, now_wall)
        return False
    if target_comp > 0 and src_comp != target_comp:
        _warn_target_mismatch(self, src_sys, src_comp, target_sys, target_comp, now_wall)
        return False
    return True


def _warn_target_mismatch(
    self,
    src_sys: int,
    src_comp: int,
    target_sys: int,
    target_comp: int,
    now_wall: float,
) -> None:
    if now_wall - self._sitl_mav_target_mismatch_warn_wall < 2.0:
        return
    print(
        f"[sitl_transport] Ignoring HEARTBEAT from src-system={src_sys}, src-comp={src_comp}; "
        f"expecting sys={target_sys}, comp={target_comp}.",
        flush=True,
    )
    self._sitl_mav_target_mismatch_warn_wall = now_wall


__all__ = [
    "_ardupilotmega_value",
    "_servo_heartbeat_target_ok",
]
