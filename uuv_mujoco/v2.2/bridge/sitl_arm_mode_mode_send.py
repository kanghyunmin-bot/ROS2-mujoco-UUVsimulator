"""SET_MODE MAVLink forwarding for SitlTransport."""

from __future__ import annotations


def _send_set_mode_mavlink(self, mav, target_sys: int, target_comp: int, mode: str) -> bool:
    if mav is None or self._sitl_mavutil is None:
        return False
    try:
        parsed = self._mode_id_for_text(mode)
        if parsed is None:
            return False
        mode_text, mode_id = parsed
        _send_set_mode_command_long(self, mav, target_sys, target_comp, mode_id)
        if self._sitl_cmd_debug:
            print(
                f"[sitl_transport] set_mode forwarded to ArduSub target={target_sys}:{target_comp} "
                f"mode={mode_text!r} custom_mode={mode_id}",
                flush=True,
            )
        return True
    except Exception as exc:
        print(f"[sitl_transport] set_mode send failed for {mode!r}: {exc}", flush=True)
        return False


def _send_set_mode_command_long(self, mav, target_sys: int, target_comp: int, mode_id: int) -> None:
    mavlink_defs = self._sitl_mavutil.mavlink
    mav.mav.command_long_send(
        int(target_sys),
        int(target_comp),
        int(mavlink_defs.MAV_CMD_DO_SET_MODE),
        0,
        int(mavlink_defs.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(mode_id),
        0,
        0,
        0,
        0,
        0,
    )


__all__ = ["_send_set_mode_mavlink"]
