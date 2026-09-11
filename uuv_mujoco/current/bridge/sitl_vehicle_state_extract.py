"""Extract armed/mode state from ArduSub HEARTBEAT messages."""

from __future__ import annotations

from .sitl_vehicle_state_log import log_vehicle_state_change


def heartbeat_armed(owner, msg) -> bool:
    armed_flag = int(owner._sitl_mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return (int(getattr(msg, "base_mode", 0)) & armed_flag) != 0


def heartbeat_mode(owner, msg) -> str:
    try:
        return str(owner._sitl_mavutil.mode_string_v10(msg))
    except Exception:
        return ""


def record_vehicle_state_from_heartbeat(owner, msg, *, command_link: bool) -> None:
    try:
        armed = heartbeat_armed(owner, msg)
        mode = heartbeat_mode(owner, msg)
        owner._sitl_vehicle_armed = armed
        if mode:
            owner._sitl_vehicle_mode = mode
        log_vehicle_state_change(owner, armed=armed, mode=mode, command_link=command_link)
    except Exception:
        pass


__all__ = ["heartbeat_armed", "heartbeat_mode", "record_vehicle_state_from_heartbeat"]
