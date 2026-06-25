"""Vehicle heartbeat state-change logging."""

from __future__ import annotations


def log_vehicle_state_change(owner, *, armed: bool, mode: str, command_link: bool) -> None:
    mode_changed = mode and owner._sitl_last_vehicle_mode != mode
    armed_changed = owner._sitl_last_vehicle_armed is None or owner._sitl_last_vehicle_armed != armed
    if not (armed_changed or mode_changed):
        return
    src = "command" if command_link else "servo"
    print(
        f"[sitl_transport] vehicle heartbeat state: armed={armed} mode={owner._sitl_vehicle_mode or 'UNKNOWN'} via {src} link",
        flush=True,
    )
    owner._sitl_last_vehicle_armed = armed
    if mode:
        owner._sitl_last_vehicle_mode = mode


__all__ = ["log_vehicle_state_change"]
