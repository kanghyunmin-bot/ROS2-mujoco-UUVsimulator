"""MAVROS state values derived from Ros2Bridge and SitlTransport."""

from __future__ import annotations


def mavros_state_values(bridge) -> tuple[bool, str, bool, bool]:
    armed = bool(bridge._mavros_armed)
    mode = str(bridge._mavros_mode)
    connected = True
    if bridge._sitl_transport is None:
        return armed, mode, connected, True
    connected = bool(getattr(bridge._sitl_transport, "mavlink_connected", False))
    armed = bool(getattr(bridge._sitl_transport, "vehicle_armed", armed))
    actual_mode = str(getattr(bridge._sitl_transport, "vehicle_mode", "") or "")
    if actual_mode:
        mode = actual_mode
    manual_input = bool(getattr(bridge._sitl_transport, "rc_override_ready", False))
    return armed, mode, connected, manual_input


__all__ = ["mavros_state_values"]
