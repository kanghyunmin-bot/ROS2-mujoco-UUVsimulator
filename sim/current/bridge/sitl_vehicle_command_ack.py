"""COMMAND_ACK handling for SITL vehicle command state."""

from __future__ import annotations


def _handle_command_ack(self, msg, *, link_name: str) -> None:
    command = int(getattr(msg, "command", -1))
    result = int(getattr(msg, "result", -1))
    if self._sitl_cmd_debug:
        print(
            f"[sitl_transport] COMMAND_ACK link={link_name} command={command} result={result}",
            flush=True,
        )
    if self._sitl_mavutil is None:
        return
    accepted, command_ids = _command_ack_constants(self)
    if result != accepted:
        return
    # COMMAND_ACK means ArduPilot accepted the command message.  It is not
    # the authoritative vehicle state.  Keep pending arm/mode commands alive
    # until a vehicle HEARTBEAT reports the target armed flag and mode.
    if command in command_ids and self._sitl_cmd_debug:
        print(
            "[sitl_transport] COMMAND_ACK accepted; waiting for HEARTBEAT "
            "state confirmation",
            flush=True,
        )


def _command_ack_constants(self) -> tuple[int, set[int]]:
    try:
        mavlink_defs = self._sitl_mavutil.mavlink
        accepted = int(mavlink_defs.MAV_RESULT_ACCEPTED)
        command_ids = {
            int(mavlink_defs.MAV_CMD_COMPONENT_ARM_DISARM),
            int(mavlink_defs.MAV_CMD_DO_SET_MODE),
            int(mavlink_defs.MAVLINK_MSG_ID_SET_MODE),
        }
    except Exception:
        accepted = 0
        command_ids = {400, 176, 11}
    return accepted, command_ids


__all__ = ["_handle_command_ack"]
