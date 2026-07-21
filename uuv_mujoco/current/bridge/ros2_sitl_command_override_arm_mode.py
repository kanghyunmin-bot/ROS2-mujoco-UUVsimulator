"""Arm and mode command override handling."""

from __future__ import annotations

from .ros2_sitl_command_override_topic import COMMAND_OVERRIDE_TOPIC


def _handle_arm_override(self, payload: dict[str, object]) -> None:
    if "arm" not in payload and "armed" not in payload:
        return
    arm_value = self._parse_command_bool(payload.get("arm", payload.get("armed")), default=False)
    ok = self._forward_arm_request(arm_value, COMMAND_OVERRIDE_TOPIC)
    print(
        f"[bridge] {COMMAND_OVERRIDE_TOPIC} arm={arm_value} forwarded={ok}",
        flush=True,
    )


def _handle_mode_override(self, payload: dict[str, object]) -> None:
    if "mode" not in payload:
        return
    mode = str(payload.get("mode", "")).strip()
    if not mode:
        return
    ok = self._forward_mode_request(mode, COMMAND_OVERRIDE_TOPIC)
    print(
        f"[bridge] {COMMAND_OVERRIDE_TOPIC} mode={mode!r} forwarded={ok}",
        flush=True,
    )


__all__ = ["_handle_arm_override", "_handle_mode_override"]
