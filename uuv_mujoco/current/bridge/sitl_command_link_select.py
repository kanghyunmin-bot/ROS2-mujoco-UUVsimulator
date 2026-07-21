"""SITL MAVLink command-link object selection helpers."""

from __future__ import annotations

from sim.transport import MavlinkCommandLink


def _mav_for_commands(self):
    return self._sitl_cmd_mav if self._sitl_cmd_mav is not None else self._sitl_mav


def _mav_for_external_nav(self):
    return self._sitl_cmd_mav if self._sitl_cmd_mav is not None else self._sitl_mav


def _mavs_for_arm_mode_commands(self) -> list:
    mavs = []
    for mav in (self._sitl_cmd_mav, self._sitl_mav):
        if mav is not None and all(mav is not existing for existing in mavs):
            mavs.append(mav)
    return mavs


def _arm_mode_command_pending(self) -> bool:
    return self._sitl_pending_arm_target is not None or bool(str(self._sitl_pending_mode or "").strip())


def _command_link_for_mav(self, mav) -> MavlinkCommandLink | None:
    if mav is None:
        return None
    if mav is self._sitl_cmd_mav:
        return self._sitl_command_link
    if mav is self._sitl_mav:
        return self._sitl_servo_command_link
    return None


__all__ = [
    "_arm_mode_command_pending",
    "_command_link_for_mav",
    "_mav_for_commands",
    "_mav_for_external_nav",
    "_mavs_for_arm_mode_commands",
]
