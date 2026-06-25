"""Internal SITL command override callback for Ros2Bridge."""

from __future__ import annotations

from .ros2_sitl_command_override_arm_mode import _handle_arm_override, _handle_mode_override
from .ros2_sitl_command_override_replay import _handle_replay_rcout
from .ros2_sitl_command_override_sensor import _mark_sensor_replay_input
from .ros2_sitl_command_override_topic import COMMAND_OVERRIDE_TOPIC


def _on_sitl_command_override(self, msg) -> None:
    payload = self._parse_command_override_payload(getattr(msg, "data", ""))
    if not payload:
        return

    _mark_sensor_replay_input(self, payload)
    _handle_replay_rcout(self, payload)
    _handle_arm_override(self, payload)
    _handle_mode_override(self, payload)


__all__ = ["COMMAND_OVERRIDE_TOPIC", "_on_sitl_command_override"]
