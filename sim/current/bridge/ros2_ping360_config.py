"""Ping360 runtime config callback for Ros2Bridge."""

from __future__ import annotations

import json

from .ping360_sim import Ping360Config, Ping360Simulator
from .ros2_mujoco_model import site_id


def on_ping360_config(self, msg) -> None:
    try:
        payload = json.loads(str(getattr(msg, "data", "") or "{}"))
    except json.JSONDecodeError as exc:
        if self.node is not None:
            self.node.get_logger().warn(f"invalid /ping360/config JSON: {exc}")
        return
    if not isinstance(payload, dict):
        if self.node is not None:
            self.node.get_logger().warn("/ping360/config must be a JSON object")
        return

    known = set(Ping360Config.__dataclass_fields__)
    current = {
        field_name: getattr(self._ping360_config, field_name)
        for field_name in Ping360Config.__dataclass_fields__
    }
    current.update({key: value for key, value in payload.items() if key in known})
    next_config = Ping360Config(**current)
    self._ping360_config = next_config
    self._ping360_site_id = site_id(self.model, next_config.site_name)
    self._ping360 = Ping360Simulator(self.model, next_config) if next_config.enabled else None
    self._ping360_image_renderer.reset()
    if self.node is not None:
        settings = self._ping360.settings.as_dict() if self._ping360 is not None else {}
        state = "on" if next_config.enabled else "off"
        self.node.get_logger().info(
            "updated /ping360/config "
            f"enabled={state} "
            f"range={settings.get('effective_range_m', next_config.requested_range_m):.3f}m "
            f"steps={settings.get('num_steps', next_config.num_steps)} "
            f"sector={settings.get('start_angle_grad', next_config.start_angle_grad)}.."
            f"{settings.get('stop_angle_grad', next_config.stop_angle_grad)} grad"
        )
