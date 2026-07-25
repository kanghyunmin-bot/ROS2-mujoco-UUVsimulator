"""Initial-depth hold release policy for one-step simulation runtime."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any


def maybe_release_initial_depth_hold(
    *,
    initial_depth_hold: dict,
    auto_release: bool,
    ros_bridge: Any,
    sitl_servo_pwm_values: Sequence[int],
    release_initial_depth_hold: Callable[[str], Any],
) -> None:
    if not initial_depth_hold["active"] or not auto_release:
        return
    if ros_bridge is None:
        return
    armed_fn = getattr(ros_bridge, "sitl_vehicle_armed", None)
    mode_fn = getattr(ros_bridge, "sitl_vehicle_mode", None)
    armed = bool(armed_fn()) if callable(armed_fn) else False
    mode = str(mode_fn()).upper() if callable(mode_fn) else ""
    if not armed:
        return
    valid_pwm = [int(v) for v in sitl_servo_pwm_values[:8] if int(v) not in (0, 65535)]
    max_delta = max((abs(v - 1500) for v in valid_pwm), default=0)
    if mode != "ALT_HOLD" and max_delta > 12:
        release_initial_depth_hold("auto:armed_nonneutral_servo")


__all__ = ["maybe_release_initial_depth_hold"]
