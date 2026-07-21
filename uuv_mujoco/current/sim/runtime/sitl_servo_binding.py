"""Bridge binding helpers for SITL/plant-replay PWM servo runtime."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

from .sitl_servo_state import SitlServoRuntime


def create_and_bind_sitl_servo_runtime(
    *,
    all_thruster_names: Sequence[str],
    raw_map: Sequence[str],
    servo_signs: Sequence[float],
    sitl_servo_scale: float,
    sitl_enabled: bool,
    plant_replay_direct_rcout: bool,
    ros_bridge_runtime: Any,
    log: Callable[[str], None],
) -> SitlServoRuntime:
    """Create the SITL PWM runtime and attach it to active bridge handlers."""

    runtime = SitlServoRuntime.create(
        all_thruster_names=list(all_thruster_names),
        raw_map=list(raw_map),
        servo_signs=[float(value) for value in servo_signs],
        sitl_servo_scale=float(sitl_servo_scale),
    )
    if sitl_enabled or plant_replay_direct_rcout:
        active_ros_bridge = ros_bridge_runtime.get()
        if active_ros_bridge is not None and sitl_enabled:
            active_ros_bridge.set_sitl_servo_handler(runtime.on_packet)
        if active_ros_bridge is not None and plant_replay_direct_rcout:
            set_replay_handler = getattr(active_ros_bridge, "set_replay_rcout_handler", None)
            if callable(set_replay_handler):
                set_replay_handler(runtime.on_packet)
        mode_label = "sitl" if sitl_enabled else "plant_replay"
        log(
            f"[{mode_label}] direct PWM thruster mode enabled: "
            + runtime.mapping_label()
            + f", servo-scale={runtime.scale:.2f}"
        )
    return runtime


__all__ = ["create_and_bind_sitl_servo_runtime"]
