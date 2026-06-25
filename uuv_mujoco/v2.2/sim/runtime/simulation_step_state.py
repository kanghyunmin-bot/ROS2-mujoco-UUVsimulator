"""State container for one-step MuJoCo/SITL execution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, MutableMapping


@dataclass
class SimulationStepRuntimeState:
    """Dependencies and mutable handles required by one simulation tick."""

    mujoco: Any
    model: Any
    data: Any
    sitl_enabled: bool
    plant_replay_direct_rcout: bool
    sitl_allow_direct_cmd: bool
    command_state: Any
    sitl_servo_runtime: Any
    thr_target: MutableMapping[str, float]
    sitl_servo_timeout_s: float
    sitl_servo_pwm_values: Any
    initial_depth_hold: dict
    initial_depth_hold_auto_release: bool
    get_ros_bridge: Callable[[], Any]
    release_initial_depth_hold: Callable[[str], Any]
    process_pending_initial_depth_release: Callable[[], Any]
    thruster_update_due: Callable[[], tuple[bool, float]]
    spin_ros_once: Callable[[], None]
    apply_direct_command_targets: Callable[[], tuple[float, float, float, float]]
    update_thruster_forces: Callable[[float], None]
    update_propeller_visuals: Callable[[float], None]
    apply_initial_depth_hold: Callable[[], None]
    apply_underwater_wrench: Callable[[float], None]
    emit_thruster_debug: Callable[[], None]
    enforce_descent_contract: Callable[[], None]
    publish_ros_once: Callable[[], None]
    publish_qgc_video_once: Callable[[], None]


__all__ = ["SimulationStepRuntimeState"]
