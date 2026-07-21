"""Raw-PWM SITL/plant-replay step path for the MuJoCo runtime."""

from __future__ import annotations

import time
from typing import Any

from .simulation_step_physics import apply_common_step_physics
from .simulation_step_timing import record_step_phase


def run_raw_pwm_runtime_step(
    runtime: Any,
    *,
    is_paused: bool,
    publish_ros: bool,
    thruster_due: bool,
    thruster_dt: float,
) -> tuple[float, float, float, float]:
    forward, sway, yaw, heave = _apply_raw_pwm_control_inputs(runtime)
    apply_common_step_physics(
        runtime,
        is_paused=is_paused,
        thruster_due=thruster_due,
        thruster_dt=thruster_dt,
        auto_release_initial_hold=True,
    )
    if publish_ros:
        started = time.perf_counter()
        runtime.publish_ros_once()
        record_step_phase(runtime, "ros_publish", time.perf_counter() - started)
    started = time.perf_counter()
    runtime.publish_qgc_video_once()
    record_step_phase(runtime, "qgc_video", time.perf_counter() - started)
    return forward, sway, yaw, heave


def _apply_raw_pwm_control_inputs(runtime: Any) -> tuple[float, float, float, float]:
    now = time.monotonic()
    if _direct_command_active(runtime, now):
        return runtime.apply_direct_command_targets()
    runtime.sitl_servo_runtime.apply_to_targets(
        runtime.thr_target,
        now_wall=now,
        timeout_s=runtime.sitl_servo_timeout_s,
    )
    return 0.0, 0.0, 0.0, 0.0


def _direct_command_active(runtime: Any, now: float) -> bool:
    return bool(runtime.sitl_allow_direct_cmd and runtime.command_state.recently_active(now))


__all__ = ["run_raw_pwm_runtime_step"]
