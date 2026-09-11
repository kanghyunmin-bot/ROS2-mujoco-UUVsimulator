"""Shared physics phase for one-step simulation runtime."""

from __future__ import annotations

import time

from .simulation_step_timing import record_step_phase
from .physics_step_guard import step_with_reset_guard


def apply_common_step_physics(
    owner,
    *,
    is_paused: bool,
    thruster_due: bool,
    thruster_dt: float,
    auto_release_initial_hold: bool,
) -> None:
    thr_dt = thruster_dt if not is_paused else 0.0
    if thruster_due:
        started = time.perf_counter()
        owner.update_thruster_forces(thr_dt)
        record_step_phase(owner, "thrusters", time.perf_counter() - started)
    started = time.perf_counter()
    owner.update_propeller_visuals(thr_dt)
    record_step_phase(owner, "prop_visuals", time.perf_counter() - started)
    if auto_release_initial_hold:
        owner.maybe_auto_release_initial_depth_hold()
    owner.process_pending_initial_depth_release()
    owner.apply_initial_depth_hold()
    started = time.perf_counter()
    owner.apply_underwater_wrench(owner.model.opt.timestep if not is_paused else 0.0)
    record_step_phase(owner, "wrenches", time.perf_counter() - started)
    owner.emit_thruster_debug()
    owner.enforce_descent_contract()

    if not is_paused:
        started = time.perf_counter()
        step_with_reset_guard(owner.mujoco, owner.model, owner.data)
        record_step_phase(owner, "mj_step", time.perf_counter() - started)
        owner.apply_initial_depth_hold()


__all__ = ["apply_common_step_physics"]
