"""Shared physics phase for one-step simulation runtime."""

from __future__ import annotations


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
        owner.update_thruster_forces(thr_dt)
    owner.update_propeller_visuals(thr_dt)
    if auto_release_initial_hold:
        owner.maybe_auto_release_initial_depth_hold()
    owner.process_pending_initial_depth_release()
    owner.apply_initial_depth_hold()
    owner.apply_underwater_wrench(owner.model.opt.timestep if not is_paused else 0.0)
    owner.emit_thruster_debug()
    owner.enforce_descent_contract()

    if not is_paused:
        owner.mujoco.mj_step(owner.model, owner.data)
        owner.apply_initial_depth_hold()


__all__ = ["apply_common_step_physics"]
