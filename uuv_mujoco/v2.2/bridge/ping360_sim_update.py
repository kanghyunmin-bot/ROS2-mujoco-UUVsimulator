"""Update-cycle helpers for the public Ping360Simulator facade."""

from __future__ import annotations

import mujoco

from .ping360_samples import held_sample
from .ping360_sim_lifecycle import ping360_simulator_active, refresh_ping360_simulator_runtime
from .ping360_sim_status import current_ping360_angle_grad
from .ping360_types import Ping360Sample
from .ping360_update_cycle import scan_and_record_ping360_profile


def update_ping360_simulator(owner: object, data: mujoco.MjData, sim_t: float) -> Ping360Sample | None:
    if not ping360_simulator_active(owner):
        return None
    sim_t = float(sim_t)
    refresh_ping360_simulator_runtime(owner)
    if owner._next_profile_t < 0.0:
        owner._next_profile_t = sim_t
    if owner._latest is not None and sim_t + 1.0e-9 < owner._next_profile_t:
        return held_sample(latest=owner._latest, sim_t=sim_t, settings=owner.settings)

    owner._latest = scan_and_record_ping360_profile(
        model=owner.model,
        data=data,
        site_id=owner.site_id,
        base_body_id=owner.base_body_id,
        runtime=owner._runtime,
        config=owner.config,
        settings=owner.settings,
        sim_t=sim_t,
        angle_grad=current_ping360_angle_grad(owner),
        ping_number=owner._ping_number + 1,
    )
    owner._ping_number += 1
    owner._runtime.sweep.advance(settings=owner.settings, config=owner.config)
    owner._next_profile_t = sim_t + owner.settings.profile_period_s
    return owner._latest


__all__ = ["update_ping360_simulator"]
