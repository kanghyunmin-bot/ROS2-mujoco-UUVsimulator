"""Update-cycle helpers for the public Ping360Simulator facade."""

from __future__ import annotations

import mujoco

from .ping360_samples import held_sample
from .ping360_sim_lifecycle import ping360_simulator_active, refresh_ping360_simulator_runtime
from .ping360_sim_status import current_ping360_angle_grad
from .ping360_types import Ping360Sample
from .ping360_update_cycle import scan_and_record_ping360_profile


# The firmware timing model clamps one profile to at least 1 ms. The normal
# 10 Hz ROS image cadence can therefore owe at most 101 profiles (including an
# exact boundary); 128 covers that case while bounding debugger/time-jump work.
MAX_PROFILE_CATCH_UP = 128
_TIME_EPSILON_S = 1.0e-9


def update_ping360_simulator(owner: object, data: mujoco.MjData, sim_t: float) -> Ping360Sample | None:
    if not ping360_simulator_active(owner):
        return None
    sim_t = float(sim_t)
    refresh_ping360_simulator_runtime(owner)
    if owner._next_profile_t < 0.0 or owner._latest is None:
        owner._next_profile_t = sim_t
    if owner._latest is not None and sim_t + _TIME_EPSILON_S < owner._next_profile_t:
        return held_sample(latest=owner._latest, sim_t=sim_t, settings=owner.settings)

    profile_period_s = max(float(owner.settings.profile_period_s), _TIME_EPSILON_S)
    profiles_generated = 0
    while (
        owner._next_profile_t <= sim_t + _TIME_EPSILON_S
        and profiles_generated < MAX_PROFILE_CATCH_UP
    ):
        profile_time_s = float(owner._next_profile_t)
        owner._latest = scan_and_record_ping360_profile(
            model=owner.model,
            data=data,
            site_id=owner.site_id,
            base_body_id=owner.base_body_id,
            runtime=owner._runtime,
            config=owner.config,
            settings=owner.settings,
            sim_t=profile_time_s,
            angle_grad=current_ping360_angle_grad(owner),
            ping_number=owner._ping_number + 1,
        )
        owner._ping_number += 1
        owner._runtime.sweep.advance(settings=owner.settings, config=owner.config)
        owner._next_profile_t = profile_time_s + profile_period_s
        profiles_generated += 1

    if owner._next_profile_t <= sim_t + _TIME_EPSILON_S:
        # A debugger pause or large time jump must not turn one ROS publish into
        # an unbounded ray-cast burst. Drop the remaining backlog and resume the
        # physical profile cadence from the current simulation time.
        owner._next_profile_t = sim_t + profile_period_s
    return owner._latest


__all__ = ["MAX_PROFILE_CATCH_UP", "update_ping360_simulator"]
