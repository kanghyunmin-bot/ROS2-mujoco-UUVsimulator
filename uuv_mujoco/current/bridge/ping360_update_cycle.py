"""One-profile Ping360 scan/update helper."""

from __future__ import annotations

import mujoco

from .ping360_profile import scan_profile
from .ping360_runtime_state import Ping360RuntimeState
from .ping360_samples import updated_sample
from .ping360_types import Ping360Config, Ping360EffectiveSettings, Ping360Sample


def scan_and_record_ping360_profile(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    base_body_id: int,
    runtime: Ping360RuntimeState,
    config: Ping360Config,
    settings: Ping360EffectiveSettings,
    sim_t: float,
    angle_grad: int,
    ping_number: int,
) -> Ping360Sample:
    profile, nearest_range, peak_intensity = scan_profile(
        model=model,
        data=data,
        site_id=site_id,
        base_body_id=base_body_id,
        geomgroup=runtime.geomgroup,
        config=config,
        settings=settings,
        rng=runtime.rng,
        angle_grad=angle_grad,
    )
    runtime.history.record(
        angle_grad=angle_grad,
        profile=profile,
        nearest_range=nearest_range,
        peak_intensity=peak_intensity,
    )
    return updated_sample(
        sim_t=sim_t,
        angle_grad=angle_grad,
        profile=profile,
        history=runtime.history,
        settings=settings,
        ping_number=ping_number,
    )


__all__ = ["scan_and_record_ping360_profile"]
