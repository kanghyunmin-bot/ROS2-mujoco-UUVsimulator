"""Ping360 profile accumulation from MuJoCo raycast returns."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .ping360_beam_model import geom_reflectivity
from .ping360_profile_hits import iter_beam_returns
from .ping360_profile_signal import add_noise_and_blind_zone, accumulate_return, return_strength
from .ping360_types import Ping360Config, Ping360EffectiveSettings


def scan_profile(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    base_body_id: int,
    geomgroup: np.ndarray,
    config: Ping360Config,
    settings: Ping360EffectiveSettings,
    rng: np.random.Generator,
    angle_grad: int,
) -> tuple[np.ndarray, float | None, float]:
    n = settings.number_of_samples
    profile = np.zeros(n, dtype=np.float64)
    if site_id < 0:
        return profile.astype(np.uint8), None, 0.0

    nearest = math.inf
    peak = 0.0
    for dist, geom_id, weight in iter_beam_returns(
        model=model,
        data=data,
        site_id=site_id,
        base_body_id=base_body_id,
        geomgroup=geomgroup,
        config=config,
        settings=settings,
        angle_grad=angle_grad,
    ):
        nearest = min(nearest, dist)
        amp = return_strength(geom_reflectivity(model, geom_id), dist, settings, config) * weight
        peak = max(peak, amp)
        accumulate_return(profile, dist, amp, settings)

    add_noise_and_blind_zone(profile, settings, config, rng)
    profile = np.clip(profile, 0.0, 255.0)
    return profile.astype(np.uint8), (nearest if math.isfinite(nearest) else None), float(min(255.0, peak))


__all__ = [
    "accumulate_return",
    "add_noise_and_blind_zone",
    "return_strength",
    "scan_profile",
]
