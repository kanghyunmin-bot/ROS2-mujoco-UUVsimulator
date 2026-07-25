"""Noise and blind-zone helpers for Ping360 profiles."""

from __future__ import annotations

import math

import numpy as np

from .ping360_types import Ping360Config, Ping360EffectiveSettings


def apply_blind_zone(profile: np.ndarray, blind_bins: int) -> tuple[int, int]:
    n = int(profile.size)
    clipped_blind_bins = min(blind_bins, n)
    if clipped_blind_bins > 0:
        profile[:clipped_blind_bins] = 0.0
    return clipped_blind_bins, max(0, n - clipped_blind_bins)


def build_ping360_noise(
    *,
    visible_bins: int,
    settings: Ping360EffectiveSettings,
    config: Ping360Config,
    rng: np.random.Generator,
) -> np.ndarray:
    noise = np.zeros(visible_bins, dtype=np.float64)
    if config.noise_floor > 0.0:
        noise += float(config.noise_floor)
    if config.speckle_std > 0.0:
        noise += rng.normal(0.0, float(config.speckle_std), visible_bins)
    apply_nearfield_fade(noise, settings=settings)
    return noise


def apply_nearfield_fade(noise: np.ndarray, *, settings: Ping360EffectiveSettings) -> None:
    fade_bins = min(
        int(noise.size),
        max(1, int(math.ceil(0.20 / max(settings.range_resolution_m, 1.0e-9)))),
    )
    if fade_bins > 1:
        noise[:fade_bins] *= np.linspace(0.0, 1.0, fade_bins, dtype=np.float64)


__all__ = ["apply_blind_zone", "apply_nearfield_fade", "build_ping360_noise"]
