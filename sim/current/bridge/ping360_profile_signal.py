"""Ping360 return-strength, pulse, and noise shaping."""

from __future__ import annotations

import math

import numpy as np

from .ping360_profile_noise import apply_blind_zone, build_ping360_noise
from .ping360_types import Ping360Config, Ping360EffectiveSettings


def add_noise_and_blind_zone(
    profile: np.ndarray,
    settings: Ping360EffectiveSettings,
    config: Ping360Config,
    rng: np.random.Generator,
) -> None:
    blind_bins, visible_bins = apply_blind_zone(profile, settings.blind_bins)
    if visible_bins <= 0:
        return
    noise = build_ping360_noise(
        visible_bins=visible_bins,
        settings=settings,
        config=config,
        rng=rng,
    )
    profile[blind_bins:] += noise


def return_strength(
    reflectivity: float,
    distance_m: float,
    settings: Ping360EffectiveSettings,
    config: Ping360Config,
) -> float:
    gain = (0.9, 1.45, 2.2)[settings.gain_setting]
    frequency_penalty = 1.0 - min(abs(settings.transmit_frequency_khz - 750.0) / 500.0, 0.5)
    absorption = 10.0 ** (-(float(config.absorption_db_per_m) * distance_m) / 20.0)
    spreading = 1.0 / max(distance_m, 1.0) ** float(config.range_power_loss)
    pulse_gain = math.sqrt(max(settings.transmit_duration_us, 1.0) / 11.0)
    return reflectivity * gain * frequency_penalty * absorption * spreading * pulse_gain


def accumulate_return(
    profile: np.ndarray,
    distance_m: float,
    amplitude: float,
    settings: Ping360EffectiveSettings,
) -> None:
    bin_width = max(settings.range_resolution_m, 1.0e-9)
    center = int(round(distance_m / bin_width))
    if center < 0 or center >= profile.size:
        return
    pulse_length_m = settings.speed_of_sound_mps * settings.transmit_duration_us * 1.0e-6 / 2.0
    sigma_bins = max(0.75, pulse_length_m / bin_width)
    radius = int(max(1, min(12, math.ceil(3.0 * sigma_bins))))
    lo = max(0, center - radius)
    hi = min(profile.size, center + radius + 1)
    idx = np.arange(lo, hi, dtype=np.float64)
    weights = np.exp(-0.5 * ((idx - center) / sigma_bins) ** 2)
    if weights.size > 0 and float(weights.max()) > 0.0:
        weights /= float(weights.max())
    profile[lo:hi] += amplitude * weights


__all__ = ["accumulate_return", "add_noise_and_blind_zone", "return_strength"]
