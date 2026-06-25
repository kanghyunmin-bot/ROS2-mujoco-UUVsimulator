"""Ping360 range and sample-period effective settings."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ping360_range_math import calculate_sample_period, clipped_requested_range, reduce_samples_for_period
from .ping360_range_quality import (
    append_effective_range_flags,
    blind_bins_for_min_range,
    effective_range_m,
    range_resolution_m,
)
from .ping360_types import Ping360Config


@dataclass(frozen=True)
class Ping360RangeSettings:
    requested_range_m: float
    effective_range_m: float
    speed_of_sound_mps: float
    number_of_samples: int
    sample_period_ticks: int
    range_resolution_m: float
    blind_bins: int
    quality_flags: list[str]


def build_range_settings(config: Ping360Config) -> Ping360RangeSettings:
    quality_flags: list[str] = []
    requested_range = float(config.requested_range_m)
    range_m = clipped_requested_range(config, quality_flags)
    speed_of_sound = float(np.clip(config.speed_of_sound_mps, 1300.0, 1700.0))
    num_samples, sample_period = reduce_samples_for_period(config, range_m, speed_of_sound)
    effective_range = effective_range_m(sample_period, num_samples, speed_of_sound)
    range_resolution = range_resolution_m(effective_range, num_samples)
    append_effective_range_flags(
        config=config,
        num_samples=num_samples,
        effective_range=effective_range,
        range_resolution=range_resolution,
        quality_flags=quality_flags,
    )
    blind_bins = blind_bins_for_min_range(config, range_resolution)
    return Ping360RangeSettings(
        requested_range_m=requested_range,
        effective_range_m=effective_range,
        speed_of_sound_mps=speed_of_sound,
        number_of_samples=num_samples,
        sample_period_ticks=sample_period,
        range_resolution_m=range_resolution,
        blind_bins=blind_bins,
        quality_flags=quality_flags,
    )
__all__ = ["Ping360RangeSettings", "build_range_settings", "calculate_sample_period"]
