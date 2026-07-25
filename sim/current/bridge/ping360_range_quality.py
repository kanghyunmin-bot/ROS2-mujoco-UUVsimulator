"""Quality flag helpers for Ping360 effective range settings."""

from __future__ import annotations

import numpy as np

from .ping360_types import SAMPLE_PERIOD_TICK_S, Ping360Config


def effective_range_m(sample_period: int, num_samples: int, speed_of_sound: float) -> float:
    return float(sample_period * SAMPLE_PERIOD_TICK_S * num_samples * speed_of_sound / 2.0)


def range_resolution_m(effective_range: float, num_samples: int) -> float:
    return float(effective_range / max(num_samples, 1))


def append_effective_range_flags(
    *,
    config: Ping360Config,
    num_samples: int,
    effective_range: float,
    range_resolution: float,
    quality_flags: list[str],
) -> None:
    if num_samples < config.max_number_of_samples:
        quality_flags.append("short_range_reduced_sample_count")
    if effective_range + 1.0e-6 < config.min_range_m:
        quality_flags.append("effective_range_below_blind_zone")
    if range_resolution > 0.025:
        quality_flags.append("range_resolution_coarser_than_2_5cm")


def blind_bins_for_min_range(config: Ping360Config, range_resolution: float) -> int:
    return int(np.ceil(config.min_range_m / max(range_resolution, 1.0e-9)))


__all__ = [
    "append_effective_range_flags",
    "blind_bins_for_min_range",
    "effective_range_m",
    "range_resolution_m",
]
