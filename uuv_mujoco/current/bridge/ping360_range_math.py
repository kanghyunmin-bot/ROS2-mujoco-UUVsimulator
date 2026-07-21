"""Range math helpers for Ping360 effective settings."""

from __future__ import annotations

import numpy as np

from .ping360_types import SAMPLE_PERIOD_TICK_S, Ping360Config


def calculate_sample_period(range_m: float, num_samples: int, speed_of_sound: float) -> int:
    if range_m <= 0.0 or num_samples <= 0 or speed_of_sound <= 0.0:
        return 88
    return int(2.0 * range_m / (num_samples * speed_of_sound * SAMPLE_PERIOD_TICK_S))


def clipped_requested_range(config: Ping360Config, quality_flags: list[str]) -> float:
    requested_range = float(config.requested_range_m)
    range_m = float(np.clip(requested_range, config.min_range_m, config.max_range_m))
    if abs(range_m - requested_range) > 1.0e-6:
        quality_flags.append("range_clamped_to_physical_limits")
    return range_m


def reduce_samples_for_period(config: Ping360Config, range_m: float, speed_of_sound: float) -> tuple[int, int]:
    num_samples = int(config.max_number_of_samples)
    sample_period = calculate_sample_period(range_m, num_samples, speed_of_sound)
    while sample_period < config.min_sample_period_ticks and num_samples > config.min_number_of_samples:
        num_samples -= 1
        sample_period = calculate_sample_period(range_m, num_samples, speed_of_sound)
    sample_period = int(np.clip(sample_period, config.min_sample_period_ticks, config.max_sample_period_ticks))
    return num_samples, sample_period


__all__ = ["calculate_sample_period", "clipped_requested_range", "reduce_samples_for_period"]
