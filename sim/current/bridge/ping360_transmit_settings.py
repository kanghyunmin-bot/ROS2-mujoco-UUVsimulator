"""Ping360 transmit duration, frequency, and gain effective settings."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ping360_range_settings import Ping360RangeSettings
from .ping360_types import SAMPLE_PERIOD_TICK_S, Ping360Config


@dataclass(frozen=True)
class Ping360TransmitSettings:
    transmit_duration_us: int
    transmit_duration_max_us: int
    transmit_frequency_khz: int
    gain_setting: int
    quality_flags: list[str]


def build_transmit_settings(config: Ping360Config, range_settings: Ping360RangeSettings) -> Ping360TransmitSettings:
    quality_flags: list[str] = []
    transmit_duration_max = min(
        int(config.max_transmit_duration_us),
        int(range_settings.sample_period_ticks * SAMPLE_PERIOD_TICK_S * 64.0e6),
    )
    auto_duration = int(round(8000.0 * range_settings.effective_range_m / range_settings.speed_of_sound_mps))
    sample_interval_us = range_settings.sample_period_ticks * SAMPLE_PERIOD_TICK_S * 1.0e6
    auto_duration = max(int(round(2.5 * sample_interval_us)), auto_duration)
    requested_duration = auto_duration if config.auto_transmit_duration else int(config.transmit_duration_us)
    transmit_duration = int(
        np.clip(
            requested_duration,
            config.min_transmit_duration_us,
            max(config.min_transmit_duration_us, transmit_duration_max),
        )
    )
    if transmit_duration != requested_duration:
        quality_flags.append("transmit_duration_clamped")
    if transmit_duration >= transmit_duration_max:
        quality_flags.append("transmit_duration_at_firmware_limit")

    transmit_frequency = int(
        np.clip(
            config.transmit_frequency_khz,
            config.min_transmit_frequency_khz,
            config.max_transmit_frequency_khz,
        )
    )
    if not (config.practical_min_transmit_frequency_khz <= transmit_frequency <= config.practical_max_transmit_frequency_khz):
        quality_flags.append("transmit_frequency_outside_practical_band")

    gain = int(np.clip(config.gain_setting, 0, 2))
    if gain >= 2:
        quality_flags.append("high_gain_saturation_risk")
    return Ping360TransmitSettings(
        transmit_duration_us=transmit_duration,
        transmit_duration_max_us=transmit_duration_max,
        transmit_frequency_khz=transmit_frequency,
        gain_setting=gain,
        quality_flags=quality_flags,
    )


__all__ = ["Ping360TransmitSettings", "build_transmit_settings"]
