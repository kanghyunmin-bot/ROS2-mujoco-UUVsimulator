"""Ping360 firmware-style effective setting calculations."""

from __future__ import annotations

from .ping360_angle_settings import build_angle_settings
from .ping360_interface_timing import (
    INTERFACE_FULL_SCAN_S,
    full_scan_period_one_grad_s,
    normalize_interface,
    profile_period_s,
)
from .ping360_range_settings import build_range_settings, calculate_sample_period
from .ping360_transmit_settings import build_transmit_settings
from .ping360_types import Ping360Config, Ping360EffectiveSettings


def build_effective_settings(config: Ping360Config) -> Ping360EffectiveSettings:
    range_settings = build_range_settings(config)
    transmit_settings = build_transmit_settings(config, range_settings)
    angle_settings = build_angle_settings(config, range_settings.effective_range_m)
    quality_flags = [
        *range_settings.quality_flags,
        *transmit_settings.quality_flags,
        *angle_settings.quality_flags,
    ]
    return Ping360EffectiveSettings(
        requested_range_m=range_settings.requested_range_m,
        effective_range_m=range_settings.effective_range_m,
        speed_of_sound_mps=range_settings.speed_of_sound_mps,
        number_of_samples=range_settings.number_of_samples,
        sample_period_ticks=range_settings.sample_period_ticks,
        transmit_duration_us=transmit_settings.transmit_duration_us,
        transmit_duration_max_us=transmit_settings.transmit_duration_max_us,
        transmit_frequency_khz=transmit_settings.transmit_frequency_khz,
        gain_setting=transmit_settings.gain_setting,
        num_steps=angle_settings.num_steps,
        angular_resolution_deg=angle_settings.angular_resolution_deg,
        start_angle_grad=angle_settings.start_angle_grad,
        stop_angle_grad=angle_settings.stop_angle_grad,
        sector_size_grad=angle_settings.sector_size_grad,
        sector_size_deg=angle_settings.sector_size_deg,
        profile_period_s=angle_settings.profile_period_s,
        scan_period_s=angle_settings.scan_period_s,
        interface_mode=angle_settings.interface_mode,
        range_resolution_m=range_settings.range_resolution_m,
        blind_bins=range_settings.blind_bins,
        quality_flags=quality_flags,
    )


__all__ = [
    "INTERFACE_FULL_SCAN_S",
    "build_effective_settings",
    "calculate_sample_period",
    "full_scan_period_one_grad_s",
    "normalize_interface",
    "profile_period_s",
]
