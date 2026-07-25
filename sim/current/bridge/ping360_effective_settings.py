"""Ping360 effective settings contract."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .ping360_constants import PING360_DEG_PER_GRAD


@dataclass
class Ping360EffectiveSettings:
    requested_range_m: float
    effective_range_m: float
    speed_of_sound_mps: float
    number_of_samples: int
    sample_period_ticks: int
    transmit_duration_us: int
    transmit_duration_max_us: int
    transmit_frequency_khz: int
    gain_setting: int
    num_steps: int
    angular_resolution_deg: float
    start_angle_grad: int
    stop_angle_grad: int
    sector_size_grad: int
    sector_size_deg: float
    profile_period_s: float
    scan_period_s: float
    interface_mode: str
    range_resolution_m: float
    blind_bins: int
    quality_flags: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "requested_range_m": self.requested_range_m,
            "effective_range_m": self.effective_range_m,
            "speed_of_sound_mps": self.speed_of_sound_mps,
            "number_of_samples": self.number_of_samples,
            "sample_period_ticks": self.sample_period_ticks,
            "sample_period_us": self.sample_period_ticks * 0.025,
            "transmit_duration_us": self.transmit_duration_us,
            "transmit_duration_max_us": self.transmit_duration_max_us,
            "transmit_frequency_khz": self.transmit_frequency_khz,
            "gain_setting": self.gain_setting,
            "num_steps": self.num_steps,
            "angular_resolution_deg": self.angular_resolution_deg,
            "start_angle_grad": self.start_angle_grad,
            "start_angle_deg": self.start_angle_grad * PING360_DEG_PER_GRAD,
            "stop_angle_grad": self.stop_angle_grad,
            "stop_angle_deg": self.stop_angle_grad * PING360_DEG_PER_GRAD,
            "sector_size_grad": self.sector_size_grad,
            "sector_size_deg": self.sector_size_deg,
            "profile_period_s": self.profile_period_s,
            "scan_period_s": self.scan_period_s,
            "interface_mode": self.interface_mode,
            "range_resolution_m": self.range_resolution_m,
            "blind_bins": self.blind_bins,
            "quality_flags": self.quality_flags,
        }


__all__ = ["Ping360EffectiveSettings"]
