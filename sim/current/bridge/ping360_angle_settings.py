"""Ping360 angular sector and scan-period effective settings."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ping360_interface_timing import normalize_interface, profile_period_s
from .ping360_types import PING360_DEG_PER_GRAD, PING360_GRADS_PER_REV, Ping360Config


@dataclass(frozen=True)
class Ping360AngleSettings:
    num_steps: int
    angular_resolution_deg: float
    start_angle_grad: int
    stop_angle_grad: int
    sector_size_grad: int
    sector_size_deg: float
    profile_period_s: float
    scan_period_s: float
    interface_mode: str
    quality_flags: list[str]


def build_angle_settings(config: Ping360Config, effective_range_m: float) -> Ping360AngleSettings:
    quality_flags: list[str] = []
    num_steps = int(np.clip(config.num_steps, config.min_num_steps, config.max_num_steps))
    if num_steps > 1:
        quality_flags.append("angular_resolution_reduced_for_speed")

    start_angle = int(round(float(config.start_angle_grad))) % PING360_GRADS_PER_REV
    stop_angle = int(round(float(config.stop_angle_grad))) % PING360_GRADS_PER_REV
    sector_grad = ((stop_angle - start_angle) % PING360_GRADS_PER_REV) + 1
    profile_period = profile_period_s(effective_range_m, num_steps, config.interface_mode)
    scan_period = np.ceil(sector_grad / num_steps) * profile_period
    return Ping360AngleSettings(
        num_steps=num_steps,
        angular_resolution_deg=num_steps * PING360_DEG_PER_GRAD,
        start_angle_grad=start_angle,
        stop_angle_grad=stop_angle,
        sector_size_grad=sector_grad,
        sector_size_deg=sector_grad * PING360_DEG_PER_GRAD,
        profile_period_s=profile_period,
        scan_period_s=float(scan_period),
        interface_mode=normalize_interface(config.interface_mode),
        quality_flags=quality_flags,
    )


__all__ = ["Ping360AngleSettings", "build_angle_settings"]
