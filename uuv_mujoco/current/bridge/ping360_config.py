"""Ping360 configuration contract and JSON loading."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Ping360Config:
    enabled: bool = True
    site_name: str = "ping360_site"
    frame_id: str = "ping360_link"
    interface_mode: str = "ethernet"
    requested_range_m: float = 2.0
    speed_of_sound_mps: float = 1500.0
    gain_setting: int = 0
    transmit_frequency_khz: int = 750
    auto_transmit_duration: bool = True
    transmit_duration_us: int = 11
    start_angle_grad: int = 0
    stop_angle_grad: int = 399
    sector_size_deg: float = 360.0
    angle_offset_grad: int = 200
    num_steps: int = 1
    sector_bounce: bool = True
    horizontal_beamwidth_deg: float = 2.0
    vertical_beamwidth_deg: float = 25.0
    horizontal_ray_count: int = 3
    vertical_ray_count: int = 7
    noise_floor: float = 2.0
    speckle_std: float = 4.0
    absorption_db_per_m: float = 0.12
    range_power_loss: float = 1.35
    image_size_px: int = 640
    image_display_gain: float = 12.0
    publish_image: bool = True
    publish_scan: bool = True
    publish_echo: bool = True
    publish_status: bool = True

    # Firmware / physical limits, kept close to Ping-Viewer and Ping Protocol.
    min_range_m: float = 0.75
    max_range_m: float = 50.0
    min_number_of_samples: int = 200
    max_number_of_samples: int = 1200
    min_sample_period_ticks: int = 80
    max_sample_period_ticks: int = 40000
    min_transmit_duration_us: int = 5
    max_transmit_duration_us: int = 500
    min_transmit_frequency_khz: int = 500
    max_transmit_frequency_khz: int = 1000
    practical_min_transmit_frequency_khz: int = 650
    practical_max_transmit_frequency_khz: int = 850
    min_num_steps: int = 1
    max_num_steps: int = 10

    @classmethod
    def from_file(cls, path: str | Path | None, overrides: dict[str, Any] | None = None) -> "Ping360Config":
        data = load_ping360_config_data(path, overrides)
        known = {field_name for field_name in cls.__dataclass_fields__}
        return cls(**{key: value for key, value in data.items() if key in known})


def load_ping360_config_data(
    path: str | Path | None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    data: dict[str, Any] = {}
    if path:
        cfg_path = Path(path).expanduser()
        if cfg_path.is_file():
            data = json.loads(cfg_path.read_text(encoding="utf-8"))
    if overrides:
        data.update({key: value for key, value in overrides.items() if value is not None})
    return data


__all__ = ["Ping360Config", "load_ping360_config_data"]
