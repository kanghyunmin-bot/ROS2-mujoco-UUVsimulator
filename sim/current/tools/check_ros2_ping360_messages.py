#!/usr/bin/env python3
"""Smoke-check Ping360 ROS message builders without ROS imports."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ping360_image_renderer import Ping360ImageRenderer  # noqa: E402
from bridge.ping360_types import Ping360Config, Ping360EffectiveSettings, Ping360Sample  # noqa: E402
from bridge.ros2_ping360_echo_message import build_ping360_echo_msg  # noqa: E402
from bridge.ros2_ping360_image_message import build_ping360_image_msg  # noqa: E402
from bridge.ros2_ping360_scan_message import build_ping360_scan_msg  # noqa: E402
from bridge.ros2_ping360_status_message import (  # noqa: E402
    build_ping360_status_msg,
    build_ping360_status_payload,
)


class Header:
    def __init__(self) -> None:
        self.stamp = None
        self.frame_id = ""


class AutoMsg:
    def __init__(self) -> None:
        self.header = Header()


class StringMsg:
    def __init__(self) -> None:
        self.data = ""


def _settings() -> Ping360EffectiveSettings:
    return Ping360EffectiveSettings(
        requested_range_m=2.0,
        effective_range_m=2.0,
        speed_of_sound_mps=1500.0,
        number_of_samples=200,
        sample_period_ticks=80,
        transmit_duration_us=11,
        transmit_duration_max_us=500,
        transmit_frequency_khz=750,
        gain_setting=6,
        num_steps=1,
        angular_resolution_deg=0.9,
        start_angle_grad=0,
        stop_angle_grad=399,
        sector_size_grad=400,
        sector_size_deg=360.0,
        profile_period_s=0.02,
        scan_period_s=8.0,
        interface_mode="ethernet",
        range_resolution_m=0.01,
        blind_bins=3,
    )


def _sample() -> Ping360Sample:
    profile = np.arange(200, dtype=np.uint8)
    return Ping360Sample(
        sim_time_s=1.25,
        angle_grad=100,
        profile=profile,
        image=np.tile(profile, (400, 1)),
        ranges_m=np.linspace(0.75, 2.0, 400, dtype=np.float32),
        intensities=np.linspace(0.0, 255.0, 400, dtype=np.float32),
        settings=_settings(),
        ping_number=3,
        updated=True,
    )


def main() -> int:
    stamp = SimpleNamespace(sec=12, nanosec=34)
    config = Ping360Config(frame_id="ping360_link", image_size_px=128)
    sample = _sample()

    status_payload = build_ping360_status_payload(
        sim_t=sample.sim_time_s,
        config=config,
        ping360=SimpleNamespace(active=True),
        sample=sample,
        site_present=True,
    )
    status_msg = build_ping360_status_msg(StringMsg, stamp, status_payload)
    status = json.loads(status_msg.data)
    assert status["stamp"] == {"sec": 12, "nanosec": 34}
    assert status["enabled"] is True
    assert status["updated"] is True

    scan = build_ping360_scan_msg(AutoMsg, stamp, sample, config)
    assert scan.header.stamp is stamp
    assert scan.header.frame_id == "ping360_link"
    assert len(scan.ranges) == 400
    assert len(scan.intensities) == 400
    assert scan.range_min == config.min_range_m
    assert scan.range_max == sample.settings.effective_range_m

    echo = build_ping360_echo_msg(AutoMsg, stamp, sample, config)
    assert echo.header.frame_id == "ping360_link"
    assert echo.number_of_samples == 200
    assert echo.transmit_frequency == 750
    assert len(echo.intensities) == 200

    image = build_ping360_image_msg(AutoMsg, stamp, sample, config, Ping360ImageRenderer())
    assert image.header.frame_id == "ping360_link"
    assert image.height == 128
    assert image.width == 128
    assert image.encoding == "mono8"
    assert len(image.data) == 128 * 128

    print("ros2_ping360_messages=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
