"""Ping360 sonar echo ROS2 message builder."""

from __future__ import annotations

from array import array
from typing import Any

import numpy as np

from .ping360_types import PING360_GRADS_PER_REV, Ping360Config, Ping360Sample


def build_ping360_echo_msg(sonar_echo_type: type, stamp: Any, sample: Ping360Sample, config: Ping360Config) -> Any:
    msg = sonar_echo_type()
    msg.header.stamp = stamp
    msg.header.frame_id = config.frame_id
    msg.angle = float(2.0 * np.pi * float(sample.angle_grad) / float(PING360_GRADS_PER_REV))
    msg.gain = int(np.clip(sample.settings.gain_setting, 0, 255))
    msg.number_of_samples = int(np.clip(sample.settings.number_of_samples, 0, 65535))
    msg.transmit_frequency = int(np.clip(sample.settings.transmit_frequency_khz, 0, 65535))
    msg.speed_of_sound = int(np.clip(round(sample.settings.speed_of_sound_mps), 0, 65535))
    msg.range = int(np.clip(round(sample.settings.effective_range_m), 0, 255))
    msg.intensities = array("B", np.asarray(sample.profile, dtype=np.uint8).tobytes())
    return msg


__all__ = ["build_ping360_echo_msg"]
