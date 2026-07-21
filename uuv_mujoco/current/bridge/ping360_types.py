"""Compatibility facade for Ping360 constants and data contracts."""

from __future__ import annotations

from .ping360_config import Ping360Config, load_ping360_config_data
from .ping360_constants import PING360_DEG_PER_GRAD, PING360_GRADS_PER_REV, SAMPLE_PERIOD_TICK_S
from .ping360_effective_settings import Ping360EffectiveSettings
from .ping360_sample import Ping360Sample


__all__ = [
    "PING360_DEG_PER_GRAD",
    "PING360_GRADS_PER_REV",
    "SAMPLE_PERIOD_TICK_S",
    "Ping360Config",
    "Ping360EffectiveSettings",
    "Ping360Sample",
    "load_ping360_config_data",
]
