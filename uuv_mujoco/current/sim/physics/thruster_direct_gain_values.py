"""Value parsing for direct thruster gain overrides."""

from __future__ import annotations

from typing import Callable

import numpy as np


def parse_direct_gain_scale(raw_value) -> float | None:
    try:
        return float(np.clip(float(raw_value), 0.001, 20.0))
    except (TypeError, ValueError):
        return None


def read_env_direct_gain_scale(env_get: Callable[[str, str], str], env_name: str) -> tuple[str, float | None]:
    raw_value = str(env_get(env_name, "") or "").strip()
    if not raw_value:
        return "", None
    return raw_value, parse_direct_gain_scale(raw_value)


__all__ = ["parse_direct_gain_scale", "read_env_direct_gain_scale"]
