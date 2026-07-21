"""Environment parsing helpers for SITL transport configuration."""

from __future__ import annotations

import os

import numpy as np


TRUE_ENV_VALUES = {"1", "true", "yes", "on", "enable", "enabled"}


def env_to_int(env_name: str, default: int, *, log_prefix: str = "[sitl_transport]") -> int:
    value = os.getenv(env_name)
    if not value:
        return int(default)
    try:
        parsed = int(value)
    except ValueError:
        print(f"{log_prefix} invalid {env_name}={value!r}, using {default}", flush=True)
        return int(default)
    return max(0, parsed)


def env_to_pwm(env_name: str, default: int, *, log_prefix: str = "[sitl_transport]") -> int:
    value = os.getenv(env_name)
    if value:
        try:
            default = int(value)
        except ValueError:
            print(f"{log_prefix} invalid {env_name}={value!r}, using {default}", flush=True)
    return int(np.clip(int(default), 1100, 1900))


def env_to_float(env_name: str, default: float, *, log_prefix: str = "[sitl_transport]") -> float:
    value = os.getenv(env_name)
    if not value:
        return float(default)
    try:
        parsed = float(value)
    except ValueError:
        print(f"{log_prefix} invalid {env_name}={value!r}, using {default}", flush=True)
        return float(default)
    return float(parsed)


def env_flag(env_name: str, default: bool) -> bool:
    value = os.getenv(env_name)
    if value is None or value == "":
        return bool(default)
    return value.strip().lower() in TRUE_ENV_VALUES


def finite_or_zero(value: float) -> float:
    return float(value) if np.isfinite(value) else 0.0
