"""Environment parsing helpers for GUI configuration."""

from __future__ import annotations

import os


def env_float_default(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def runtime_profile() -> str:
    profile = os.environ.get("UUV_RUNTIME_PROFILE", "balanced").strip().lower()
    if profile in {"low", "balanced", "high"}:
        return profile
    return "balanced"


def profile_default_update_ms() -> int:
    return {
        "low": 200,
        "balanced": 25,
        "high": 20,
    }[runtime_profile()]


def env_int(name: str, default: int, min_value: int, max_value: int) -> int:
    try:
        value = int(os.environ.get(name, default))
    except (TypeError, ValueError):
        value = default
    return max(min_value, min(max_value, value))


__all__ = [
    "env_float_default",
    "env_int",
    "profile_default_update_ms",
    "runtime_profile",
]
