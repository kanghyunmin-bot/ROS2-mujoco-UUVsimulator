"""Runtime environment parsing helpers."""

from __future__ import annotations

import os


TRUE_ENV_VALUES = {"1", "true", "yes", "on", "enable", "enabled"}


def env_float(name: str, default: float) -> float:
    """Return an environment variable parsed as float, or default on invalid input."""
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def env_flag(name: str, default: bool = False) -> bool:
    """Return an environment variable parsed as a common truthy flag."""
    value = os.environ.get(name)
    if value is None or value == "":
        return bool(default)
    return value.strip().lower() in TRUE_ENV_VALUES


__all__ = ["TRUE_ENV_VALUES", "env_flag", "env_float"]
