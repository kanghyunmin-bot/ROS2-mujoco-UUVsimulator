"""Configuration helpers for JSON-SITL servo receiver sockets."""

from __future__ import annotations

import os


def positive_env_int(name: str, default: int) -> int:
    try:
        value = int(os.getenv(name, str(default)))
    except ValueError:
        return int(default)
    return int(value) if value > 0 else int(default)


__all__ = ["positive_env_int"]
