"""Shared helpers for UUV MuJoCo CLI construction."""

from __future__ import annotations

import os


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)
