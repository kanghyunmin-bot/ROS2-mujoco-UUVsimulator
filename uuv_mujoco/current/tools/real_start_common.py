"""Shared scalar helpers for real-start state extraction."""

from __future__ import annotations

import math
from typing import Any


def finite(value: Any, default: float = math.nan) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def truthy(value: Any) -> bool:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "armed"}:
        return True
    if text in {"0", "false", "no", "disarmed", ""}:
        return False
    value_f = finite(value, 0.0)
    return bool(value_f)
