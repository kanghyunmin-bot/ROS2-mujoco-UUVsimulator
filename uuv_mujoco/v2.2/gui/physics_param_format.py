"""JSON profile and value formatting helpers for GUI physics parameters."""

from __future__ import annotations

from typing import Any

from .physics_param_nested import _get_nested_value, _set_nested_value
from .physics_profile_select import _current_physics_profile


def _format_physics_value(value: Any) -> str:
    if isinstance(value, (list, tuple)):
        return " ".join(f"{float(item):g}" for item in value)
    return f"{float(value):g}"


__all__ = [
    "_current_physics_profile",
    "_format_physics_value",
    "_get_nested_value",
    "_set_nested_value",
]
