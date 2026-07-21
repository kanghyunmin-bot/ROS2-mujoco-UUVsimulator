"""Current-mode status helpers for GUI physics parameters."""

from __future__ import annotations

from .config import CURRENT_MODE_INACTIVE_PHYSICS_KEYS


def _physics_param_inactive_in_current(key: str) -> bool:
    return key in CURRENT_MODE_INACTIVE_PHYSICS_KEYS


def _physics_current_mode_status(key: str) -> str:
    reason = CURRENT_MODE_INACTIVE_PHYSICS_KEYS.get(key)
    if reason:
        return f"inactive: {reason}"
    return "active"


__all__ = ["_physics_current_mode_status", "_physics_param_inactive_in_current"]
