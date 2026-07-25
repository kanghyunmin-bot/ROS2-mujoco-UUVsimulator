"""GUI backend-name normalization helpers."""

from __future__ import annotations

from .config import BACKEND_AUTO, BACKEND_MAVROS, BACKEND_NONE, BACKEND_SIM_BRIDGE


def normalize_backend_name(name: str) -> str:
    value = str(name or BACKEND_AUTO).strip().lower()
    if value in ("none", "off", BACKEND_NONE):
        return BACKEND_NONE
    if value in ("sim", BACKEND_SIM_BRIDGE):
        return BACKEND_SIM_BRIDGE
    if value == BACKEND_MAVROS:
        return BACKEND_MAVROS
    return BACKEND_AUTO
