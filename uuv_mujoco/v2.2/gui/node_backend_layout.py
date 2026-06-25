"""Backend label and RC layout helpers."""

from __future__ import annotations

from typing import Any

from .config import BACKEND_AUTO, DEFAULT_AUTO_BACKEND, RC_LAYOUTS


def effective_backend(owner: Any) -> str:
    if owner._backend_preference != BACKEND_AUTO:
        return owner._backend_preference
    return owner._backend_detected


def active_layout(owner: Any):
    backend = effective_backend(owner)
    return RC_LAYOUTS.get(backend, RC_LAYOUTS[DEFAULT_AUTO_BACKEND])


def backend_label(owner: Any) -> str:
    backend = effective_backend(owner)
    layout = active_layout(owner)
    if owner._backend_preference == BACKEND_AUTO:
        return f"auto->{backend} ({layout.label})"
    return f"{backend} ({layout.label})"


def rc_mapping_summary(owner: Any) -> str:
    return active_layout(owner).summary


__all__ = [
    "active_layout",
    "backend_label",
    "effective_backend",
    "rc_mapping_summary",
]
