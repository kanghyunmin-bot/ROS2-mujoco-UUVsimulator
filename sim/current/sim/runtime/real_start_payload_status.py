"""Compatibility facade for real-start status and payload assembly."""

from __future__ import annotations

from .real_start_payload_builders import (
    build_latched_payload,
    build_required_payload,
    real_start_not_required_payload,
)
from .real_start_status_eval import determine_real_start_status


__all__ = [
    "build_latched_payload",
    "build_required_payload",
    "determine_real_start_status",
    "real_start_not_required_payload",
]
