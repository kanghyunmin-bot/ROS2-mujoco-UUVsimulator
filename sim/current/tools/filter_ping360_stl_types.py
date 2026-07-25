"""Shared binary STL types/constants for Ping360 filtering."""

from __future__ import annotations


Triangle = tuple[
    tuple[float, float, float],
    tuple[tuple[float, float, float], ...],
    int,
]

TRIANGLE_BYTES = 50
HEADER_BYTES = 84


__all__ = ["HEADER_BYTES", "TRIANGLE_BYTES", "Triangle"]
