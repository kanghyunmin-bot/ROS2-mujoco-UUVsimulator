"""Compatibility facade for Ping360 binary STL IO helpers."""

from __future__ import annotations

from filter_ping360_stl_geometry import compute_bbox, face_normal, offset_triangle
from filter_ping360_stl_read import read_binary_stl
from filter_ping360_stl_types import HEADER_BYTES, TRIANGLE_BYTES, Triangle
from filter_ping360_stl_write import write_binary_stl


__all__ = [
    "HEADER_BYTES",
    "TRIANGLE_BYTES",
    "Triangle",
    "compute_bbox",
    "face_normal",
    "offset_triangle",
    "read_binary_stl",
    "write_binary_stl",
]
