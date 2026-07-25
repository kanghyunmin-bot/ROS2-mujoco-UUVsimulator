"""Binary STL reader for Ping360 mesh filtering."""

from __future__ import annotations

import struct
from pathlib import Path

from filter_ping360_stl_types import HEADER_BYTES, TRIANGLE_BYTES, Triangle


def _triangle_from_payload(payload: bytes, offset: int) -> Triangle:
    normal = struct.unpack_from("<fff", payload, offset)
    vertex_offset = offset + 12
    vertices = tuple(
        struct.unpack_from("<fff", payload, vertex_offset + idx * 12)
        for idx in range(3)
    )
    attr = struct.unpack_from("<H", payload, vertex_offset + 36)[0]
    return normal, vertices, attr


def read_binary_stl(path: Path) -> tuple[bytes, list[Triangle]]:
    payload = path.read_bytes()
    if len(payload) < HEADER_BYTES:
        raise ValueError(f"STL too small: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected = HEADER_BYTES + triangle_count * TRIANGLE_BYTES
    if expected != len(payload):
        raise ValueError(f"expected binary STL size {expected}, got {len(payload)}")
    triangles = [
        _triangle_from_payload(payload, HEADER_BYTES + idx * TRIANGLE_BYTES)
        for idx in range(triangle_count)
    ]
    return payload[:80], triangles


__all__ = ["read_binary_stl"]
