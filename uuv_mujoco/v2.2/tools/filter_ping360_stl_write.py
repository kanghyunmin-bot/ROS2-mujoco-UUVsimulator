"""Binary STL writer for Ping360 mesh filtering."""

from __future__ import annotations

import struct
from pathlib import Path

from filter_ping360_stl_geometry import face_normal, offset_triangle
from filter_ping360_stl_types import Triangle


STL_HEADER = b"Ping360 body without cable, generated for MuJoCo".ljust(80, b" ")


def write_binary_stl(path: Path, triangles: list[Triangle], offset_mm: tuple[float, float, float]) -> None:
    with path.open("wb") as handle:
        handle.write(STL_HEADER)
        handle.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            _, shifted, attr = offset_triangle(triangle, offset_mm)
            normal = face_normal(shifted)
            handle.write(struct.pack("<fff", *normal))
            for vertex in shifted:
                handle.write(struct.pack("<fff", *vertex))
            handle.write(struct.pack("<H", int(attr)))


__all__ = ["STL_HEADER", "write_binary_stl"]
