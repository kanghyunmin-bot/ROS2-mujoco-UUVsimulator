#!/usr/bin/env python3
"""Regression smoke for Ping360 binary STL IO helpers."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
for item in (ROOT, TOOLS):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))

from filter_ping360_stl_io import compute_bbox, face_normal, read_binary_stl, write_binary_stl  # noqa: E402


def main() -> int:
    triangle = (
        (0.0, 0.0, 1.0),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        7,
    )
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "sample.stl"
        write_binary_stl(path, [triangle], (0.0, 0.0, 0.0))
        header, triangles = read_binary_stl(path)
    assert len(header) == 80
    assert len(triangles) == 1
    assert triangles[0][2] == 7
    assert compute_bbox(triangles) == ([0.0, 0.0, 0.0], [1.0, 1.0, 0.0])
    assert face_normal(triangles[0][1]) == (0.0, 0.0, 1.0)
    print("filter_ping360_stl_io=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
