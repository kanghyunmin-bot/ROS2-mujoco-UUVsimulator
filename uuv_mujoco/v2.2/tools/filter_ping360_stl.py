#!/usr/bin/env python3
"""Create a Ping360 visual mesh with the cable assembly removed.

The official assembly STL includes a long external cable. For simulation the
sonar body should be mounted on the vehicle, while the cable would create a
large irrelevant visual/collision extent. This script keeps the upper sonar
body components and removes long, slender cable-like components.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path


TRIANGLE_BYTES = 50
HEADER_BYTES = 84


class UnionFind:
    def __init__(self) -> None:
        self.parent: list[int] = []
        self.rank: list[int] = []

    def make(self) -> int:
        idx = len(self.parent)
        self.parent.append(idx)
        self.rank.append(0)
        return idx

    def find(self, idx: int) -> int:
        while self.parent[idx] != idx:
            self.parent[idx] = self.parent[self.parent[idx]]
            idx = self.parent[idx]
        return idx

    def union(self, a: int, b: int) -> None:
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return
        if self.rank[ra] < self.rank[rb]:
            ra, rb = rb, ra
        self.parent[rb] = ra
        if self.rank[ra] == self.rank[rb]:
            self.rank[ra] += 1


def read_binary_stl(path: Path) -> tuple[bytes, list[tuple[tuple[float, float, float], tuple[tuple[float, float, float], ...], int]]]:
    payload = path.read_bytes()
    if len(payload) < HEADER_BYTES:
        raise ValueError(f"STL too small: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected = HEADER_BYTES + triangle_count * TRIANGLE_BYTES
    if expected != len(payload):
        raise ValueError(f"expected binary STL size {expected}, got {len(payload)}")

    triangles = []
    for idx in range(triangle_count):
        offset = HEADER_BYTES + idx * TRIANGLE_BYTES
        normal = struct.unpack_from("<fff", payload, offset)
        offset += 12
        vertices = []
        for _ in range(3):
            vertices.append(struct.unpack_from("<fff", payload, offset))
            offset += 12
        attr = struct.unpack_from("<H", payload, offset)[0]
        triangles.append((normal, tuple(vertices), attr))
    return payload[:80], triangles


def component_stats(triangles, quantize: float) -> tuple[list[dict], list[int]]:
    uf = UnionFind()
    vertex_ids: dict[tuple[int, int, int], int] = {}
    triangle_vertex_ids: list[tuple[int, int, int]] = []

    for _, vertices, _ in triangles:
        ids = []
        for vertex in vertices:
            key = tuple(int(round(coord * quantize)) for coord in vertex)
            vid = vertex_ids.get(key)
            if vid is None:
                vid = uf.make()
                vertex_ids[key] = vid
            ids.append(vid)
        uf.union(ids[0], ids[1])
        uf.union(ids[1], ids[2])
        triangle_vertex_ids.append((ids[0], ids[1], ids[2]))

    root_for_triangle = []
    stats_by_root: dict[int, dict] = {}
    for tri_idx, (_, vertices, _) in enumerate(triangles):
        root = uf.find(triangle_vertex_ids[tri_idx][0])
        root_for_triangle.append(root)
        stats = stats_by_root.setdefault(
            root,
            {
                "root": root,
                "triangles": 0,
                "min": [math.inf, math.inf, math.inf],
                "max": [-math.inf, -math.inf, -math.inf],
            },
        )
        stats["triangles"] += 1
        for vertex in vertices:
            for axis, value in enumerate(vertex):
                stats["min"][axis] = min(stats["min"][axis], value)
                stats["max"][axis] = max(stats["max"][axis], value)

    components = []
    for stats in stats_by_root.values():
        size = [stats["max"][axis] - stats["min"][axis] for axis in range(3)]
        center = [(stats["max"][axis] + stats["min"][axis]) * 0.5 for axis in range(3)]
        stats["size"] = size
        stats["center"] = center
        components.append(stats)
    components.sort(key=lambda item: item["triangles"], reverse=True)
    return components, root_for_triangle


def is_cable_like(component: dict, cable_min_length_mm: float, cable_max_thickness_mm: float) -> bool:
    dims = sorted(component["size"], reverse=True)
    return dims[0] >= cable_min_length_mm and dims[1] <= cable_max_thickness_mm


def classify_components(components: list[dict], *, keep_min_z_mm: float, cable_min_length_mm: float, cable_max_thickness_mm: float) -> set[int]:
    keep_roots = set()
    for component in components:
        cable_like = is_cable_like(component, cable_min_length_mm, cable_max_thickness_mm)
        component["cable_like"] = cable_like
        component["kept"] = (not cable_like) and component["max"][2] >= keep_min_z_mm
        if component["kept"]:
            keep_roots.add(component["root"])
    return keep_roots


def compute_bbox(triangles) -> tuple[list[float], list[float]]:
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    for _, vertices, _ in triangles:
        for vertex in vertices:
            for axis, value in enumerate(vertex):
                mins[axis] = min(mins[axis], value)
                maxs[axis] = max(maxs[axis], value)
    return mins, maxs


def face_normal(vertices) -> tuple[float, float, float]:
    ax, ay, az = vertices[0]
    bx, by, bz = vertices[1]
    cx, cy, cz = vertices[2]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0
    return nx / norm, ny / norm, nz / norm


def write_binary_stl(path: Path, triangles, offset_mm: tuple[float, float, float]) -> None:
    header = b"Ping360 body without cable, generated for MuJoCo".ljust(80, b" ")
    with path.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for _, vertices, attr in triangles:
            shifted = tuple(
                (
                    vertex[0] - offset_mm[0],
                    vertex[1] - offset_mm[1],
                    vertex[2] - offset_mm[2],
                )
                for vertex in vertices
            )
            normal = face_normal(shifted)
            f.write(struct.pack("<fff", *normal))
            for vertex in shifted:
                f.write(struct.pack("<fff", *vertex))
            f.write(struct.pack("<H", int(attr)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Source Ping360 assembly STL")
    parser.add_argument("output", type=Path, help="Filtered output STL")
    parser.add_argument("--metadata", type=Path, default=None, help="Optional JSON metadata output")
    parser.add_argument("--keep-min-z-mm", type=float, default=840.0)
    parser.add_argument("--cable-min-length-mm", type=float, default=500.0)
    parser.add_argument("--cable-max-thickness-mm", type=float, default=8.0)
    parser.add_argument("--quantize", type=float, default=10000.0)
    args = parser.parse_args()

    _, triangles = read_binary_stl(args.input)
    components, root_for_triangle = component_stats(triangles, args.quantize)
    keep_roots = classify_components(
        components,
        keep_min_z_mm=args.keep_min_z_mm,
        cable_min_length_mm=args.cable_min_length_mm,
        cable_max_thickness_mm=args.cable_max_thickness_mm,
    )

    kept_triangles = [triangle for triangle, root in zip(triangles, root_for_triangle) if root in keep_roots]
    if not kept_triangles:
        raise SystemExit("no triangles selected; adjust filtering thresholds")

    bbox_min, bbox_max = compute_bbox(kept_triangles)
    bbox_center = tuple((bbox_min[axis] + bbox_max[axis]) * 0.5 for axis in range(3))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_binary_stl(args.output, kept_triangles, bbox_center)

    out_bbox_min, out_bbox_max = compute_bbox(
        [
            (
                normal,
                tuple(
                    (
                        vertex[0] - bbox_center[0],
                        vertex[1] - bbox_center[1],
                        vertex[2] - bbox_center[2],
                    )
                    for vertex in vertices
                ),
                attr,
            )
            for normal, vertices, attr in kept_triangles
        ]
    )
    metadata = {
        "source": str(args.input),
        "output": str(args.output),
        "source_triangle_count": len(triangles),
        "kept_triangle_count": len(kept_triangles),
        "component_count": len(components),
        "source_units": "millimeter",
        "output_units": "millimeter",
        "source_offset_removed_mm": list(bbox_center),
        "output_bbox_min_mm": out_bbox_min,
        "output_bbox_max_mm": out_bbox_max,
        "output_size_mm": [out_bbox_max[axis] - out_bbox_min[axis] for axis in range(3)],
        "components": components,
    }
    if args.metadata is not None:
        args.metadata.parent.mkdir(parents=True, exist_ok=True)
        args.metadata.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print(
        "kept "
        f"{len(kept_triangles)}/{len(triangles)} triangles; "
        f"bbox_mm={metadata['output_size_mm']}; "
        f"offset_mm={metadata['source_offset_removed_mm']}"
    )


if __name__ == "__main__":
    main()
