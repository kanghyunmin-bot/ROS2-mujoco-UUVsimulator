"""Connected-component statistics for Ping360 STL meshes."""

from __future__ import annotations

import math

from filter_ping360_stl_io import Triangle
from filter_ping360_union_find import UnionFind


def component_stats(triangles: list[Triangle], quantize: float) -> tuple[list[dict], list[int]]:
    uf, triangle_vertex_ids = _label_triangle_vertices(triangles, quantize)
    root_for_triangle: list[int] = []
    stats_by_root: dict[int, dict] = {}
    for tri_idx, (_, vertices, _) in enumerate(triangles):
        root = uf.find(triangle_vertex_ids[tri_idx][0])
        root_for_triangle.append(root)
        stats = _root_stats(stats_by_root, root)
        stats["triangles"] += 1
        _expand_bounds(stats, vertices)
    return _finalize_components(stats_by_root), root_for_triangle


def _label_triangle_vertices(
    triangles: list[Triangle],
    quantize: float,
) -> tuple[UnionFind, list[tuple[int, int, int]]]:
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
    return uf, triangle_vertex_ids


def _root_stats(stats_by_root: dict[int, dict], root: int) -> dict:
    return stats_by_root.setdefault(
        root,
        {
            "root": root,
            "triangles": 0,
            "min": [math.inf, math.inf, math.inf],
            "max": [-math.inf, -math.inf, -math.inf],
        },
    )


def _expand_bounds(stats: dict, vertices: tuple[tuple[float, float, float], ...]) -> None:
    for vertex in vertices:
        for axis, value in enumerate(vertex):
            stats["min"][axis] = min(stats["min"][axis], value)
            stats["max"][axis] = max(stats["max"][axis], value)


def _finalize_components(stats_by_root: dict[int, dict]) -> list[dict]:
    components = []
    for stats in stats_by_root.values():
        size = [stats["max"][axis] - stats["min"][axis] for axis in range(3)]
        center = [(stats["max"][axis] + stats["min"][axis]) * 0.5 for axis in range(3)]
        stats["size"] = size
        stats["center"] = center
        components.append(stats)
    components.sort(key=lambda item: item["triangles"], reverse=True)
    return components


__all__ = ["component_stats"]
