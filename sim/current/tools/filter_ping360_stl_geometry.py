"""Triangle geometry helpers for Ping360 STL filtering."""

from __future__ import annotations

import math

from filter_ping360_stl_types import Triangle


def compute_bbox(triangles: list[Triangle]) -> tuple[list[float], list[float]]:
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    for _, vertices, _ in triangles:
        for vertex in vertices:
            for axis, value in enumerate(vertex):
                mins[axis] = min(mins[axis], value)
                maxs[axis] = max(maxs[axis], value)
    return mins, maxs


def offset_triangle(triangle: Triangle, offset_mm: tuple[float, float, float]) -> Triangle:
    normal, vertices, attr = triangle
    shifted = tuple(
        (
            vertex[0] - offset_mm[0],
            vertex[1] - offset_mm[1],
            vertex[2] - offset_mm[2],
        )
        for vertex in vertices
    )
    return normal, shifted, attr


def face_normal(vertices: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
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


__all__ = ["compute_bbox", "face_normal", "offset_triangle"]
