#!/usr/bin/env python3
"""Generate a first-pass STL for a front-open buoy collector frame.

The model is intentionally simple and simulation-friendly:
- clean low PVC basket frame
- side/back/bottom/top net proxy panels constrained inside the frame
- front face stays open so buoys can enter the capture pocket
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = ROOT / "generated_meshes"
PREVIEW_DIR = OUT_DIR / "previews"
MODEL_NAME = "front_open_buoy_collector_v8"
STL_PATH = OUT_DIR / f"{MODEL_NAME}.stl"
PREVIEW_ISO = PREVIEW_DIR / f"{MODEL_NAME}_iso.png"
PREVIEW_FRONT = PREVIEW_DIR / f"{MODEL_NAME}_front.png"
PREVIEW_MOUTH = PREVIEW_DIR / f"{MODEL_NAME}_mouth.png"
SPEC_PATH = OUT_DIR / f"{MODEL_NAME}_spec.txt"


FRONT_X = 0.17
BACK_X = -0.17
HALF_WIDTH = 0.172
BOTTOM_Z = 0.00
TOP_Z = 0.307
PIPE_R = 0.010


@dataclass
class Triangle:
    vertices: np.ndarray
    color: tuple[float, float, float, float]


def unit(vec: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vec))
    if length <= 1e-12:
        raise ValueError("zero-length vector")
    return vec / length


def basis_from_axis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w = unit(axis)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(w, ref))) > 0.92:
        ref = np.array([0.0, 1.0, 0.0])
    u = unit(np.cross(ref, w))
    v = unit(np.cross(w, u))
    return u, v, w


def normal_of(tri: np.ndarray) -> np.ndarray:
    n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    length = float(np.linalg.norm(n))
    if length <= 1e-12:
        return np.zeros(3)
    return n / length


def add_tri(tris: list[Triangle], verts: list[np.ndarray], color: tuple[float, float, float, float]) -> None:
    tris.append(Triangle(np.array(verts, dtype=float), color))


def add_quad(
    tris: list[Triangle],
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    d: np.ndarray,
    color: tuple[float, float, float, float],
) -> None:
    add_tri(tris, [a, b, c], color)
    add_tri(tris, [a, c, d], color)


def add_box(
    tris: list[Triangle],
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: tuple[float, float, float, float],
) -> None:
    cx, cy, cz = center
    sx, sy, sz = size
    x0, x1 = cx - sx, cx + sx
    y0, y1 = cy - sy, cy + sy
    z0, z1 = cz - sz, cz + sz
    p = {
        "000": np.array([x0, y0, z0]),
        "001": np.array([x0, y0, z1]),
        "010": np.array([x0, y1, z0]),
        "011": np.array([x0, y1, z1]),
        "100": np.array([x1, y0, z0]),
        "101": np.array([x1, y0, z1]),
        "110": np.array([x1, y1, z0]),
        "111": np.array([x1, y1, z1]),
    }
    faces = [
        ("100", "110", "111", "101"),
        ("000", "001", "011", "010"),
        ("010", "011", "111", "110"),
        ("000", "100", "101", "001"),
        ("001", "101", "111", "011"),
        ("000", "010", "110", "100"),
    ]
    for face in faces:
        add_quad(tris, *(p[k] for k in face), color=color)


def add_cylinder(
    tris: list[Triangle],
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    color: tuple[float, float, float, float],
    segments: int = 20,
    caps: bool = True,
) -> None:
    a = np.array(p0, dtype=float)
    b = np.array(p1, dtype=float)
    u, v, _ = basis_from_axis(b - a)
    c0 = []
    c1 = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        offset = radius * (math.cos(theta) * u + math.sin(theta) * v)
        c0.append(a + offset)
        c1.append(b + offset)
    for i in range(segments):
        j = (i + 1) % segments
        add_quad(tris, c0[i], c0[j], c1[j], c1[i], color)
        if caps:
            add_tri(tris, [a, c0[j], c0[i]], color)
            add_tri(tris, [b, c1[i], c1[j]], color)


def add_ellipsoid(
    tris: list[Triangle],
    center: tuple[float, float, float],
    radii: tuple[float, float, float],
    color: tuple[float, float, float, float],
    lat: int = 14,
    lon: int = 28,
) -> None:
    c = np.array(center, dtype=float)
    rx, ry, rz = radii
    pts: list[list[np.ndarray]] = []
    for i in range(lat + 1):
        phi = -0.5 * math.pi + math.pi * i / lat
        row = []
        for j in range(lon):
            theta = 2.0 * math.pi * j / lon
            row.append(
                c
                + np.array(
                    [
                        rx * math.cos(phi) * math.cos(theta),
                        ry * math.cos(phi) * math.sin(theta),
                        rz * math.sin(phi),
                    ]
                )
            )
        pts.append(row)
    for i in range(lat):
        for j in range(lon):
            k = (j + 1) % lon
            if i == 0:
                add_tri(tris, [pts[i][j], pts[i + 1][k], pts[i + 1][j]], color)
            elif i == lat - 1:
                add_tri(tris, [pts[i][j], pts[i][k], pts[i + 1][j]], color)
            else:
                add_quad(tris, pts[i][j], pts[i][k], pts[i + 1][k], pts[i + 1][j], color)


def add_ramp_prism(
    tris: list[Triangle],
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z_front: float,
    z_back: float,
    thickness: float,
    color: tuple[float, float, float, float],
) -> None:
    top = {
        "fl": np.array([x1, y0, z_front]),
        "fr": np.array([x1, y1, z_front]),
        "bl": np.array([x0, y0, z_back]),
        "br": np.array([x0, y1, z_back]),
    }
    bot = {k: v - np.array([0.0, 0.0, thickness]) for k, v in top.items()}
    add_quad(tris, top["fl"], top["fr"], top["br"], top["bl"], color)
    add_quad(tris, bot["fr"], bot["fl"], bot["bl"], bot["br"], color)
    add_quad(tris, top["fl"], top["bl"], bot["bl"], bot["fl"], color)
    add_quad(tris, top["fr"], bot["fr"], bot["br"], top["br"], color)
    add_quad(tris, top["bl"], top["br"], bot["br"], bot["bl"], color)
    add_quad(tris, top["fl"], bot["fl"], bot["fr"], top["fr"], color)


def add_net_panel_y(
    tris: list[Triangle],
    y: float,
    x0: float,
    x1: float,
    z0: float,
    z1: float,
    color: tuple[float, float, float, float],
) -> None:
    radius = 0.006
    xs = np.linspace(x0, x1, 8)
    zs = np.linspace(z0, z1, 7)
    for x in xs:
        add_cylinder(tris, (x, y, z0), (x + 0.035, y, z1), radius, color, segments=8)
    for z in zs:
        add_cylinder(tris, (x0, y, z), (x1, y, z + 0.045), radius, color, segments=8)
    for x in np.linspace(x0 + 0.06, x1 - 0.06, 5):
        add_cylinder(tris, (x, y, z0), (x + 0.18, y, z1), radius * 0.8, color, segments=8)


def add_net_panel_x(
    tris: list[Triangle],
    x: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    color: tuple[float, float, float, float],
) -> None:
    radius = 0.006
    ys = np.linspace(y0, y1, 9)
    zs = np.linspace(z0, z1, 7)
    for y in ys:
        add_cylinder(tris, (x, y, z0), (x, y + 0.035, z1), radius, color, segments=8)
    for z in zs:
        add_cylinder(tris, (x, y0, z), (x, y1, z + 0.035), radius, color, segments=8)
    for y in np.linspace(y0 + 0.06, y1 - 0.06, 6):
        add_cylinder(tris, (x, y, z0), (x, y + 0.16, z1), radius * 0.8, color, segments=8)


def build_mesh() -> list[Triangle]:
    tris: list[Triangle] = []
    white = (0.90, 0.90, 0.86, 1.0)
    panel = (0.26, 0.33, 0.40, 0.26)
    dark = (0.08, 0.10, 0.12, 1.0)

    # Low rectangular PVC basket. +X is the open mouth; the top has a ceiling panel.
    xs = (BACK_X, FRONT_X)
    ys = (-HALF_WIDTH, HALF_WIDTH)
    for y in ys:
        add_cylinder(tris, (BACK_X, y, BOTTOM_Z), (FRONT_X, y, BOTTOM_Z), PIPE_R, white, segments=20)
        add_cylinder(tris, (BACK_X, y, TOP_Z), (FRONT_X, y, TOP_Z), PIPE_R, white, segments=20)
    for x in xs:
        add_cylinder(tris, (x, -HALF_WIDTH, BOTTOM_Z), (x, HALF_WIDTH, BOTTOM_Z), PIPE_R, white, segments=20)
    add_cylinder(tris, (BACK_X, -HALF_WIDTH, TOP_Z), (BACK_X, HALF_WIDTH, TOP_Z), PIPE_R, white, segments=20)
    for x in xs:
        for y in ys:
            add_cylinder(tris, (x, y, BOTTOM_Z), (x, y, TOP_Z), PIPE_R, white, segments=20)

    # Clean internal stiffeners on the basket frame axes.
    add_cylinder(tris, (BACK_X, 0.0, BOTTOM_Z), (FRONT_X, 0.0, BOTTOM_Z), PIPE_R * 0.72, white, segments=16)
    add_cylinder(tris, (BACK_X, -HALF_WIDTH, TOP_Z * 0.5), (BACK_X, HALF_WIDTH, TOP_Z * 0.5), PIPE_R * 0.72, white, segments=16)

    # Net/catch proxy panels are inset inside the PVC frame. There is no front panel.
    inset = PIPE_R + 0.006
    inner_x_half = (FRONT_X - BACK_X) * 0.5 - inset
    inner_y_half = HALF_WIDTH - inset
    inner_z_half = (TOP_Z - BOTTOM_Z) * 0.5 - inset
    center_x = (FRONT_X + BACK_X) * 0.5
    center_z = (TOP_Z + BOTTOM_Z) * 0.5
    side_y = HALF_WIDTH - inset * 0.55
    back_x = BACK_X + inset * 0.55
    bottom_z = BOTTOM_Z + inset * 0.55
    top_z = TOP_Z - inset * 0.55
    add_box(tris, (center_x, -side_y, center_z), (inner_x_half, 0.004, inner_z_half), panel)
    add_box(tris, (center_x, side_y, center_z), (inner_x_half, 0.004, inner_z_half), panel)
    add_box(tris, (back_x, 0.0, center_z), (0.004, inner_y_half, inner_z_half), panel)
    add_box(tris, (center_x, 0.0, bottom_z), (inner_x_half, inner_y_half, 0.006), panel)
    add_box(tris, (center_x, 0.0, top_z), (inner_x_half, inner_y_half, 0.006), panel)

    # Small rear pads only; sized to sit on the existing central frame rails.
    add_box(tris, (BACK_X - 0.018, -0.074, 0.050), (0.018, 0.026, 0.012), white)
    add_box(tris, (BACK_X - 0.018, 0.074, 0.050), (0.018, 0.026, 0.012), white)
    for y in (-0.074, 0.074):
        add_cylinder(tris, (BACK_X - 0.026, y - 0.016, 0.064), (BACK_X - 0.026, y + 0.016, 0.064), 0.003, dark, segments=10)

    return tris


def write_ascii_stl(tris: list[Triangle], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as f:
        f.write(f"solid {MODEL_NAME}\n")
        for tri in tris:
            n = normal_of(tri.vertices)
            f.write(f"  facet normal {n[0]:.8e} {n[1]:.8e} {n[2]:.8e}\n")
            f.write("    outer loop\n")
            for v in tri.vertices:
                f.write(f"      vertex {v[0]:.8e} {v[1]:.8e} {v[2]:.8e}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {MODEL_NAME}\n")


def render_preview(tris: list[Triangle], path: Path, *, elev: float, azim: float, title: str) -> None:
    fig = plt.figure(figsize=(12, 8), dpi=160)
    ax = fig.add_subplot(111, projection="3d")
    polys = [tri.vertices for tri in tris]
    colors = [tri.color for tri in tris]
    collection = Poly3DCollection(polys, facecolors=colors, edgecolors=(0.15, 0.15, 0.15, 0.16), linewidths=0.12)
    ax.add_collection3d(collection)
    all_pts = np.vstack(polys)
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    centers = (mins + maxs) / 2.0
    span = float(np.max(maxs - mins)) * 0.62
    ax.set_xlim(centers[0] - span, centers[0] + span)
    ax.set_ylim(centers[1] - span, centers[1] + span)
    ax.set_zlim(centers[2] - span * 0.55, centers[2] + span * 0.80)
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontsize=14)
    ax.set_xlabel("X front")
    ax.set_ylabel("Y width")
    ax.set_zlabel("Z up")
    ax.set_box_aspect((1.4, 1.0, 1.0))
    ax.grid(False)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_alpha(0.0)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def write_spec(path: Path, tri_count: int) -> None:
    text = """front_open_buoy_collector_v8

Coordinate convention:
- +X = robot front / capture entry direction
- Z = up

Approximate envelope:
- length: 0.38 m including rear mount pads
- width: 0.36 m
- height: 0.28 m

Design choices:
- Front face is open: no net panel at +X.
- Top is closed by an inset ceiling panel inside the PVC frame.
- PVC frame is a clean low basket frame.
- Stiff net is represented only by panels inset inside the frame faces.
- Side, rear, bottom, and top panels catch buoys; the front face remains open.
- STL is collector-module-only, not the full UUV body.

MuJoCo integration note:
- Use the STL as a visual mesh with contype=0, conaffinity=0, density=0.
- Add separate primitive side/rear/bottom/top capture geoms for buoy collision.
- Keep capture geoms out of fluid drag calculation so water can pass through the net.

Triangle count: {tri_count}
""".format(
        tri_count=tri_count
    )
    path.write_text(text, encoding="utf-8")


def main() -> None:
    tris = build_mesh()
    write_ascii_stl(tris, STL_PATH)
    render_preview(
        tris,
        PREVIEW_ISO,
        elev=24,
        azim=-45,
        title="Front-open buoy collector STL v8: taller frame-matched basket",
    )
    render_preview(
        tris,
        PREVIEW_FRONT,
        elev=12,
        azim=0,
        title="Front view (+X): open buoy entry, no front net panel",
    )
    render_preview(
        tris,
        PREVIEW_MOUTH,
        elev=16,
        azim=28,
        title="Front oblique: open front, closed top ceiling",
    )
    write_spec(SPEC_PATH, len(tris))
    print(f"wrote {STL_PATH}")
    print(f"wrote {PREVIEW_ISO}")
    print(f"wrote {PREVIEW_FRONT}")
    print(f"wrote {PREVIEW_MOUTH}")
    print(f"wrote {SPEC_PATH}")
    print(f"triangles={len(tris)}")


if __name__ == "__main__":
    main()
