#!/usr/bin/env python3
"""Generate the visible PVC collector frame and convex side collision proxy.

The model is intentionally simple and simulation-friendly:
- clean low PVC basket frame
- collision proxies remain separate so the visible STL has no solid panels
- front face stays open so buoys can enter the capture pocket
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = ROOT / "generated_meshes"
ASSET_DIR = ROOT / "sim" / "current" / "assets" / "urdf_full" / "meshes_split"
PREVIEW_DIR = OUT_DIR / "previews"
MODEL_NAME = "front_open_buoy_collector_v9"
STL_PATH = ASSET_DIR / f"{MODEL_NAME}.stl"
SIDE_PROXY_NAME = "front_open_buoy_collector_side_proxy_v1"
SIDE_PROXY_PATH = ASSET_DIR / f"{SIDE_PROXY_NAME}.stl"
PREVIEW_ISO = PREVIEW_DIR / f"{MODEL_NAME}_iso.png"
PREVIEW_FRONT = PREVIEW_DIR / f"{MODEL_NAME}_front.png"
PREVIEW_MOUTH = PREVIEW_DIR / f"{MODEL_NAME}_mouth.png"
SPEC_PATH = OUT_DIR / f"{MODEL_NAME}_spec.txt"


FRONT_X = 0.315
BACK_X = -0.315
HALF_WIDTH = 0.315
BOTTOM_Z = 0.00
BACK_TOP_Z = 0.390
FRONT_TOP_Z = 0.450
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

    # +X is the open mouth. The higher front and lower rear make the side
    # profile trapezoidal while preserving the existing 630mm footprint.
    ys = (-HALF_WIDTH, HALF_WIDTH)
    for y in ys:
        add_cylinder(tris, (BACK_X, y, BOTTOM_Z), (FRONT_X, y, BOTTOM_Z), PIPE_R, white, segments=20)
        add_cylinder(
            tris,
            (BACK_X, y, BACK_TOP_Z),
            (FRONT_X, y, FRONT_TOP_Z),
            PIPE_R,
            white,
            segments=20,
        )
    for x, top_z in ((BACK_X, BACK_TOP_Z), (FRONT_X, FRONT_TOP_Z)):
        add_cylinder(tris, (x, -HALF_WIDTH, BOTTOM_Z), (x, HALF_WIDTH, BOTTOM_Z), PIPE_R, white, segments=20)
        add_cylinder(tris, (x, -HALF_WIDTH, top_z), (x, HALF_WIDTH, top_z), PIPE_R, white, segments=20)
        for y in ys:
            add_cylinder(tris, (x, y, BOTTOM_Z), (x, y, top_z), PIPE_R, white, segments=20)

    # Internal stiffeners follow the existing simulation frame.
    add_cylinder(tris, (BACK_X, 0.0, BOTTOM_Z), (FRONT_X, 0.0, BOTTOM_Z), PIPE_R * 0.72, white, segments=16)
    add_cylinder(
        tris,
        (BACK_X, -HALF_WIDTH, BACK_TOP_Z * 0.5),
        (BACK_X, HALF_WIDTH, BACK_TOP_Z * 0.5),
        PIPE_R * 0.72,
        white,
        segments=16,
    )

    return tris


def build_side_proxy_mesh() -> list[Triangle]:
    """Build one closed convex side panel matching the inset trapezoid."""
    tris: list[Triangle] = []
    color = (0.20, 0.30, 0.38, 1.0)
    x0, x1 = BACK_X + 0.016, FRONT_X - 0.016
    y0, y1 = -0.015, 0.015
    z0 = BOTTOM_Z + 0.015
    z_back = BACK_TOP_Z - 0.015
    z_front = FRONT_TOP_Z - 0.015
    p = {
        "b0": np.array([x0, y0, z0]),
        "b1": np.array([x0, y1, z0]),
        "bt0": np.array([x0, y0, z_back]),
        "bt1": np.array([x0, y1, z_back]),
        "f0": np.array([x1, y0, z0]),
        "f1": np.array([x1, y1, z0]),
        "ft0": np.array([x1, y0, z_front]),
        "ft1": np.array([x1, y1, z_front]),
    }
    add_quad(tris, p["b0"], p["f0"], p["ft0"], p["bt0"], color)
    add_quad(tris, p["f1"], p["b1"], p["bt1"], p["ft1"], color)
    add_quad(tris, p["b1"], p["b0"], p["bt0"], p["bt1"], color)
    add_quad(tris, p["f0"], p["f1"], p["ft1"], p["ft0"], color)
    add_quad(tris, p["bt0"], p["ft0"], p["ft1"], p["bt1"], color)
    add_quad(tris, p["b1"], p["f1"], p["f0"], p["b0"], color)
    return tris


def write_binary_stl(tris: list[Triangle], path: Path, solid_name: str = MODEL_NAME) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = f"{solid_name} generated collision-safe visual".encode("ascii")[:80].ljust(80, b"\0")
    with path.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(tris)))
        for tri in tris:
            n = normal_of(tri.vertices)
            payload = [float(n[0]), float(n[1]), float(n[2])]
            for v in tri.vertices:
                payload.extend((float(v[0]), float(v[1]), float(v[2])))
            f.write(struct.pack("<12fH", *payload, 0))


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
    text = """front_open_buoy_collector_v9

Coordinate convention:
- +X = robot front / capture entry direction
- Z = up

Approximate envelope:
- length: 0.63 m
- width: 0.63 m
- rear height: 0.39 m
- front opening height: 0.45 m

Design choices:
- Front face is open: no net panel at +X.
- The side profile is trapezoidal: low rear, full-height front opening.
- STL contains only the white PVC frame; net panels and mounts stay hidden in MuJoCo.
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
    write_binary_stl(tris, STL_PATH)
    side_proxy_tris = build_side_proxy_mesh()
    write_binary_stl(side_proxy_tris, SIDE_PROXY_PATH, SIDE_PROXY_NAME)
    render_preview(
        tris,
        PREVIEW_ISO,
        elev=24,
        azim=-45,
        title="Front-open buoy collector STL v9: trapezoidal PVC frame",
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
    print(f"wrote {SIDE_PROXY_PATH}")
    print(f"wrote {PREVIEW_ISO}")
    print(f"wrote {PREVIEW_FRONT}")
    print(f"wrote {PREVIEW_MOUTH}")
    print(f"wrote {SPEC_PATH}")
    print(f"triangles={len(tris)}")


if __name__ == "__main__":
    main()
