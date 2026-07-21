#!/usr/bin/env python3
"""Render the front-open buoy collector mounted on top of the current UUV scene.

This writes a temporary XML only. The collector is represented as MuJoCo
primitive geoms so the PVC frame and net proxy panels remain visually distinct.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
SCENE = ROOT / "uuv_mujoco" / "v2.2" / "scenes" / "tank_current_scene.xml"
OUT_DIR = ROOT / "generated_meshes" / "previews"
TMP_SCENE = OUT_DIR / "tank_current_scene_with_front_open_collector_v8_preview.xml"
ISO_PNG = OUT_DIR / "front_open_buoy_collector_v8_mounted_iso.png"
FRONT_PNG = OUT_DIR / "front_open_buoy_collector_v8_mounted_front.png"
SIDE_PNG = OUT_DIR / "front_open_buoy_collector_v8_mounted_side.png"
TOP_PNG = OUT_DIR / "front_open_buoy_collector_v8_mounted_top.png"


MOUNT_POS = np.array([-0.011, 0.0, 0.160])
FRONT_X = 0.17
BACK_X = -0.17
HALF_WIDTH = 0.172
BOTTOM_Z = 0.00
TOP_Z = 0.307
PIPE_R = 0.010


def vec_text(values: tuple[float, ...] | list[float] | np.ndarray) -> str:
    return " ".join(f"{float(value):.6f}" for value in values)


def rgba_text(values: tuple[float, float, float, float]) -> str:
    return " ".join(f"{value:.3f}" for value in values)


def add_geom(parent: ET.Element, **attrs: str) -> ET.Element:
    return ET.SubElement(parent, "geom", attrs)


def add_capsule(
    parent: ET.Element,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    rgba: tuple[float, float, float, float],
) -> None:
    add_geom(
        parent,
        name=name,
        type="capsule",
        fromto=f"{vec_text(p0)} {vec_text(p1)}",
        size=f"{radius:.6f}",
        rgba=rgba_text(rgba),
        contype="0",
        conaffinity="0",
        density="0",
        group="0",
    )


def add_box(
    parent: ET.Element,
    name: str,
    pos: tuple[float, float, float],
    size: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
) -> None:
    add_geom(
        parent,
        name=name,
        type="box",
        pos=vec_text(pos),
        size=vec_text(size),
        rgba=rgba_text(rgba),
        contype="0",
        conaffinity="0",
        density="0",
        group="0",
    )


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.findall(".//body"):
        if body.get("name") == name:
            return body
    raise RuntimeError(f"missing body {name}")


def remove_existing_collectors(root: ET.Element) -> None:
    for parent in root.findall(".//body"):
        for child in list(parent):
            if child.tag == "body" and child.get("name") in {
                "front_open_buoy_collector",
                "front_open_buoy_collector_preview",
            }:
                parent.remove(child)


def add_collector(root: ET.Element) -> None:
    base_link = find_body(root, "base_link")
    collector = ET.SubElement(base_link, "body", {"name": "front_open_buoy_collector_preview", "pos": vec_text(MOUNT_POS)})

    pipe = (0.92, 0.92, 0.86, 1.0)
    panel = (0.25, 0.34, 0.42, 0.42)
    mount = (0.86, 0.86, 0.80, 1.0)
    marker = (0.00, 0.90, 1.00, 0.85)

    xs = (BACK_X, FRONT_X)
    ys = (-HALF_WIDTH, HALF_WIDTH)
    for y in ys:
        add_capsule(collector, f"collector_bottom_rail_y_{y:+.2f}", (BACK_X, y, BOTTOM_Z), (FRONT_X, y, BOTTOM_Z), PIPE_R, pipe)
        add_capsule(collector, f"collector_top_rail_y_{y:+.2f}", (BACK_X, y, TOP_Z), (FRONT_X, y, TOP_Z), PIPE_R, pipe)
    for x in xs:
        add_capsule(collector, f"collector_bottom_cross_x_{x:+.2f}", (x, -HALF_WIDTH, BOTTOM_Z), (x, HALF_WIDTH, BOTTOM_Z), PIPE_R, pipe)
    add_capsule(collector, "collector_back_top_cross", (BACK_X, -HALF_WIDTH, TOP_Z), (BACK_X, HALF_WIDTH, TOP_Z), PIPE_R, pipe)
    for x in xs:
        for y in ys:
            add_capsule(collector, f"collector_post_x{x:+.2f}_y{y:+.2f}", (x, y, BOTTOM_Z), (x, y, TOP_Z), PIPE_R, pipe)

    add_capsule(collector, "collector_bottom_center_rail", (BACK_X, 0.0, BOTTOM_Z), (FRONT_X, 0.0, BOTTOM_Z), PIPE_R * 0.72, pipe)
    add_capsule(
        collector,
        "collector_back_mid_cross",
        (BACK_X, -HALF_WIDTH, TOP_Z * 0.5),
        (BACK_X, HALF_WIDTH, TOP_Z * 0.5),
        PIPE_R * 0.72,
        pipe,
    )

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
    add_box(collector, "collector_left_net_proxy", (center_x, -side_y, center_z), (inner_x_half, 0.004, inner_z_half), panel)
    add_box(collector, "collector_right_net_proxy", (center_x, side_y, center_z), (inner_x_half, 0.004, inner_z_half), panel)
    add_box(collector, "collector_back_net_proxy", (back_x, 0.0, center_z), (0.004, inner_y_half, inner_z_half), panel)
    add_box(collector, "collector_bottom_net_proxy", (center_x, 0.0, bottom_z), (inner_x_half, inner_y_half, 0.006), panel)
    add_box(collector, "collector_top_net_proxy", (center_x, 0.0, top_z), (inner_x_half, inner_y_half, 0.006), panel)

    add_box(collector, "collector_rear_mount_left", (BACK_X - 0.018, -0.074, 0.050), (0.018, 0.026, 0.012), mount)
    add_box(collector, "collector_rear_mount_right", (BACK_X - 0.018, 0.074, 0.050), (0.018, 0.026, 0.012), mount)

    # Tiny blue lip marks the open intake side in the preview only.
    add_capsule(collector, "collector_open_mouth_marker", (FRONT_X, -HALF_WIDTH, TOP_Z + 0.013), (FRONT_X, HALF_WIDTH, TOP_Z + 0.013), 0.005, marker)


def write_temp_scene() -> None:
    root = ET.parse(SCENE).getroot()
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", str(ROOT / "uuv_mujoco" / "v2.2" / "assets" / "urdf_full" / "meshes_split"))
    remove_existing_collectors(root)
    add_collector(root)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(TMP_SCENE, encoding="utf-8", xml_declaration=False)


def render(path: Path, *, azimuth: float, elevation: float, distance: float, lookat: tuple[float, float, float]) -> None:
    model = mujoco.MjModel.from_xml_path(str(TMP_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = azimuth
    camera.elevation = elevation
    camera.distance = distance
    camera.lookat[:] = np.array(lookat, dtype=float)

    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        plt.imsave(path, renderer.render())


def main() -> None:
    write_temp_scene()
    # Base link starts at z=-0.45. The compact collector sits just above the hull.
    # MuJoCo free-camera lookat is in world coordinates; base_link starts at (-2, 0.9, -0.45).
    lookat = (-2.01, 0.90, -0.20)
    render(ISO_PNG, azimuth=130, elevation=-18, distance=0.72, lookat=lookat)
    render(FRONT_PNG, azimuth=180, elevation=-8, distance=0.62, lookat=lookat)
    render(SIDE_PNG, azimuth=90, elevation=-10, distance=0.62, lookat=lookat)
    render(TOP_PNG, azimuth=90, elevation=-88, distance=0.62, lookat=lookat)
    print(f"wrote {TMP_SCENE}")
    print(f"wrote {ISO_PNG}")
    print(f"wrote {FRONT_PNG}")
    print(f"wrote {SIDE_PNG}")
    print(f"wrote {TOP_PNG}")


if __name__ == "__main__":
    main()
