#!/usr/bin/env python3
"""Smoke-check GUI buoy XY layout XML editing helpers."""

from __future__ import annotations

import math
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.buoy_layout_model import (  # noqa: E402
    TANK_X_EDIT_LIMIT_M,
    TANK_Y_EDIT_LIMIT_M,
    load_buoy_layout,
    load_robot_spawn,
    save_buoy_layout,
)


SCENE = ROOT / "scenes" / "tank_current_scene.xml"


def floats(value: str | None) -> tuple[float, ...]:
    if not value:
        return ()
    return tuple(float(part) for part in value.split())


def body_map(root: ET.Element) -> dict[str, ET.Element]:
    return {
        str(body.get("name")): body
        for body in root.findall(".//body")
        if body.get("name")
    }


def geom_map(root: ET.Element) -> dict[str, ET.Element]:
    return {
        str(geom.get("name")): geom
        for geom in root.findall(".//geom")
        if geom.get("name")
    }


def pos(element: ET.Element) -> tuple[float, float, float]:
    values = floats(element.get("pos"))
    if len(values) != 3:
        raise AssertionError(f"invalid pos for {element.get('name')}: {element.get('pos')!r}")
    return values


def assert_xy_z(element: ET.Element, x: float, y: float, z: float, label: str) -> None:
    actual = pos(element)
    if not math.isclose(actual[0], x, abs_tol=1e-6) or not math.isclose(actual[1], y, abs_tol=1e-6):
        raise AssertionError(f"{label} xy={actual[:2]} expected {(x, y)}")
    if not math.isclose(actual[2], z, abs_tol=1e-6):
        raise AssertionError(f"{label} z changed: {actual[2]} expected {z}")


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="uuv_buoy_layout_") as tmp:
        temp_scene = Path(tmp) / "tank_current_scene.xml"
        shutil.copy2(SCENE, temp_scene)
        items = load_buoy_layout(temp_scene)
        red_items = [item for item in items if item.color_name == "red"]
        fixed_items = [item for item in items if item.fixed_underwater]
        if len(red_items) != 10:
            raise AssertionError(f"expected 10 red surface buoys, got {len(red_items)}")
        if len(fixed_items) != 15:
            raise AssertionError(f"expected 15 fixed underwater buoys, got {len(fixed_items)}")
        robot_spawn = load_robot_spawn(temp_scene)

        red = red_items[0]
        fixed = next(item for item in fixed_items if item.projection_geom_names)
        before_root = ET.parse(temp_scene).getroot()
        before_geoms = geom_map(before_root)
        projection_z = {
            geom_name: pos(before_geoms[geom_name])[2]
            for geom_name in fixed.projection_geom_names
        }
        positions = {item.prefix: (item.x, item.y) for item in items}
        red_target = (red.x + 0.4, red.y - 0.3)
        fixed_target = (-99.0, 99.0)
        positions[red.prefix] = red_target
        positions[fixed.prefix] = fixed_target
        robot_target = (robot_spawn.x + 0.8, robot_spawn.y - 0.6)
        backup = save_buoy_layout(temp_scene, positions, robot_xy=robot_target)
        if not backup.exists():
            raise AssertionError("save did not create a scene backup")

        root = ET.parse(temp_scene).getroot()
        bodies = body_map(root)
        geoms = geom_map(root)
        assert_xy_z(bodies[red.float_body_name], red_target[0], red_target[1], red.z, red.float_body_name)
        fixed_x = -TANK_X_EDIT_LIMIT_M
        fixed_y = TANK_Y_EDIT_LIMIT_M
        base = bodies[fixed.magnet_base_name or ""]
        float_body = bodies[fixed.float_body_name]
        assert_xy_z(base, fixed_x, fixed_y, -11.0, fixed.magnet_base_name or "")
        assert_xy_z(float_body, fixed_x, fixed_y, fixed.z, fixed.float_body_name)
        for geom_name in fixed.projection_geom_names:
            projection = geoms[geom_name]
            assert_xy_z(projection, fixed_x, fixed_y, projection_z[geom_name], geom_name)
        robot_body = bodies[robot_spawn.body_name]
        assert_xy_z(robot_body, robot_target[0], robot_target[1], robot_spawn.z, robot_spawn.body_name)

    print("gui_course_layout_editor=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
