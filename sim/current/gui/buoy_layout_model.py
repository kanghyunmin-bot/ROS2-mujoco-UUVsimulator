"""Scene XML helpers for editing competition-course XY layout."""

from __future__ import annotations

from dataclasses import dataclass
import io
from pathlib import Path
import xml.etree.ElementTree as ET

from .file_persistence import atomic_write_text, backup_file


TANK_X_HALF_M = 17.5
TANK_Y_HALF_M = 15.0
TANK_X_EDIT_LIMIT_M = 17.2
TANK_Y_EDIT_LIMIT_M = 14.7
ROBOT_SPAWN_BODY_NAME = "base_link"


@dataclass(frozen=True)
class BuoyLayoutItem:
    prefix: str
    label: str
    course: str
    color_name: str
    color_hex: str
    layer: str
    x: float
    y: float
    z: float
    float_body_name: str
    magnet_base_name: str | None = None
    projection_geom_names: tuple[str, ...] = ()

    @property
    def fixed_underwater(self) -> bool:
        return self.magnet_base_name is not None


@dataclass(frozen=True)
class RobotSpawnItem:
    body_name: str
    label: str
    color_hex: str
    layer: str
    x: float
    y: float
    z: float


def floats(value: str | None) -> tuple[float, ...]:
    if not value:
        return ()
    return tuple(float(part) for part in value.split())


def pos_xyz(element: ET.Element) -> tuple[float, float, float]:
    values = floats(element.get("pos"))
    if len(values) != 3:
        raise ValueError(f"{element.get('name')} has invalid pos={element.get('pos')!r}")
    return values


def set_xy_preserve_z(element: ET.Element, x: float, y: float) -> None:
    _, _, z = pos_xyz(element)
    element.set("pos", f"{x:.3f} {y:.3f} {z:.3f}")


def set_xy_fixed_z(element: ET.Element, x: float, y: float, z: float) -> None:
    element.set("pos", f"{x:.3f} {y:.3f} {z:.3f}")


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


def classify_prefix(prefix: str) -> tuple[str, str, str, str]:
    parts = prefix.split("_")
    course = parts[2].upper() if len(parts) > 3 and parts[2] in {"a", "b"} else "P"
    if "_red_" in prefix:
        return course, "red", "#ef1d1d", "surface"
    if "_yellow_" in prefix:
        return course, "yellow", "#f5dd17", "moored"
    if "_orange_" in prefix:
        return course, "orange", "#f59e0b", "moored"
    if "_pinger_white_" in prefix:
        return "P", "white", "#f8fafc", "moored"
    return course, "unknown", "#64748b", "moored"


def buoy_label(prefix: str) -> str:
    parts = prefix.split("_")
    if "_pinger_white_" in prefix:
        return "P W1"
    if len(parts) >= 5:
        course = parts[2].upper()
        color_code = parts[3][0].upper()
        return f"{course} {color_code}{parts[4]}"
    return prefix.removeprefix("course_buoy_")


def buoy_sort_key(item: BuoyLayoutItem) -> tuple[int, int, int, str]:
    course_rank = {"A": 0, "B": 1, "P": 2}.get(item.course, 3)
    color_rank = {"red": 0, "yellow": 1, "orange": 2, "white": 3}.get(item.color_name, 4)
    suffix = item.prefix.rsplit("_", 1)[-1]
    try:
        index = int(suffix)
    except ValueError:
        index = 0
    return course_rank, color_rank, index, item.prefix


def load_buoy_layout(scene_path: Path) -> list[BuoyLayoutItem]:
    root = ET.parse(scene_path).getroot()
    bodies = body_map(root)
    geoms = geom_map(root)
    items: list[BuoyLayoutItem] = []

    for body_name, body in bodies.items():
        if not body_name.startswith("course_buoy_") or not body_name.endswith("_float"):
            continue
        prefix = body_name.removesuffix("_float")
        if "_red_" not in prefix:
            continue
        x, y, z = pos_xyz(body)
        course, color_name, color_hex, layer = classify_prefix(prefix)
        items.append(
            BuoyLayoutItem(
                prefix=prefix,
                label=buoy_label(prefix),
                course=course,
                color_name=color_name,
                color_hex=color_hex,
                layer=layer,
                x=x,
                y=y,
                z=z,
                float_body_name=body_name,
            )
        )

    for base_name, base in bodies.items():
        if not base_name.startswith("course_buoy_") or not base_name.endswith("_magnet_base"):
            continue
        prefix = base_name.removesuffix("_magnet_base")
        float_name = f"{prefix}_float"
        float_body = bodies.get(float_name)
        if float_body is None:
            continue
        x, y, z = pos_xyz(float_body)
        base_x, base_y, _ = pos_xyz(base)
        course, color_name, color_hex, layer = classify_prefix(prefix)
        projection_names = tuple(
            name
            for name in (
                f"{prefix}_surface_projection",
                f"{prefix}_surface_projection_outline",
            )
            if name in geoms
        )
        items.append(
            BuoyLayoutItem(
                prefix=prefix,
                label=buoy_label(prefix),
                course=course,
                color_name=color_name,
                color_hex=color_hex,
                layer=layer,
                x=base_x,
                y=base_y,
                z=z,
                float_body_name=float_name,
                magnet_base_name=base_name,
                projection_geom_names=projection_names,
            )
        )

    return sorted(items, key=buoy_sort_key)


def load_robot_spawn(scene_path: Path) -> RobotSpawnItem:
    root = ET.parse(scene_path).getroot()
    bodies = body_map(root)
    body = bodies.get(ROBOT_SPAWN_BODY_NAME)
    if body is None:
        raise ValueError(f"{ROBOT_SPAWN_BODY_NAME} body not found in {scene_path}")
    x, y, z = pos_xyz(body)
    return RobotSpawnItem(
        body_name=ROBOT_SPAWN_BODY_NAME,
        label="ROBOT",
        color_hex="#2563eb",
        layer="surface spawn",
        x=x,
        y=y,
        z=z,
    )


def clamp_xy(x: float, y: float) -> tuple[float, float]:
    return (
        max(-TANK_X_EDIT_LIMIT_M, min(TANK_X_EDIT_LIMIT_M, x)),
        max(-TANK_Y_EDIT_LIMIT_M, min(TANK_Y_EDIT_LIMIT_M, y)),
    )


def save_buoy_layout(
    scene_path: Path,
    positions: dict[str, tuple[float, float]],
    *,
    robot_xy: tuple[float, float] | None = None,
) -> Path:
    tree = ET.parse(scene_path)
    root = tree.getroot()
    bodies = body_map(root)
    geoms = geom_map(root)
    current_items = {item.prefix: item for item in load_buoy_layout(scene_path)}
    robot_spawn = load_robot_spawn(scene_path)

    for prefix, (raw_x, raw_y) in positions.items():
        item = current_items.get(prefix)
        if item is None:
            continue
        x, y = clamp_xy(raw_x, raw_y)
        float_body = bodies.get(item.float_body_name)
        if float_body is not None:
            set_xy_preserve_z(float_body, x, y)
        if item.magnet_base_name:
            base = bodies.get(item.magnet_base_name)
            if base is not None:
                set_xy_preserve_z(base, x, y)
            for geom_name in item.projection_geom_names:
                projection = geoms.get(geom_name)
                if projection is not None:
                    set_xy_preserve_z(projection, x, y)

    if robot_xy is not None:
        robot_body = bodies.get(ROBOT_SPAWN_BODY_NAME)
        if robot_body is None:
            raise ValueError(f"{ROBOT_SPAWN_BODY_NAME} body not found in {scene_path}")
        x, y = clamp_xy(robot_xy[0], robot_xy[1])
        set_xy_fixed_z(robot_body, x, y, robot_spawn.z)

    backup = backup_file(scene_path, category="course")
    output = io.StringIO()
    tree.write(output, encoding="unicode")
    atomic_write_text(scene_path, output.getvalue())
    return backup


__all__ = [
    "BuoyLayoutItem",
    "RobotSpawnItem",
    "ROBOT_SPAWN_BODY_NAME",
    "TANK_X_HALF_M",
    "TANK_Y_HALF_M",
    "clamp_xy",
    "load_buoy_layout",
    "load_robot_spawn",
    "save_buoy_layout",
]
