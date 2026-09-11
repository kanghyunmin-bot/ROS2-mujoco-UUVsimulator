"""Edit research-pool robot and moored buoy positions in the scene itself."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .buoy_layout_model import load_buoy_layout, load_robot_spawn
from .file_persistence import atomic_write_text, backup_file
from .research_pool_rope import build_mooring_rope

MODE = "research_pool"


def load_layout(path: Path) -> dict:
    root = ET.parse(path).getroot()
    floor = root.find('.//geom[@name="pool_floor"]')
    half_x, half_y, half_z = map(float, floor.get("size").split())
    depth = -float(floor.get("pos").split()[2]) - half_z
    robot = load_robot_spawn(path)
    return {
        "path": str(path),
        "mode": MODE,
        "active_mode": MODE,
        "status": "Research pool · drag items or edit position/depth; save and restart to apply",
        "tank": {
            "x_half_m": half_x,
            "y_half_m": half_y,
            "length_m": 2 * half_x,
            "width_m": 2 * half_y,
            "depth_m": depth,
        },
        "robot_target_id": "base_link",
        "robot": {
            "id": "base_link",
            "label": "ROV",
            "color_hex": "#2563eb",
            "layer": "수중 로봇",
            "x": robot.x,
            "y": robot.y,
            "z": robot.z,
        },
        "items": [
            {
                "id": item.prefix,
                "label": f"부표 {index + 1}",
                "color_hex": item.color_hex,
                "color_name": item.color_name,
                "layer": "수중 · 바닥 고정",
                "x": item.x,
                "y": item.y,
                "z": item.z,
                "fixed_underwater": True,
            }
            for index, item in enumerate(load_buoy_layout(path))
        ],
    }


def save_layout(path: Path, positions: dict, robot_position=None) -> Path:
    """Validate all positions [m] before atomically saving geometry and welds."""
    layout = load_layout(path)
    tank = layout["tank"]
    tree = ET.parse(path)
    root = tree.getroot()
    items = {item["id"]: item for item in layout["items"]}
    if set(positions) - set(items):
        raise ValueError("Unknown research-pool buoy")

    def xyz(value, current, robot=False):
        if not isinstance(value, dict):
            raise ValueError("Position must contain x, y and optional z")
        values = [float(value.get(axis, current[axis])) for axis in ("x", "y", "z")]
        x, y, z = values
        margin = 0.45 if robot else 0.25
        floor_margin = 0.45 if robot else 0.7
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Position must be finite")
        if (
            abs(x) > tank["x_half_m"] - margin
            or abs(y) > tank["y_half_m"] - margin
            or not -tank["depth_m"] + floor_margin <= z <= -0.4
        ):
            raise ValueError(
                "Position must be underwater and clear of the pool floor/walls"
            )
        return values

    cleaned = {name: xyz(value, items[name]) for name, value in positions.items()}
    robot_xyz = (
        None if robot_position is None else xyz(robot_position, layout["robot"], True)
    )
    for prefix, (x, y, z) in cleaned.items():
        base = root.find(f'.//body[@name="{prefix}_magnet_base"]')
        floating = root.find(f'.//body[@name="{prefix}_float"]')
        delta = z - float(floating.get("pos").split()[2])
        base.set("pos", f"{x:.4f} {y:.4f} {-tank['depth_m']:.4f}")
        floating.set("pos", f"{x:.4f} {y:.4f} {z:.4f}")
        for element in base:
            if element.get("name", "").endswith("_mooring_rod"):
                values = list(map(float, element.get("fromto").split()))
                values[5] += delta
                element.set("fromto", " ".join(map(str, values)))
            elif element.get("pos"):
                values = list(map(float, element.get("pos").split()))
                if values[2] > 0.2:
                    values[2] += delta
                    element.set("pos", " ".join(map(str, values)))
        weld = root.find(f'.//weld[@name="{prefix}_magnet_weld"]')
        if base.find(f'body[@name="{prefix}_rope_00"]') is not None:
            build_mooring_rope(root, prefix, z + tank["depth_m"] - 0.215 - 0.109)
        else:
            weld.set("relpose", f"0 0 {z + tank['depth_m']:.4f} 1 0 0 0")
    if robot_xyz is not None:
        root.find('.//body[@name="base_link"]').set(
            "pos", " ".join(f"{v:.4f}" for v in robot_xyz)
        )
    backup = backup_file(path, category="course")
    ET.indent(root, space="  ")
    atomic_write_text(path, ET.tostring(root, encoding="unicode") + "\n")
    return backup
