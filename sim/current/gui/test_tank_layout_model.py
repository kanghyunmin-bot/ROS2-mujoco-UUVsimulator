"""Persist GUI test-tank layout and generate an isolated MuJoCo scene."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

from .file_persistence import atomic_write_text, backup_file


COURSE_MODE_COMPETITION = "competition"
COURSE_MODE_TEST_TANK = "test_tank"
COURSE_MODES = (COURSE_MODE_COMPETITION, COURSE_MODE_TEST_TANK)

TEST_TANK_LENGTH_M = 5.49
TEST_TANK_WIDTH_M = 2.74
TEST_TANK_DEPTH_M = 1.32
TEST_TANK_X_HALF_M = TEST_TANK_LENGTH_M / 2.0
TEST_TANK_Y_HALF_M = TEST_TANK_WIDTH_M / 2.0

TEST_TANK_YELLOW_ID = "test_tank_yellow_marker"
TEST_TANK_YELLOW_BUOY_PREFIX = "course_buoy_test_tank_yellow_1"
TEST_TANK_PINGER_ID = "test_tank_standalone_pinger"
TEST_TANK_PINGER_SITE_NAME = "test_tank_pinger_acoustic_site"
# This is the initial CoB height of the physical yellow float.
TEST_TANK_YELLOW_Z_M = -0.56
TEST_TANK_PINGER_Z_M = -1.08
TEST_TANK_NYLON_LENGTH_M = 0.05
# Make the lollipop-style PVC stem easier to identify in the camera without
# changing the float mass, CoM, CoB, or the 50 mm detachable nylon contract.
TEST_TANK_STICK_EXTENSION_M = 0.025
# Synthetic PCM amplitude intentionally saturates inside 1 m, so the acoustic
# range observable cannot distinguish 0.55 m from 1.0 m in test-tank mode.
TEST_TANK_HOMING_SUCCESS_RANGE_M = 1.05

_CONFIG_VERSION = 1
_WALL_HALF_THICKNESS_M = 0.05
_WALL_TOP_Z_M = 0.35
_POSITION_MARGIN_M = 0.10
_TILE_PANEL_SIZE_M = TEST_TANK_WIDTH_M / 2.0


@dataclass(frozen=True)
class CourseRuntimeSelection:
    mode: str
    scene_path: Path
    pinger_site_name: str


def default_course_layout_config() -> dict[str, Any]:
    return {
        "version": _CONFIG_VERSION,
        "active_mode": COURSE_MODE_TEST_TANK,
        "test_tank": {
            "dimensions_m": {
                "length": TEST_TANK_LENGTH_M,
                "width": TEST_TANK_WIDTH_M,
                "depth": TEST_TANK_DEPTH_M,
            },
            "robot_xy": {"x": -1.80, "y": 0.0},
            "yellow_buoy_xy": {"x": 0.0, "y": 0.45},
            "pinger_xy": {"x": 1.80, "y": -0.45},
        },
    }


def normalize_course_mode(value: object, *, default: str = COURSE_MODE_TEST_TANK) -> str:
    mode = str(value or "").strip().lower()
    return mode if mode in COURSE_MODES else default


def load_course_layout_config(path: Path) -> dict[str, Any]:
    default = default_course_layout_config()
    if not path.is_file():
        return default
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default
    if not isinstance(raw, dict):
        return default

    test_raw = raw.get("test_tank") if isinstance(raw.get("test_tank"), dict) else {}
    merged = default
    merged["active_mode"] = normalize_course_mode(raw.get("active_mode"))
    merged_test = merged["test_tank"]
    for key in ("robot_xy", "yellow_buoy_xy", "pinger_xy"):
        value = test_raw.get(key)
        if isinstance(value, dict):
            merged_test[key] = _xy_dict(value, fallback=merged_test[key])
    return merged


def save_course_layout_config(
    path: Path,
    *,
    mode: str,
    robot_xy: tuple[float, float] | None = None,
    yellow_xy: tuple[float, float] | None = None,
    pinger_xy: tuple[float, float] | None = None,
) -> Path | None:
    config = load_course_layout_config(path)
    config["active_mode"] = normalize_course_mode(mode)
    test = config["test_tank"]
    if robot_xy is not None:
        test["robot_xy"] = _clamped_xy_dict(*robot_xy)
    if yellow_xy is not None:
        test["yellow_buoy_xy"] = _clamped_xy_dict(*yellow_xy)
    if pinger_xy is not None:
        test["pinger_xy"] = _clamped_xy_dict(*pinger_xy)

    path.parent.mkdir(parents=True, exist_ok=True)
    backup = backup_file(path, category="course") if path.exists() else None
    atomic_write_text(path, json.dumps(config, indent=2, ensure_ascii=True) + "\n")
    return backup


def test_tank_positions(config: dict[str, Any]) -> dict[str, tuple[float, float]]:
    test = config.get("test_tank") if isinstance(config.get("test_tank"), dict) else {}
    defaults = default_course_layout_config()["test_tank"]
    return {
        "robot": _xy_tuple(test.get("robot_xy"), fallback=defaults["robot_xy"]),
        "yellow": _xy_tuple(test.get("yellow_buoy_xy"), fallback=defaults["yellow_buoy_xy"]),
        "pinger": _xy_tuple(test.get("pinger_xy"), fallback=defaults["pinger_xy"]),
    }


def prepare_active_course_runtime(
    *,
    config_path: Path,
    competition_scene_path: Path,
    test_tank_scene_path: Path,
) -> CourseRuntimeSelection:
    config = load_course_layout_config(config_path)
    mode = normalize_course_mode(config.get("active_mode"))
    if mode == COURSE_MODE_COMPETITION:
        return CourseRuntimeSelection(
            mode=mode,
            scene_path=competition_scene_path,
            pinger_site_name="course_buoy_pinger_white_1_acoustic_site",
        )
    generate_test_tank_scene(
        base_scene_path=competition_scene_path,
        output_scene_path=test_tank_scene_path,
        config=config,
    )
    return CourseRuntimeSelection(
        mode=mode,
        scene_path=test_tank_scene_path,
        pinger_site_name=TEST_TANK_PINGER_SITE_NAME,
    )


def generate_test_tank_scene(
    *,
    base_scene_path: Path,
    output_scene_path: Path,
    config: dict[str, Any],
) -> Path:
    tree = ET.parse(base_scene_path)
    root = tree.getroot()
    root.set("model", "uuv_test_tank_549x274x132")
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"worldbody missing in {base_scene_path}")

    source_prefix = "course_buoy_a_yellow_1"
    source_magnet_base = _named_child(root, "body", f"{source_prefix}_magnet_base")
    source_float = _named_child(root, "body", f"{source_prefix}_float")
    if source_magnet_base is None or source_float is None:
        raise ValueError(f"physical yellow buoy template missing in {base_scene_path}")
    magnet_base_template = copy.deepcopy(source_magnet_base)
    float_template = copy.deepcopy(source_float)

    _remove_competition_layout(worldbody, root)
    _resize_pool(worldbody)
    _append_test_tank_tile_wall_panels(worldbody)
    positions = test_tank_positions(config)
    robot = _named_child(root, "body", "base_link")
    if robot is None:
        raise ValueError(f"base_link body missing in {base_scene_path}")
    _set_xy_preserve_z(robot, *positions["robot"])
    _append_test_tank_markers(
        worldbody,
        root,
        positions,
        magnet_base_template=magnet_base_template,
        float_template=float_template,
        source_prefix=source_prefix,
    )

    output_scene_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=output_scene_path.parent,
        prefix=f".{output_scene_path.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        temp_path = Path(handle.name)
        tree.write(handle, encoding="unicode", xml_declaration=False)
    temp_path.replace(output_scene_path)
    return output_scene_path


def test_tank_payload(config: dict[str, Any]) -> dict[str, Any]:
    positions = test_tank_positions(config)
    robot_x, robot_y = positions["robot"]
    yellow_x, yellow_y = positions["yellow"]
    pinger_x, pinger_y = positions["pinger"]
    return {
        "tank": {
            "x_half_m": TEST_TANK_X_HALF_M,
            "y_half_m": TEST_TANK_Y_HALF_M,
            "depth_m": TEST_TANK_DEPTH_M,
            "length_m": TEST_TANK_LENGTH_M,
            "width_m": TEST_TANK_WIDTH_M,
        },
        "robot": {
            "id": "base_link",
            "label": "ROBOT",
            "color_hex": "#2563eb",
            "layer": "surface spawn",
            "x": robot_x,
            "y": robot_y,
            "z": -0.45,
        },
        "items": [
            {
                "id": TEST_TANK_YELLOW_ID,
                "prefix": TEST_TANK_YELLOW_BUOY_PREFIX,
                "label": "Yellow buoy",
                "course": "tank",
                "color_name": "yellow",
                "color_hex": "#facc15",
                "layer": "underwater marker",
                "item_kind": "buoy",
                "x": yellow_x,
                "y": yellow_y,
                "z": TEST_TANK_YELLOW_Z_M,
                "fixed_underwater": True,
            },
            {
                "id": TEST_TANK_PINGER_ID,
                "prefix": TEST_TANK_PINGER_ID,
                "label": "Pinger",
                "course": "tank",
                "color_name": "red",
                "color_hex": "#dc2626",
                "layer": "standalone acoustic",
                "item_kind": "pinger",
                "x": pinger_x,
                "y": pinger_y,
                "z": TEST_TANK_PINGER_Z_M,
                "fixed_underwater": True,
            },
        ],
    }


def _remove_competition_layout(worldbody: ET.Element, root: ET.Element) -> None:
    for child in list(worldbody):
        name = str(child.get("name") or "")
        if child.tag == "body" and name.startswith("course_buoy_"):
            worldbody.remove(child)
        elif child.tag == "geom" and name.startswith("course_"):
            worldbody.remove(child)

    for parent in root.iter():
        if parent.tag in {"worldbody", "default"}:
            continue
        for child in list(parent):
            if any("course_buoy_" in str(value) for value in child.attrib.values()):
                parent.remove(child)


def _resize_pool(worldbody: ET.Element) -> None:
    floor_z = -(TEST_TANK_DEPTH_M + _WALL_HALF_THICKNESS_M)
    wall_center_z = (_WALL_TOP_Z_M - TEST_TANK_DEPTH_M) / 2.0
    wall_half_z = (_WALL_TOP_Z_M + TEST_TANK_DEPTH_M) / 2.0
    x_outer = TEST_TANK_X_HALF_M + _WALL_HALF_THICKNESS_M
    y_outer = TEST_TANK_Y_HALF_M + _WALL_HALF_THICKNESS_M

    _set_geom(worldbody, "pool_floor", pos=(0, 0, floor_z), size=(TEST_TANK_X_HALF_M, TEST_TANK_Y_HALF_M, _WALL_HALF_THICKNESS_M))
    _set_geom(worldbody, "pool_wall_px", pos=(x_outer, 0, wall_center_z), size=(_WALL_HALF_THICKNESS_M, y_outer, wall_half_z))
    _set_geom(worldbody, "pool_wall_nx", pos=(-x_outer, 0, wall_center_z), size=(_WALL_HALF_THICKNESS_M, y_outer, wall_half_z))
    _set_geom(worldbody, "pool_wall_py", pos=(0, y_outer, wall_center_z), size=(x_outer, _WALL_HALF_THICKNESS_M, wall_half_z))
    _set_geom(worldbody, "pool_wall_ny", pos=(0, -y_outer, wall_center_z), size=(x_outer, _WALL_HALF_THICKNESS_M, wall_half_z))
    _set_geom(worldbody, "water_vis", pos=(0, 0, -TEST_TANK_DEPTH_M / 2.0), size=(TEST_TANK_X_HALF_M, TEST_TANK_Y_HALF_M, TEST_TANK_DEPTH_M / 2.0))
    _set_geom(worldbody, "water_surface", pos=(0, 0, 0.004), size=(TEST_TANK_X_HALF_M, TEST_TANK_Y_HALF_M, 0.004))

    x_line = TEST_TANK_X_HALF_M - 0.05
    y_line = TEST_TANK_Y_HALF_M - 0.05
    _set_fromto(worldbody, "waterline_x_min", (-x_line, -TEST_TANK_Y_HALF_M, 0.012, -x_line, TEST_TANK_Y_HALF_M, 0.012))
    _set_fromto(worldbody, "waterline_x_max", (x_line, -TEST_TANK_Y_HALF_M, 0.012, x_line, TEST_TANK_Y_HALF_M, 0.012))
    _set_fromto(worldbody, "waterline_y_min", (-TEST_TANK_X_HALF_M, -y_line, 0.012, TEST_TANK_X_HALF_M, -y_line, 0.012))
    _set_fromto(worldbody, "waterline_y_max", (-TEST_TANK_X_HALF_M, y_line, 0.012, TEST_TANK_X_HALF_M, y_line, 0.012))
    _set_geom(worldbody, "waterline_wall_x_min", pos=(-x_line - 0.03, 0, 0.006), size=(0.018, TEST_TANK_Y_HALF_M, 0.018))
    _set_geom(worldbody, "waterline_wall_x_max", pos=(x_line + 0.03, 0, 0.006), size=(0.018, TEST_TANK_Y_HALF_M, 0.018))
    _set_geom(worldbody, "waterline_wall_y_min", pos=(0, -y_line - 0.03, 0.006), size=(TEST_TANK_X_HALF_M, 0.018, 0.018))
    _set_geom(worldbody, "waterline_wall_y_max", pos=(0, y_line + 0.03, 0.006), size=(TEST_TANK_X_HALF_M, 0.018, 0.018))

    overview = _direct_named_child(worldbody, "camera", "course_overview")
    if overview is not None:
        overview.set("pos", "0 0 6.2")
        overview.set("fovy", "54")
    side = _direct_named_child(worldbody, "camera", "course_side")
    if side is not None:
        side.set("pos", "0 -4.2 -0.45")
        side.set("fovy", "48")


def _append_test_tank_tile_wall_panels(worldbody: ET.Element) -> None:
    """Add square UV-mapped wall panels without changing collision geometry."""
    panel_z = -TEST_TANK_DEPTH_M / 2.0
    material = "pool_wall_tile_turquoise_panel"
    mesh = "pool_wall_tile_panel_mesh"

    def add(name: str, pos: tuple[float, float, float], quat: str) -> None:
        ET.SubElement(
            worldbody,
            "geom",
            name=name,
            type="mesh",
            mesh=mesh,
            material=material,
            pos=_format_values(pos),
            quat=quat,
            contype="0",
            conaffinity="0",
            group="0",
        )

    # Four unscaled square images span the 5.49 m front/rear walls.
    for index in range(4):
        x = (index - 1.5) * _TILE_PANEL_SIZE_M
        add(f"pool_wall_tile_panel_y_neg_{index:02d}", (x, -TEST_TANK_Y_HALF_M + 0.001, panel_z), "1 0 0 0")
        add(f"pool_wall_tile_panel_y_pos_{index:02d}", (x, TEST_TANK_Y_HALF_M - 0.001, panel_z), "0 1 0 0")

    # Two unscaled square images span each 2.74 m side wall.
    for index in range(2):
        y = (index - 0.5) * _TILE_PANEL_SIZE_M
        add(f"pool_wall_tile_panel_x_neg_{index:02d}", (-TEST_TANK_X_HALF_M + 0.001, y, panel_z), "0.70710678 0 0 -0.70710678")
        add(f"pool_wall_tile_panel_x_pos_{index:02d}", (TEST_TANK_X_HALF_M - 0.001, y, panel_z), "0.70710678 0 0 0.70710678")


def _append_test_tank_markers(
    worldbody: ET.Element,
    root: ET.Element,
    positions: dict[str, tuple[float, float]],
    *,
    magnet_base_template: ET.Element,
    float_template: ET.Element,
    source_prefix: str,
) -> None:
    yellow_x, yellow_y = positions["yellow"]
    _append_physical_test_tank_yellow_buoy(
        worldbody,
        root,
        yellow_x=yellow_x,
        yellow_y=yellow_y,
        magnet_base_template=magnet_base_template,
        float_template=float_template,
        source_prefix=source_prefix,
    )

    pinger_x, pinger_y = positions["pinger"]
    pinger = ET.SubElement(
        worldbody,
        "body",
        name=TEST_TANK_PINGER_ID,
        pos=_format_values((pinger_x, pinger_y, TEST_TANK_PINGER_Z_M)),
    )
    ET.SubElement(
        pinger,
        "geom",
        name=f"{TEST_TANK_PINGER_ID}_housing",
        type="cylinder",
        size="0.045 0.090",
        rgba="0.10 0.12 0.14 1",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        pinger,
        "geom",
        name=f"{TEST_TANK_PINGER_ID}_band",
        type="cylinder",
        pos="0 0 0.055",
        size="0.049 0.012",
        rgba="0.90 0.08 0.08 1",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        pinger,
        "site",
        name=TEST_TANK_PINGER_SITE_NAME,
        pos="0 0 0",
        size="0.012",
        rgba="1 0.05 0.05 0.85",
        group="4",
    )


def _append_physical_test_tank_yellow_buoy(
    worldbody: ET.Element,
    root: ET.Element,
    *,
    yellow_x: float,
    yellow_y: float,
    magnet_base_template: ET.Element,
    float_template: ET.Element,
    source_prefix: str,
) -> None:
    """Reuse the competition buoy and add a short detachable nylon tether."""

    prefix = TEST_TANK_YELLOW_BUOY_PREFIX
    magnet_base = copy.deepcopy(magnet_base_template)
    float_body = copy.deepcopy(float_template)
    _replace_prefix(magnet_base, source_prefix, prefix)
    _replace_prefix(float_body, source_prefix, prefix)

    base_z = -TEST_TANK_DEPTH_M
    # The physical template's CoB site is +35 mm from the free-body origin.
    float_body_z = TEST_TANK_YELLOW_Z_M - 0.035
    magnet_base.set("pos", _format_values((yellow_x, yellow_y, base_z)))
    float_body.set("pos", _format_values((yellow_x, yellow_y, float_body_z)))

    # The shallow test tank uses the same base/float hardware with a shortened
    # rigid stand.  The final 50 mm between the magnet and moving attachment is
    # represented by the detachable composite cable below.
    extension = TEST_TANK_STICK_EXTENSION_M
    fixed_shift = -extension
    _require_named(magnet_base, "geom", f"{prefix}_mooring_rod").set(
        "fromto", _format_values((0.0, 0.0, 0.109, 0.0, 0.0, 0.395 + fixed_shift))
    )
    _require_named(magnet_base, "geom", f"{prefix}_rod_top_jig").set(
        "pos", _format_values((0.0, 0.0, 0.405 + fixed_shift))
    )
    _require_named(magnet_base, "geom", f"{prefix}_fixed_magnet_stem").set(
        "fromto", _format_values((0.0, 0.0, 0.405 + fixed_shift, 0.0, 0.0, 0.440 + fixed_shift))
    )
    _require_named(magnet_base, "geom", f"{prefix}_fixed_magnet").set(
        "pos", _format_values((0.0, 0.0, 0.450 + fixed_shift))
    )
    magnet_site = _require_named(magnet_base, "site", f"{prefix}_magnet_site")
    magnet_site.set("pos", _format_values((0.0, 0.0, 0.460 + fixed_shift)))

    pvc_pipe = _require_named(float_body, "geom", f"{prefix}_pvc_pipe")
    pvc_pipe.set("fromto", _format_values((0.0, 0.0, -0.063, 0.0, 0.0, -0.170 - extension)))
    pvc_pipe.set("size", "0.0075")
    pvc_pipe.set("rgba", "1.00 0.98 0.84 1")
    for suffix, base_z_value in (
        ("moving_lower_jig", -0.183),
        ("magnet_lower_plate", -0.195),
        ("magnet", -0.205),
    ):
        _require_named(float_body, "geom", f"{prefix}_{suffix}").set(
            "pos", _format_values((0.0, 0.0, base_z_value - extension))
        )

    # Darker, thicker latitude rings expose the yellow float silhouette in
    # the simulated camera while remaining collisionless visual geometry.
    float_geom = _require_named(float_body, "geom", f"{prefix}_float_geom")
    float_geom.set("rgba", "1.00 0.88 0.01 1")
    for band_index in range(1, 9):
        band = _require_named(float_body, "geom", f"{prefix}_equator_band_{band_index:02d}")
        band.set("size", "0.0032")
        band.set("rgba", "0.48 0.16 0.00 1")
    _append_test_tank_float_outline_ring(float_body, prefix=prefix, ring_name="upper", z=0.080, radius=0.047)
    _append_test_tank_float_outline_ring(float_body, prefix=prefix, ring_name="lower", z=-0.010, radius=0.047)

    attach_site = _require_named(float_body, "site", f"{prefix}_attach_site")
    attach_site.set("pos", _format_values((0.0, 0.0, -0.215 - extension)))
    attach_local_z = _vector_z(attach_site.get("pos"), label=f"{prefix}_attach_site")
    line_bottom_z = base_z + 0.460
    line_bottom_z += fixed_shift
    line_top_z = float_body_z + attach_local_z
    nylon_length = line_top_z - line_bottom_z
    if not math.isclose(nylon_length, TEST_TANK_NYLON_LENGTH_M, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError(f"test-tank nylon contract is {nylon_length:.6f} m, expected {TEST_TANK_NYLON_LENGTH_M:.6f} m")

    worldbody.append(magnet_base)
    worldbody.append(float_body)
    _append_nylon_cable(
        worldbody,
        prefix=prefix,
        x=yellow_x,
        y=yellow_y,
        bottom_z=line_bottom_z,
        top_z=line_top_z,
    )
    _append_test_tank_buoy_equalities(
        root,
        prefix=prefix,
        relative_float_z=float_body_z - base_z,
        cable_bottom_anchor=(yellow_x, yellow_y, line_bottom_z),
    )


def _append_test_tank_float_outline_ring(
    float_body: ET.Element,
    *,
    prefix: str,
    ring_name: str,
    z: float,
    radius: float,
) -> None:
    points = [
        (radius * math.cos(2.0 * math.pi * index / 8.0), radius * math.sin(2.0 * math.pi * index / 8.0), z)
        for index in range(8)
    ]
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        ET.SubElement(
            float_body,
            "geom",
            name=f"{prefix}_{ring_name}_outline_{index + 1:02d}",
            type="capsule",
            fromto=_format_values((*start, *end)),
            size="0.0024",
            rgba="0.48 0.16 0.00 1",
            density="0",
            contype="0",
            conaffinity="0",
            group="0",
        )


def _append_nylon_cable(
    worldbody: ET.Element,
    *,
    prefix: str,
    x: float,
    y: float,
    bottom_z: float,
    top_z: float,
) -> None:
    vertices = [
        (x, y, bottom_z + (top_z - bottom_z) * index / 3.0)
        for index in range(4)
    ]
    vertex_text = "\n" + "\n".join(f"      {_format_values(vertex)}" for vertex in vertices) + "\n      "
    composite = ET.SubElement(
        worldbody,
        "composite",
        prefix=f"{prefix}_flex_line_",
        type="cable",
        initial="none",
        vertex=vertex_text,
    )
    plugin = ET.SubElement(composite, "plugin", plugin="mujoco.elasticity.cable")
    ET.SubElement(plugin, "config", key="twist", value="80")
    ET.SubElement(plugin, "config", key="bend", value="1.2")
    ET.SubElement(plugin, "config", key="vmax", value="0.30")
    ET.SubElement(composite, "joint", kind="main", damping="0.010", armature="0.00002")
    ET.SubElement(
        composite,
        "geom",
        type="capsule",
        size="0.0010",
        density="35",
        rgba="0.88 0.90 0.92 1",
        contype="0",
        conaffinity="1",
        condim="4",
        friction="0.90 0.06 0.006",
        solref="0.016 1",
        solimp="0.82 0.95 0.001",
    )


def _append_test_tank_buoy_equalities(
    root: ET.Element,
    *,
    prefix: str,
    relative_float_z: float,
    cable_bottom_anchor: tuple[float, float, float],
) -> None:
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "weld",
        name=f"{prefix}_collector_weld",
        body1="front_open_buoy_collector",
        body2=f"{prefix}_float",
        relpose="0 0 0.2 1 0 0 0",
        active="false",
        solref="0.050 1",
        solimp="0.75 0.95 0.002",
    )
    ET.SubElement(
        equality,
        "weld",
        name=f"{prefix}_magnet_weld",
        body1=f"{prefix}_magnet_base",
        body2=f"{prefix}_float",
        relpose=f"0 0 {relative_float_z:.6f} 1 0 0 0",
        solref="0.020 1",
        solimp="0.95 0.99 0.0005",
    )
    anchor = _format_values(cable_bottom_anchor)
    ET.SubElement(
        equality,
        "connect",
        name=f"{prefix}_flex_line_bottom_connect",
        body1=f"{prefix}_flex_line_B_first",
        body2=f"{prefix}_magnet_base",
        anchor=anchor,
        solref="0.008 1",
        solimp="0.95 0.99 0.0005",
    )
    ET.SubElement(
        equality,
        "weld",
        name=f"{prefix}_flex_line_top_connect",
        body1=f"{prefix}_flex_line_B_last",
        body2=f"{prefix}_float",
        solref="0.008 1",
        solimp="0.92 0.98 0.001",
    )


def _replace_prefix(element: ET.Element, old: str, new: str) -> None:
    for child in element.iter():
        for key, value in tuple(child.attrib.items()):
            if old in value:
                child.set(key, value.replace(old, new))


def _require_named(root: ET.Element, tag: str, name: str) -> ET.Element:
    element = _named_child(root, tag, name)
    if element is None:
        raise ValueError(f"required physical buoy template element missing: {name}")
    return element


def _vector_z(value: str | None, *, label: str) -> float:
    parts = [float(part) for part in str(value or "").split()]
    if len(parts) != 3:
        raise ValueError(f"invalid {label} position: {value!r}")
    return parts[2]


def _set_geom(worldbody: ET.Element, name: str, *, pos: tuple[float, ...], size: tuple[float, ...]) -> None:
    geom = _direct_named_child(worldbody, "geom", name)
    if geom is None:
        raise ValueError(f"required pool geom missing: {name}")
    geom.set("pos", _format_values(pos))
    geom.set("size", _format_values(size))


def _set_fromto(worldbody: ET.Element, name: str, values: tuple[float, ...]) -> None:
    geom = _direct_named_child(worldbody, "geom", name)
    if geom is None:
        raise ValueError(f"required pool geom missing: {name}")
    geom.set("fromto", _format_values(values))


def _direct_named_child(parent: ET.Element, tag: str, name: str) -> ET.Element | None:
    return next((child for child in parent.findall(tag) if child.get("name") == name), None)


def _named_child(root: ET.Element, tag: str, name: str) -> ET.Element | None:
    return next((child for child in root.findall(f".//{tag}") if child.get("name") == name), None)


def _set_xy_preserve_z(element: ET.Element, x: float, y: float) -> None:
    parts = [float(part) for part in str(element.get("pos") or "").split()]
    if len(parts) != 3:
        raise ValueError(f"invalid pos for {element.get('name')}: {element.get('pos')!r}")
    clamped = _clamped_xy_dict(x, y)
    element.set("pos", _format_values((clamped["x"], clamped["y"], parts[2])))


def _clamped_xy_dict(x: float, y: float) -> dict[str, float]:
    return {
        "x": max(-TEST_TANK_X_HALF_M + _POSITION_MARGIN_M, min(TEST_TANK_X_HALF_M - _POSITION_MARGIN_M, float(x))),
        "y": max(-TEST_TANK_Y_HALF_M + _POSITION_MARGIN_M, min(TEST_TANK_Y_HALF_M - _POSITION_MARGIN_M, float(y))),
    }


def _xy_dict(value: dict[str, Any], *, fallback: dict[str, float]) -> dict[str, float]:
    try:
        x = float(value.get("x", fallback["x"]))
        y = float(value.get("y", fallback["y"]))
    except (TypeError, ValueError):
        x, y = float(fallback["x"]), float(fallback["y"])
    if not (math.isfinite(x) and math.isfinite(y)):
        x, y = float(fallback["x"]), float(fallback["y"])
    return _clamped_xy_dict(x, y)


def _xy_tuple(value: object, *, fallback: dict[str, float]) -> tuple[float, float]:
    if not isinstance(value, dict):
        value = fallback
    clean = _xy_dict(value, fallback=fallback)
    return clean["x"], clean["y"]


def _format_values(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.6f}".rstrip("0").rstrip(".") if value else "0" for value in values)


__all__ = [
    "COURSE_MODE_COMPETITION",
    "COURSE_MODE_TEST_TANK",
    "COURSE_MODES",
    "CourseRuntimeSelection",
    "TEST_TANK_DEPTH_M",
    "TEST_TANK_HOMING_SUCCESS_RANGE_M",
    "TEST_TANK_LENGTH_M",
    "TEST_TANK_PINGER_ID",
    "TEST_TANK_PINGER_SITE_NAME",
    "TEST_TANK_PINGER_Z_M",
    "TEST_TANK_NYLON_LENGTH_M",
    "TEST_TANK_STICK_EXTENSION_M",
    "TEST_TANK_WIDTH_M",
    "TEST_TANK_X_HALF_M",
    "TEST_TANK_YELLOW_BUOY_PREFIX",
    "TEST_TANK_YELLOW_ID",
    "TEST_TANK_Y_HALF_M",
    "default_course_layout_config",
    "generate_test_tank_scene",
    "load_course_layout_config",
    "normalize_course_mode",
    "prepare_active_course_runtime",
    "save_course_layout_config",
    "test_tank_payload",
    "test_tank_positions",
]
