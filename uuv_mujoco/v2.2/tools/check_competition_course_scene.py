#!/usr/bin/env python3
"""Validate the 2026 competition-course proxy in the MuJoCo tank scene."""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
COURSE_BUOY_FULL_IMMERSION_NET_LIFT_N = 0.98
COURSE_BUOY_FLOAT_HALF_HEIGHT_M = 0.085
COURSE_BUOY_MAX_VISIBLE_FLOAT_MASS_KG = 0.021
COURSE_BUOY_MIN_SURFACE_CENTER_Z_M = 0.045


def floats(value: str | None) -> tuple[float, ...]:
    if not value:
        return ()
    return tuple(float(part) for part in value.split())


def body_map(root: ET.Element) -> dict[str, ET.Element]:
    bodies: dict[str, ET.Element] = {}
    for body in root.findall(".//body"):
        name = body.get("name")
        if name:
            bodies[name] = body
    return bodies


def child_by_name(body: ET.Element, tag: str, name: str) -> ET.Element | None:
    for child in body.findall(tag):
        if child.get("name") == name:
            return child
    return None


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def pos_z(element: ET.Element) -> float:
    pos = floats(element.get("pos"))
    if len(pos) != 3:
        raise ValueError(f"{element.get('name')} has invalid pos={element.get('pos')!r}")
    return pos[2]


def geom_z_bounds(element: ET.Element) -> tuple[float, float]:
    size = floats(element.get("size"))
    if len(size) < 3:
        raise ValueError(f"{element.get('name')} has invalid size={element.get('size')!r}")
    center_z = pos_z(element)
    return center_z - size[2], center_z + size[2]


def float_attr(element: ET.Element, name: str, default: float = 0.0) -> float:
    value = element.get(name)
    if value is None:
        return default
    return float(value)


def inertial_mass(body: ET.Element) -> float:
    inertial = body.find("inertial")
    if inertial is None:
        return math.nan
    return float_attr(inertial, "mass", math.nan)


def surface_float_center_z(mass_kg: float) -> float:
    weight_n = mass_kg * 9.81
    full_upthrust_n = weight_n + COURSE_BUOY_FULL_IMMERSION_NET_LIFT_N
    equilibrium_fraction = min(max(weight_n / full_upthrust_n, 0.0), 1.0)
    return COURSE_BUOY_FLOAT_HALF_HEIGHT_M - 2.0 * COURSE_BUOY_FLOAT_HALF_HEIGHT_M * equilibrium_fraction


def check_tank(root: ET.Element, failures: list[str]) -> None:
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom") if geom.get("name")}
    meshes = {mesh.get("name"): mesh for mesh in root.findall(".//mesh") if mesh.get("name")}
    cameras = {camera.get("name"): camera for camera in root.findall(".//camera") if camera.get("name")}
    floor = geoms.get("pool_floor")
    water = geoms.get("water_vis")
    surface = geoms.get("water_surface")
    require(floor is not None, "missing pool_floor", failures)
    require(water is not None, "missing water_vis", failures)
    require(surface is not None, "missing water_surface", failures)
    if floor is not None:
        require(floats(floor.get("size")) == (17.5, 15.0, 0.05), "pool_floor must be 35m x 30m", failures)
        require(math.isclose(pos_z(floor), -11.05, abs_tol=1e-6), "pool_floor must sit below 11m depth", failures)
    if water is not None:
        require(floats(water.get("size")) == (17.5, 15.0, 5.5), "water volume must fill 11m depth", failures)
    if surface is not None:
        require(math.isclose(pos_z(surface), 0.004, abs_tol=1e-6), "water surface must stay at z=0", failures)
        rgba = floats(surface.get("rgba"))
        require(len(rgba) == 4 and rgba[3] <= 0.25, "water surface must be transparent enough to see underwater course", failures)
    for name in ("pool_wall_px", "pool_wall_nx", "pool_wall_py", "pool_wall_ny"):
        wall = geoms.get(name)
        require(wall is not None, f"missing tank wall {name}", failures)
        if wall is None:
            continue
        bottom_z, top_z = geom_z_bounds(wall)
        require(math.isclose(bottom_z, -11.0, abs_tol=1e-6), f"{name} bottom must stay at 11m depth", failures)
        require(top_z >= 0.55, f"{name} must extend above the water surface with visible freeboard", failures)
    for name in (
        "waterline_x_min",
        "waterline_x_max",
        "waterline_y_min",
        "waterline_y_max",
        "waterline_wall_x_min",
        "waterline_wall_x_max",
        "waterline_wall_y_min",
        "waterline_wall_y_max",
    ):
        require(name in geoms, f"missing visible waterline geom {name}", failures)
    for name in ("course_overview", "course_side"):
        require(name in cameras, f"missing viewer camera {name}", failures)
    require("ping360_visual" not in geoms, "Ping360 sonar STL visual geom must stay removed", failures)
    require("ping360_body_no_cable" not in meshes, "Ping360 sonar STL mesh asset must stay removed", failures)
    collector_top = geoms.get("collector_top_net_proxy")
    require(collector_top is not None, "missing collector_top_net_proxy", failures)
    if collector_top is not None:
        require(
            math.isclose(pos_z(collector_top), 0.3130, abs_tol=1e-6),
            "collector roof must use the compact two-thirds-height capture net",
            failures,
        )
    for name in ("stereo_left", "stereo_right"):
        camera = cameras.get(name)
        require(camera is not None, f"missing onboard camera {name}", failures)
        if camera is None:
            continue
        require(
            floats(camera.get("xyaxes")) == (0.0, -1.0, 0.0, 0.0, 0.0, 1.0),
            f"{name} must look straight forward along robot +X with image up +Z",
            failures,
        )
        require(camera.get("quat") is None, f"{name} must use explicit xyaxes, not ambiguous quat", failures)


def check_red_buoys(bodies: dict[str, ET.Element], failures: list[str]) -> None:
    red = sorted(name for name in bodies if name.startswith("course_buoy_") and "_red_" in name and name.endswith("_float"))
    require(len(red) == 10, f"expected 10 red surface buoys, got {len(red)}", failures)
    require(sum(name.startswith("course_buoy_a_") for name in red) == 5, "expected 5 red buoys in A course", failures)
    require(sum(name.startswith("course_buoy_b_") for name in red) == 5, "expected 5 red buoys in B course", failures)
    for body_name in red:
        body = bodies[body_name]
        prefix = body_name.removesuffix("_float")
        require(math.isclose(float_attr(body, "gravcomp"), 0.0, abs_tol=1e-9), f"{body_name} must not pin depth with gravcomp", failures)
        mass_kg = inertial_mass(body)
        target_z = surface_float_center_z(mass_kg)
        require(abs(pos_z(body) - target_z) < 1e-3, f"{body_name} must start at surface equilibrium z={target_z:.3f}, got {pos_z(body):.3f}", failures)
        require(
            mass_kg <= COURSE_BUOY_MAX_VISIBLE_FLOAT_MASS_KG,
                f"{body_name} mass must stay light enough for 0.98N net lift to visibly float; got {mass_kg:.3f}kg",
            failures,
        )
        require(
            target_z >= COURSE_BUOY_MIN_SURFACE_CENTER_Z_M,
            f"{body_name} 0.98N net-lift equilibrium must keep center visibly above water; got z={target_z:.3f}m",
            failures,
        )
        free_joint = child_by_name(body, "joint", f"{prefix}_free")
        require(free_joint is not None and free_joint.get("type") == "free", f"{body_name} must use a free joint for real x/y/z buoy motion", failures)
        require(child_by_name(body, "joint", f"{prefix}_slide_x") is None, f"{body_name} must not pin surface x with slide joint", failures)
        require(child_by_name(body, "joint", f"{prefix}_slide_y") is None, f"{body_name} must not pin surface y with slide joint", failures)
        require(child_by_name(body, "site", f"{prefix}_attach_site") is None, f"{body_name} must not have magnet attach site", failures)
        required_geoms = [
            f"{prefix}_float_geom",
            f"{prefix}_viewer_surface_disc",
            f"{prefix}_top_stem",
            f"{prefix}_top_bolt",
            f"{prefix}_top_jig",
            f"{prefix}_bottom_socket",
            f"{prefix}_pvc_pipe",
            f"{prefix}_lower_bolt",
            f"{prefix}_lower_jig",
        ]
        for geom_name in required_geoms:
            require(child_by_name(body, "geom", geom_name) is not None, f"{body_name} missing {geom_name}", failures)
        viewer_disc = child_by_name(body, "geom", f"{prefix}_viewer_surface_disc")
        if viewer_disc is not None:
            require(viewer_disc.get("group") == "5", f"{prefix} viewer surface disc must stay in hidden helper group 5", failures)
        bands = [geom for geom in body.findall("geom") if "_equator_band_" in (geom.get("name") or "")]
        require(len(bands) == 8, f"{body_name} must have 8 equator-band segments", failures)


def check_fixed_buoys(root: ET.Element, bodies: dict[str, ET.Element], failures: list[str]) -> None:
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom") if geom.get("name")}
    welds = {weld.get("name"): weld for weld in root.findall("./equality/weld") if weld.get("name")}
    bases = sorted(name for name in bodies if name.startswith("course_buoy_") and name.endswith("_magnet_base"))
    require(len(bases) == 15, f"expected 15 magnet-attached yellow/orange/white buoys, got {len(bases)}", failures)
    yellow = [name for name in bases if "_yellow_" in name]
    orange = [name for name in bases if "_orange_" in name]
    pinger = [name for name in bases if "_pinger_white_" in name]
    require(len(yellow) == 10, f"expected 10 yellow fixed buoys, got {len(yellow)}", failures)
    require(sum(name.startswith("course_buoy_a_") for name in yellow) == 5, "expected 5 yellow buoys in A course", failures)
    require(sum(name.startswith("course_buoy_b_") for name in yellow) == 5, "expected 5 yellow buoys in B course", failures)
    require(len(orange) == 4, f"expected 4 orange fixed buoys, got {len(orange)}", failures)
    require(sum(name.startswith("course_buoy_a_") for name in orange) == 2, "expected 2 orange buoys in A course", failures)
    require(sum(name.startswith("course_buoy_b_") for name in orange) == 2, "expected 2 orange buoys in B course", failures)
    require(len(pinger) == 1, f"expected 1 central pinger-attached buoy, got {len(pinger)}", failures)
    for base_name in bases:
        base = bodies[base_name]
        prefix = base_name.removesuffix("_magnet_base")
        float_body = bodies.get(f"{prefix}_float")
        require(float_body is not None, f"{prefix} missing float body", failures)
        if float_body is None:
            continue

        require(math.isclose(pos_z(base), -11.0, abs_tol=1e-6), f"{base_name} must sit on 11m floor", failures)
        require(math.isclose(pos_z(float_body), -8.5, abs_tol=1e-6), f"{prefix}_float must be 2.5m above floor", failures)
        require(
            math.isclose(float_attr(float_body, "gravcomp"), 0.0, abs_tol=1e-9),
            f"{prefix}_float must not use unconditional gravcomp; runtime waterline buoyancy provides 0.98N net lift only underwater",
            failures,
        )
        mass_kg = inertial_mass(float_body)
        require(
            mass_kg <= COURSE_BUOY_MAX_VISIBLE_FLOAT_MASS_KG,
            f"{prefix}_float mass must stay light enough to float after magnet release; got {mass_kg:.3f}kg",
            failures,
        )
        require(
            surface_float_center_z(mass_kg) >= COURSE_BUOY_MIN_SURFACE_CENTER_Z_M,
            f"{prefix}_float 0.98N net-lift equilibrium must keep center visibly above water after release; got z={surface_float_center_z(mass_kg):.3f}m",
            failures,
        )
        require(child_by_name(float_body, "joint", f"{prefix}_free") is not None, f"{prefix}_float must keep a free joint for future magnet release", failures)
        require(f"{prefix}_surface_projection" in geoms, f"{prefix} missing viewer surface projection", failures)
        require(f"{prefix}_surface_projection_outline" in geoms, f"{prefix} missing viewer surface projection outline", failures)
        for helper_name in (f"{prefix}_surface_projection", f"{prefix}_surface_projection_outline"):
            helper = geoms.get(helper_name)
            if helper is not None:
                require(helper.get("group") == "5", f"{helper_name} must stay in hidden helper group 5", failures)

        magnet_site = child_by_name(base, "site", f"{prefix}_magnet_site")
        attach_site = child_by_name(float_body, "site", f"{prefix}_attach_site")
        require(magnet_site is not None, f"{base_name} missing magnet site", failures)
        require(attach_site is not None, f"{prefix}_float missing attach site", failures)
        weld = welds.get(f"{prefix}_magnet_weld")
        require(weld is not None, f"{prefix} missing magnet weld equality", failures)
        if weld is not None:
            require(weld.get("site1") == f"{prefix}_magnet_site", f"{prefix} magnet weld must use magnet site as site1", failures)
            require(weld.get("site2") == f"{prefix}_attach_site", f"{prefix} magnet weld must use attach site as site2", failures)
        if magnet_site is not None and attach_site is not None:
            magnet_world_z = pos_z(base) + pos_z(magnet_site)
            attach_world_z = pos_z(float_body) + pos_z(attach_site)
            require(
                math.isclose(magnet_world_z, attach_world_z, abs_tol=1e-6),
                f"{prefix} magnet and attach sites do not coincide: {magnet_world_z:.3f} vs {attach_world_z:.3f}",
                failures,
            )

        nylon = child_by_name(base, "geom", f"{prefix}_nylon_line")
        nylon_segments = [
            geom
            for geom in base.findall("geom")
            if (geom.get("name") or "").startswith(f"{prefix}_nylon_line_seg_")
        ]
        pvc = child_by_name(base, "geom", f"{prefix}_pvc_pipe")
        magnet = child_by_name(base, "geom", f"{prefix}_magnet")
        weight = child_by_name(base, "geom", f"{prefix}_diver_weight")
        require(nylon is None, f"{base_name} must not use a single rigid nylon cylinder", failures)
        require(len(nylon_segments) >= 6, f"{base_name} must use segmented visual nylon rope", failures)
        require(pvc is not None, f"{base_name} missing visible PVC pipe", failures)
        require(magnet is not None, f"{base_name} missing magnet", failures)
        require(weight is not None, f"{base_name} missing diver weight", failures)
        for segment in nylon_segments:
            require(segment.get("type") == "capsule", f"{segment.get('name')} must be a capsule rope segment", failures)
            require(segment.get("contype") == "0", f"{segment.get('name')} must not collide as a rigid obstacle", failures)
            require(segment.get("conaffinity") == "0", f"{segment.get('name')} must not collide as a rigid obstacle", failures)
        if pvc is not None:
            rgba = floats(pvc.get("rgba"))
            require(len(rgba) == 4 and rgba[:3] == (1.0, 1.0, 1.0) and rgba[3] == 1.0, f"{prefix} PVC must be opaque white", failures)
        if magnet is not None:
            require(magnet.get("contype") == "0", f"{prefix} magnet must be visual-only while weld handles attachment", failures)
            require(
                magnet.get("conaffinity") == "0",
                f"{prefix} magnet must be visual-only while weld handles attachment",
                failures,
            )

        bands = [geom for geom in float_body.findall("geom") if "_equator_band_" in (geom.get("name") or "")]
        require(len(bands) == 8, f"{prefix}_float must have 8 equator-band segments", failures)
        require(child_by_name(float_body, "geom", f"{prefix}_top_jig") is not None, f"{prefix}_float missing top jig", failures)
        require(child_by_name(float_body, "geom", f"{prefix}_bottom_socket") is not None, f"{prefix}_float missing bottom socket", failures)

    pinger = bodies.get("course_buoy_pinger_white_1_float")
    require(pinger is not None, "missing pinger white float", failures)
    if pinger is not None:
        for geom_name in [
            "course_buoy_pinger_white_1_pinger_body",
            "course_buoy_pinger_white_1_pinger_left_rod",
            "course_buoy_pinger_white_1_pinger_right_rod",
            "course_buoy_pinger_white_1_pinger_bottom_jig",
        ]:
            require(child_by_name(pinger, "geom", geom_name) is not None, f"pinger missing {geom_name}", failures)


def check_visual_propellers(bodies: dict[str, ET.Element], failures: list[str]) -> None:
    prop_names = (
        "prop_ver_lf",
        "prop_ver_lr",
        "prop_ver_rf",
        "prop_ver_rr",
        "prop_yaw_lf",
        "prop_yaw_lr",
        "prop_yaw_rf",
        "prop_yaw_rr",
    )
    for prop_name in prop_names:
        body = bodies.get(prop_name)
        require(body is not None, f"missing visual propeller body {prop_name}", failures)
        if body is None:
            continue
        inertial = body.find("inertial")
        require(inertial is not None, f"{prop_name} must define stable visual-only inertial", failures)
        if inertial is not None:
            require(
                math.isclose(float_attr(inertial, "mass"), 0.001, abs_tol=1e-9),
                f"{prop_name} visual propeller mass must stay tiny",
                failures,
            )
            require(
                floats(inertial.get("diaginertia")) == (0.000001, 0.000001, 0.000001),
                f"{prop_name} visual propeller inertia must stay explicit and isotropic",
                failures,
            )
        joint = child_by_name(body, "joint", f"{prop_name}_j")
        require(joint is not None, f"{prop_name} missing visual spin joint", failures)
        if joint is not None:
            require(float_attr(joint, "armature") >= 0.01, f"{prop_name} visual spin joint must have stabilizing armature", failures)
            require(float_attr(joint, "damping") >= 1.0, f"{prop_name} visual spin joint must have stabilizing damping", failures)
        prop_geom = next((geom for geom in body.findall("geom") if geom.get("mesh") == "t200_prop"), None)
        require(prop_geom is not None, f"{prop_name} missing t200 prop visual mesh", failures)
        if prop_geom is not None:
            require(prop_geom.get("density") == "0", f"{prop_name} prop mesh must be massless visual geometry", failures)
            require(prop_geom.get("contype") == "0", f"{prop_name} prop mesh must not collide", failures)
            require(prop_geom.get("conaffinity") == "0", f"{prop_name} prop mesh must not collide", failures)


def main() -> int:
    root = ET.parse(SCENE).getroot()
    failures: list[str] = []
    bodies = body_map(root)
    check_tank(root, failures)
    check_red_buoys(bodies, failures)
    check_fixed_buoys(root, bodies, failures)
    check_visual_propellers(bodies, failures)
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("competition course scene OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
