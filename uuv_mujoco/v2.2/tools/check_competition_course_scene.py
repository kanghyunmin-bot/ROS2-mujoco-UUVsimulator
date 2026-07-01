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
        require(
            child_by_name(body, "geom", f"{prefix}_viewer_surface_disc") is None,
            f"{prefix}_viewer_surface_disc must not exist; it creates distracting contact-time discs",
            failures,
        )
        float_geom = child_by_name(body, "geom", f"{prefix}_float_geom")
        if float_geom is not None:
            require(float_geom.get("contype") == "2", f"{prefix}_float_geom must use buoy collision contype=2", failures)
            require(float_geom.get("conaffinity") == "1", f"{prefix}_float_geom must collide with vehicle/collector but not other buoys", failures)
        bands = [geom for geom in body.findall("geom") if "_equator_band_" in (geom.get("name") or "")]
        require(len(bands) == 8, f"{body_name} must have 8 equator-band segments", failures)


def check_fixed_buoys(root: ET.Element, bodies: dict[str, ET.Element], failures: list[str]) -> None:
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom") if geom.get("name")}
    composites = {
        composite.get("prefix"): composite
        for composite in root.findall("./worldbody/composite")
        if composite.get("prefix")
    }
    connects = {
        connect.get("name"): connect
        for connect in root.findall("./equality/connect")
        if connect.get("name")
    }
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
        require(f"{prefix}_tether_jig" not in bodies, f"{prefix}_tether_jig free body must not exist; visible jig rides on the float for stability", failures)
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
        float_geom = child_by_name(float_body, "geom", f"{prefix}_float_geom")
        require(float_geom is not None, f"{prefix}_float missing float geom", failures)
        if float_geom is not None:
            require(float_geom.get("contype") == "2", f"{prefix}_float_geom must use buoy collision contype=2", failures)
            require(float_geom.get("conaffinity") == "1", f"{prefix}_float_geom must collide with vehicle/collector but not other buoys", failures)
        require(child_by_name(float_body, "joint", f"{prefix}_free") is not None, f"{prefix}_float must keep a free joint for future magnet release", failures)
        require(f"{prefix}_surface_projection" not in geoms, f"{prefix} surface projection disc must not exist", failures)
        require(
            f"{prefix}_surface_projection_outline" not in geoms,
            f"{prefix} surface projection outline disc must not exist",
            failures,
        )

        magnet_site = child_by_name(base, "site", f"{prefix}_magnet_site")
        attach_site = child_by_name(float_body, "site", f"{prefix}_attach_site")
        require(magnet_site is not None, f"{base_name} missing invisible magnet site", failures)
        require(attach_site is not None, f"{prefix}_float missing attach site", failures)
        weld = welds.get(f"{prefix}_magnet_weld")
        require(weld is not None, f"{prefix} missing magnet weld equality", failures)
        if weld is not None:
            require(weld.get("site1") == f"{prefix}_magnet_site", f"{prefix} magnet weld must use magnet site as site1", failures)
            require(weld.get("site2") == f"{prefix}_attach_site", f"{prefix} magnet weld must use attach site as site2", failures)
            require(weld.get("solref") == "0.030 1", f"{prefix} magnet weld must be soft enough to tilt before release", failures)
            require(weld.get("solimp") == "0.75 0.95 0.002", f"{prefix} magnet weld must be soft enough to tilt before release", failures)
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
        magnet = child_by_name(float_body, "geom", f"{prefix}_magnet")
        weight = child_by_name(base, "geom", f"{prefix}_diver_weight")
        require(nylon is None, f"{base_name} must not use a single rigid nylon cylinder", failures)
        require(len(nylon_segments) == 0, f"{base_name} must use flex-line cable composite, not rigid rope segments", failures)
        flex_line = composites.get(f"{prefix}_flex_line_")
        require(flex_line is not None, f"{prefix} missing flexible cable composite", failures)
        if flex_line is not None:
            vertices = floats(flex_line.get("vertex"))
            require(flex_line.get("type") == "cable", f"{prefix} flexible line must be a MuJoCo cable composite", failures)
            require(flex_line.get("initial") == "none", f"{prefix} flexible line must use explicit world vertices", failures)
            require(len(vertices) >= 21 and len(vertices) % 3 == 0, f"{prefix} flexible line must have at least 7 vertices", failures)
            if len(vertices) >= 9 and len(vertices) % 3 == 0:
                points = [vertices[index : index + 3] for index in range(0, len(vertices), 3)]
                start_xy = points[0][:2]
                end_xy = points[-1][:2]
                max_slack_xy = 0.0
                for index, point in enumerate(points[1:-1], start=1):
                    t = index / (len(points) - 1)
                    line_x = start_xy[0] + (end_xy[0] - start_xy[0]) * t
                    line_y = start_xy[1] + (end_xy[1] - start_xy[1]) * t
                    max_slack_xy = max(max_slack_xy, math.hypot(point[0] - line_x, point[1] - line_y))
                require(max_slack_xy >= 0.025, f"{prefix} flexible line must have visible slack, not a straight rod", failures)
            plugin = flex_line.find("plugin")
            require(
                plugin is not None and plugin.get("plugin") == "mujoco.elasticity.cable",
                f"{prefix} flexible line must use mujoco.elasticity.cable",
                failures,
            )
            if plugin is not None:
                configs = {str(config.get("key")): str(config.get("value")) for config in plugin.findall("config")}
                require(configs.get("twist") == "80", f"{prefix} flexible line twist must be soft, not rod-like", failures)
                require(configs.get("bend") == "1.2", f"{prefix} flexible line bend must be soft, not rod-like", failures)
                require(configs.get("vmax") == "0.30", f"{prefix} flexible line vmax must allow visible rope motion", failures)
            joint = flex_line.find("joint")
            require(joint is not None and joint.get("kind") == "main", f"{prefix} flexible line missing main joint", failures)
            if joint is not None:
                require(joint.get("damping") == "0.010", f"{prefix} flexible line joint damping must stay rope-like", failures)
                require(joint.get("armature") == "0.00002", f"{prefix} flexible line joint armature must stay low", failures)
            cable_geom = flex_line.find("geom")
            require(cable_geom is not None, f"{prefix} flexible line missing capsule geom", failures)
            if cable_geom is not None:
                require(cable_geom.get("type") == "capsule", f"{prefix} flexible line geom must be a capsule", failures)
                require(cable_geom.get("size") == "0.0100", f"{prefix} flexible line must be thick enough to see and catch", failures)
                require(cable_geom.get("rgba") == "1 1 1 1", f"{prefix} flexible line must be opaque white", failures)
                require(cable_geom.get("contype") == "0", f"{prefix} flexible line must avoid self-collision", failures)
                require(cable_geom.get("conaffinity") == "1", f"{prefix} flexible line must collide with robot/collector geoms", failures)
                require(cable_geom.get("condim") == "4", f"{prefix} flexible line must use frictional contacts", failures)
        bottom_connect = connects.get(f"{prefix}_flex_line_bottom_connect")
        top_weld = welds.get(f"{prefix}_flex_line_top_connect")
        require(bottom_connect is not None, f"{prefix} flexible line missing bottom connect", failures)
        require(top_weld is not None, f"{prefix} flexible line missing moving jig top weld", failures)
        if bottom_connect is not None:
            require(bottom_connect.get("body1") == f"{prefix}_flex_line_B_first", f"{prefix} bottom connect must attach first cable body", failures)
            require(bottom_connect.get("body2") == f"{prefix}_magnet_base", f"{prefix} bottom connect must attach to magnet base", failures)
        if top_weld is not None:
            require(top_weld.get("body1") == f"{prefix}_flex_line_B_last", f"{prefix} top weld must attach last cable body", failures)
            require(top_weld.get("body2") == f"{prefix}_float", f"{prefix} top weld must attach to the moving float underside jig", failures)
            require(top_weld.get("solref") == "0.004 1", f"{prefix} top weld must keep the cable end on the moving float underside jig", failures)
            require(top_weld.get("solimp") == "0.95 0.99 0.0005", f"{prefix} top weld must keep the cable end on the moving float underside jig", failures)
        require(pvc is not None, f"{base_name} missing visible PVC pipe", failures)
        require(magnet is not None, f"{prefix}_float missing moving visible magnet", failures)
        require(weight is not None, f"{base_name} missing diver weight", failures)
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
        tether_link = child_by_name(float_body, "geom", f"{prefix}_tether_socket_link")
        require(tether_link is not None, f"{prefix}_float missing moving visible tether socket link", failures)
        if tether_link is not None:
            require(tether_link.get("type") == "capsule", f"{prefix}_tether_socket_link must be a capsule", failures)
            if attach_site is not None:
                fromto = floats(tether_link.get("fromto"))
                attach_z = pos_z(attach_site)
                require(len(fromto) == 6, f"{prefix}_tether_socket_link must have a valid fromto", failures)
                if len(fromto) == 6:
                    require(math.isclose(fromto[2], attach_z - 0.031, abs_tol=1e-6), f"{prefix}_tether_socket_link must start below the attach site", failures)
                    require(math.isclose(fromto[5], attach_z - 0.008, abs_tol=1e-6), f"{prefix}_tether_socket_link must end below the attach site", failures)
            require(tether_link.get("size") == "0.0075", f"{prefix}_tether_socket_link must be thick enough to see", failures)
            require(tether_link.get("rgba") == "1 1 1 1", f"{prefix}_tether_socket_link must be white", failures)
            require(tether_link.get("contype") == "0", f"{prefix}_tether_socket_link must be visual-only", failures)
            require(tether_link.get("conaffinity") == "0", f"{prefix}_tether_socket_link must be visual-only", failures)
        tether_collar = child_by_name(float_body, "geom", f"{prefix}_tether_collar")
        require(tether_collar is not None, f"{prefix}_float missing moving visible tether collar", failures)
        if tether_collar is not None:
            require(tether_collar.get("type") == "cylinder", f"{prefix}_tether_collar must be a cylinder", failures)
            if attach_site is not None:
                collar_pos = floats(tether_collar.get("pos"))
                require(len(collar_pos) == 3 and math.isclose(collar_pos[2], pos_z(attach_site) - 0.013, abs_tol=1e-6), f"{prefix}_tether_collar must sit on the moving jig cable top", failures)
            require(tether_collar.get("contype") == "0", f"{prefix}_tether_collar must be visual-only", failures)
            require(tether_collar.get("conaffinity") == "0", f"{prefix}_tether_collar must be visual-only", failures)

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
