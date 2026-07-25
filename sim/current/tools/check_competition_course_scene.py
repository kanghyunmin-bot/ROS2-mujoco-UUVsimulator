#!/usr/bin/env python3
"""Validate the 2026 competition-course proxy in the MuJoCo tank scene."""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
COURSE_BUOY_FULL_IMMERSION_NET_LIFT_N = 1.0
COURSE_BUOY_FLOAT_HALF_HEIGHT_M = 0.085
COURSE_BUOY_MAX_VISIBLE_FLOAT_MASS_KG = 0.060
COURSE_BUOY_MIN_SURFACE_CENTER_Z_M = 0.045
COURSE_BUOY_COM_POS = (0.0, 0.0, -0.035)
COURSE_BUOY_COB_POS = (0.0, 0.0, 0.035)


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
    option = root.find("option")
    require(option is not None and option.get("integrator") == "Euler", "course dynamics require the stable Euler integrator", failures)
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom") if geom.get("name")}
    meshes = {mesh.get("name"): mesh for mesh in root.findall(".//mesh") if mesh.get("name")}
    cameras = {camera.get("name"): camera for camera in root.findall(".//camera") if camera.get("name")}
    bodies = body_map(root)
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
    base = bodies.get("base_link")
    require(base is not None, "missing base_link", failures)
    if base is not None:
        inertial = base.find("inertial")
        require(inertial is not None, "base_link must define physical inertia", failures)
        if inertial is not None:
            inertia = floats(inertial.get("diaginertia"))
            require(
                len(inertia) == 3 and min(inertia) >= 0.5,
                "15kg base_link inertia must match the vehicle dimensions, not a point mass",
                failures,
            )
    require("ping360_visual" not in geoms, "Ping360 sonar STL visual geom must stay removed", failures)
    require("ping360_body_no_cable" not in meshes, "Ping360 sonar STL mesh asset must stay removed", failures)
    collector_frame = geoms.get("collector_frame_visual")
    require(collector_frame is not None, "missing white trapezoidal collector STL", failures)
    if collector_frame is not None:
        require(collector_frame.get("mesh") == "front_open_buoy_collector_v9", "collector must use the v9 trapezoidal PVC frame", failures)
        require(collector_frame.get("contype") == "0" and collector_frame.get("conaffinity") == "0", "collector STL must remain visual-only", failures)
    collector_top = geoms.get("collector_top_net_proxy")
    require(collector_top is not None, "missing collector_top_net_proxy", failures)
    if collector_top is not None:
        require(
            math.isclose(pos_z(collector_top), 0.4050, abs_tol=1e-6)
            and collector_top.get("quat") == "0.99875 0 -0.04994 0",
            "collector roof must follow the trapezoidal PVC frame",
            failures,
        )
    for name in ("collector_left_net_proxy", "collector_right_net_proxy"):
        side = geoms.get(name)
        require(side is not None and side.get("mesh") == "front_open_buoy_collector_side_proxy_v1", f"{name} must use one non-overlapping trapezoid proxy", failures)
        if side is not None:
            rgba = floats(side.get("rgba"))
            require(len(rgba) == 4 and 0.08 <= rgba[3] <= 0.18, f"{name} must render as a visible transparent net wall", failures)
    for name in ("collector_back_net_proxy", "collector_bottom_net_proxy", "collector_top_net_proxy", "collector_front_net_flap"):
        panel = geoms.get(name)
        require(panel is not None, f"missing collector net panel {name}", failures)
        if panel is not None:
            rgba = floats(panel.get("rgba"))
            require(len(rgba) == 4 and 0.05 <= rgba[3] <= 0.18, f"{name} must stay translucent", failures)
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
        inertial = body.find("inertial")
        require(inertial is not None, f"{body_name} missing explicit inertia", failures)
        if inertial is not None:
            require(
                floats(inertial.get("pos")) == COURSE_BUOY_COM_POS,
                f"{body_name} CoM must stay 35mm below its body frame",
                failures,
            )
        com_site = child_by_name(body, "site", f"{prefix}_com_site")
        cob_site = child_by_name(body, "site", f"{prefix}_cob_site")
        require(com_site is not None, f"{body_name} missing explicit CoM site", failures)
        require(cob_site is not None, f"{body_name} missing explicit CoB site", failures)
        if com_site is not None:
            require(floats(com_site.get("pos")) == COURSE_BUOY_COM_POS, f"{prefix} CoM site is inconsistent with inertia", failures)
        if cob_site is not None:
            require(floats(cob_site.get("pos")) == COURSE_BUOY_COB_POS, f"{prefix} CoB must stay 70mm above its CoM", failures)
        initial_center_z = pos_z(body) + (pos_z(cob_site) if cob_site is not None else 0.0)
        require(abs(initial_center_z - target_z) < 1e-3, f"{body_name} must start at surface equilibrium z={target_z:.3f}, got {initial_center_z:.3f}", failures)
        require(
            mass_kg <= COURSE_BUOY_MAX_VISIBLE_FLOAT_MASS_KG,
                f"{body_name} mass must stay light enough for 1.0N net lift to visibly float; got {mass_kg:.3f}kg",
            failures,
        )
        require(
            target_z >= COURSE_BUOY_MIN_SURFACE_CENTER_Z_M,
            f"{body_name} 1.0N net-lift equilibrium must keep center visibly above water; got z={target_z:.3f}m",
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
            require(floats(float_geom.get("pos")) == (0.0, 0.0, 0.035), f"{prefix}_float_geom center must coincide with its CoB site", failures)
            require(float_geom.get("contype") == "2", f"{prefix}_float_geom must use buoy collision contype=2", failures)
            require(float_geom.get("conaffinity") == "1", f"{prefix}_float_geom must collide with vehicle/collector but not other buoys", failures)
        bands = [geom for geom in body.findall("geom") if "_equator_band_" in (geom.get("name") or "")]
        require(len(bands) == 8, f"{body_name} must have 8 equator-band segments", failures)


def check_fixed_buoys(root: ET.Element, bodies: dict[str, ET.Element], failures: list[str]) -> None:
    pvc_material = child_by_name(root.find("asset"), "material", "course_buoy_pvc")
    require(pvc_material is not None, "competition scene must define one uniform PVC material", failures)
    if pvc_material is not None:
        require(
            floats(pvc_material.get("rgba")) == (0.86, 0.90, 0.94, 1.0)
            and pvc_material.get("specular") == "0.35"
            and pvc_material.get("shininess") == "0.45",
            "PVC material must provide natural light/shadow contrast without painted bands",
            failures,
        )
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom") if geom.get("name")}
    composites = {
        composite.get("prefix"): composite
        for composite in root.findall("./worldbody/composite")
        if composite.get("prefix")
    }
    equalities = {
        equality.get("name"): equality
        for equality in root.findall("./equality/*")
        if equality.get("name")
    }
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

    rake_centers: dict[str, list[float]] = {}
    for side, expected_center in (("port", 0.160), ("starboard", -0.160)):
        tines = [geoms.get(f"mission_{side}_rake_tine_{index}") for index in range(1, 6)]
        require(all(tine is not None for tine in tines), f"{side} rake must have five physical tines", failures)
        if not all(tine is not None for tine in tines):
            continue
        centers = []
        for tine in tines:
            assert tine is not None
            fromto = floats(tine.get("fromto"))
            require(len(fromto) == 6, f"{tine.get('name')} must use explicit endpoints", failures)
            if len(fromto) == 6:
                centers.append(fromto[1])
                require(math.isclose(fromto[1], fromto[4], abs_tol=1e-6), f"{tine.get('name')} must point straight forward", failures)
                require(math.isclose(fromto[0], 0.300, abs_tol=1e-6) and math.isclose(fromto[3], 0.480, abs_tol=1e-6), f"{tine.get('name')} must be 200mm long including round ends", failures)
            require(tine.get("size") == "0.010", f"{tine.get('name')} must be 20mm wide", failures)
            require(tine.get("contype") == "1" and tine.get("conaffinity") == "2", f"{tine.get('name')} must catch the 13mm PVC stick", failures)
        centers.sort()
        rake_centers[side] = centers
        if len(centers) == 5:
            require(math.isclose(sum(centers) / 5.0, expected_center, abs_tol=1e-6), f"{side} rake group lateral position changed", failures)
            for left, right in zip(centers, centers[1:]):
                clear_gap = right - left - 2.0 * 0.010
                require(math.isclose(clear_gap, 0.020, abs_tol=1e-6), f"{side} rake clear gap must be 20mm, got {clear_gap:.3f}m", failures)
        root = geoms.get(f"mission_{side}_rake_root_probe")
        require(root is not None, f"{side} rake missing the rear stop bar", failures)
        if root is not None:
            require(root.get("size") == "0.010", f"{side} rake rear stop must match the 20mm tines", failures)
            require(root.get("contype") == "1" and root.get("conaffinity") == "2", f"{side} rake stop bar must physically release the buoy", failures)
    if len(rake_centers.get("port", [])) == 5 and len(rake_centers.get("starboard", [])) == 5:
        inner_clear_width = (
            min(rake_centers["port"]) - 0.010
            - (max(rake_centers["starboard"]) + 0.010)
        )
        require(
            math.isclose(inner_clear_width, 0.140, abs_tol=1e-6),
            f"left/right rake clear width must be 140mm, got {inner_clear_width:.3f}m",
            failures,
        )
    moored_core_signatures: list[tuple[object, ...]] = []
    for base_name in bases:
        base = bodies[base_name]
        prefix = base_name.removesuffix("_magnet_base")
        float_body = bodies.get(f"{prefix}_float")
        require(float_body is not None, f"{prefix} missing float body", failures)
        require(f"{prefix}_magnet_jig" not in bodies, f"{prefix} legacy dynamic magnet jig must be removed", failures)
        require(f"{prefix}_tether_jig" not in bodies, f"{prefix} legacy tether jig must be removed", failures)
        if float_body is None:
            continue

        require(math.isclose(pos_z(base), -11.0, abs_tol=1e-6), f"{base_name} must sit on 11m floor", failures)
        require(
            len(base.findall("joint")) == 0,
            f"{base_name} diver weight must remain fixed to the floor",
            failures,
        )
        for anchor_geom_name in (
            f"{prefix}_diver_weight",
            f"{prefix}_weight_eye",
            f"{prefix}_bottom_jig",
            f"{prefix}_mooring_rod",
            f"{prefix}_rod_top_jig",
            f"{prefix}_fixed_magnet_stem",
            f"{prefix}_fixed_magnet",
        ):
            require(
                child_by_name(base, "geom", anchor_geom_name) is not None,
                f"{base_name} missing fixed anchor component {anchor_geom_name}",
                failures,
            )
        require(math.isclose(pos_z(float_body), -8.535, abs_tol=1e-6), f"{prefix}_float body frame must place its CoB 2.5m above floor", failures)
        require(
            math.isclose(float_attr(float_body, "gravcomp"), 0.0, abs_tol=1e-9),
            f"{prefix}_float must not use unconditional gravcomp; runtime waterline buoyancy provides 1.0N net lift only underwater",
            failures,
        )
        mass_kg = inertial_mass(float_body)
        require(
            mass_kg <= COURSE_BUOY_MAX_VISIBLE_FLOAT_MASS_KG,
            f"{prefix}_float mass must stay light enough to float after magnet release; got {mass_kg:.3f}kg",
            failures,
        )
        inertial = float_body.find("inertial")
        require(inertial is not None, f"{prefix}_float missing explicit inertia", failures)
        if inertial is not None:
            require(
                floats(inertial.get("pos")) == COURSE_BUOY_COM_POS,
                f"{prefix}_float CoM must stay 35mm below its body frame",
                failures,
            )
        com_site = child_by_name(float_body, "site", f"{prefix}_com_site")
        cob_site = child_by_name(float_body, "site", f"{prefix}_cob_site")
        require(com_site is not None, f"{prefix}_float missing explicit CoM site", failures)
        require(cob_site is not None, f"{prefix}_float missing explicit CoB site", failures)
        if com_site is not None:
            require(floats(com_site.get("pos")) == COURSE_BUOY_COM_POS, f"{prefix} CoM site is inconsistent with inertia", failures)
        if cob_site is not None:
            require(floats(cob_site.get("pos")) == COURSE_BUOY_COB_POS, f"{prefix} CoB must stay 70mm above its CoM", failures)
        require(
            surface_float_center_z(mass_kg) >= COURSE_BUOY_MIN_SURFACE_CENTER_Z_M,
            f"{prefix}_float 1.0N net-lift equilibrium must keep center visibly above water after release; got z={surface_float_center_z(mass_kg):.3f}m",
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
        magnet_weld = equalities.get(f"{prefix}_magnet_weld")
        require(magnet_weld is not None and magnet_weld.tag == "weld", f"{prefix} missing rigid magnetic weld", failures)
        if magnet_weld is not None:
            require(magnet_weld.get("body1") == f"{prefix}_magnet_base", f"{prefix} magnet weld must use fixed rod body as body1", failures)
            require(magnet_weld.get("body2") == f"{prefix}_float", f"{prefix} magnet weld must use buoy as body2", failures)
            require(floats(magnet_weld.get("relpose")) == (0.0, 0.0, 2.465, 1.0, 0.0, 0.0, 0.0), f"{prefix} magnet weld relative pose changed", failures)
            require(magnet_weld.get("solref") == "0.020 1", f"{prefix} magnet weld must remain stable at the 8ms simulation step", failures)
            require(magnet_weld.get("solimp") == "0.95 0.99 0.0005", f"{prefix} magnet weld must hold both magnet faces together", failures)
        if magnet_site is not None and attach_site is not None:
            magnet_world_z = pos_z(base) + pos_z(magnet_site)
            attach_world_z = pos_z(float_body) + pos_z(attach_site)
            require(
                math.isclose(magnet_world_z, attach_world_z, abs_tol=1e-6),
                f"{prefix} magnet and attach sites do not coincide: {magnet_world_z:.3f} vs {attach_world_z:.3f}",
                failures,
            )

        pvc = child_by_name(float_body, "geom", f"{prefix}_pvc_pipe")
        magnet = child_by_name(float_body, "geom", f"{prefix}_magnet")
        weight = child_by_name(base, "geom", f"{prefix}_diver_weight")
        rod = child_by_name(base, "geom", f"{prefix}_mooring_rod")
        rod_top_jig = child_by_name(base, "geom", f"{prefix}_rod_top_jig")
        require(composites.get(f"{prefix}_flex_line_") is None, f"{prefix} elasticity cable must be removed", failures)
        require(equalities.get(f"{prefix}_flex_line_bottom_connect") is None, f"{prefix} cable bottom equality must be removed", failures)
        require(equalities.get(f"{prefix}_flex_line_top_connect") is None, f"{prefix} cable top equality must be removed", failures)
        require(rod is not None, f"{base_name} missing fixed mooring rod", failures)
        require(rod_top_jig is not None, f"{base_name} missing fixed rod top jig", failures)
        if rod is not None:
            endpoints = floats(rod.get("fromto"))
            require(rod.get("type") == "capsule", f"{prefix} mooring rod must be a capsule", failures)
            require(rod.get("size") == "0.0050", f"{prefix} mooring rod must be 10mm diameter", failures)
            require(rod.get("rgba") == "0.82 0.84 0.86 1", f"{prefix} mooring rod must be visible light gray", failures)
            require(rod.get("density") == "0", f"{prefix} fixed mooring rod must not add dynamic mass", failures)
            require(rod.get("contype") == "0" and rod.get("conaffinity") == "0", f"{prefix} fixed rod must be visual-only", failures)
            require(len(endpoints) == 6, f"{prefix} mooring rod must use explicit endpoints", failures)
            if len(endpoints) == 6:
                require(
                    endpoints[:2] == (0.0, 0.0) and endpoints[3:5] == (0.0, 0.0),
                    f"{prefix} mooring rod must remain vertical",
                    failures,
                )
                require(
                    math.isclose(endpoints[2], 0.109, abs_tol=1e-6)
                    and math.isclose(endpoints[5], 2.204, abs_tol=1e-6),
                    f"{prefix} mooring rod endpoints changed: {endpoints}",
                    failures,
                )
        bottom_jig = child_by_name(base, "geom", f"{prefix}_bottom_jig")
        if rod is not None and bottom_jig is not None:
            endpoints = floats(rod.get("fromto"))
            jig_pos = floats(bottom_jig.get("pos"))
            jig_size = floats(bottom_jig.get("size"))
            require(len(endpoints) == 6 and math.isclose(endpoints[2], jig_pos[2] + jig_size[1], abs_tol=1e-6), f"{prefix} rod bottom must meet the lower jig top", failures)
        if rod is not None and rod_top_jig is not None:
            endpoints = floats(rod.get("fromto"))
            jig_pos = floats(rod_top_jig.get("pos"))
            jig_size = floats(rod_top_jig.get("size"))
            require(len(endpoints) == 6 and math.isclose(endpoints[5], jig_pos[2] - jig_size[1], abs_tol=1e-6), f"{prefix} rod top must meet the fixed jig underside", failures)
        require(pvc is not None, f"{prefix}_float missing moving PVC stick", failures)
        require(magnet is not None, f"{prefix}_float missing moving visible magnet", failures)
        require(weight is not None, f"{base_name} missing diver weight", failures)
        if pvc is not None:
            fromto = floats(pvc.get("fromto"))
            require(len(fromto) == 6, f"{prefix} PVC stick must use explicit endpoints", failures)
            if len(fromto) == 6:
                require(
                    math.isclose(abs(fromto[5] - fromto[2]), 0.098, abs_tol=1e-6),
                    f"{prefix} PVC stick must make the complete assembly exactly 400mm",
                    failures,
                )
            require(pvc.get("size") == "0.0065", f"{prefix} PVC stick must use the 13mm outside diameter", failures)
            require(pvc.get("material") == "course_buoy_pvc", f"{prefix} PVC must use the uniform shaded material", failures)
            require(pvc.get("contype") == "2", f"{prefix} PVC stick must be physically catchable by the rake", failures)
            require(pvc.get("conaffinity") == "1", f"{prefix} PVC stick must collide only with the vehicle/rake", failures)
        if magnet is not None:
            require(magnet.get("contype") == "0", f"{prefix} magnet must be visual-only while weld handles attachment", failures)
            require(
                magnet.get("conaffinity") == "0",
                f"{prefix} magnet must be visual-only while weld handles attachment",
                failures,
            )

        bands = [geom for geom in float_body.findall("geom") if "_equator_band_" in (geom.get("name") or "")]
        require(len(bands) == 8, f"{prefix}_float must have 8 equator-band segments", failures)
        top_jig = child_by_name(float_body, "geom", f"{prefix}_top_jig")
        require(top_jig is not None, f"{prefix}_float missing top jig", failures)
        require(child_by_name(float_body, "geom", f"{prefix}_bottom_socket") is not None, f"{prefix}_float missing bottom socket", failures)
        if top_jig is not None and attach_site is not None:
            top_pos = floats(top_jig.get("pos"))
            top_size = floats(top_jig.get("size"))
            assembly_height = top_pos[2] + top_size[1] - pos_z(attach_site)
            require(
                math.isclose(assembly_height, 0.400, abs_tol=1e-6),
                f"{prefix} buoy/stick/jig assembly must be exactly 400mm, got {assembly_height:.3f}m",
                failures,
            )
        moving_jig_name = (
            f"{prefix}_pinger_bottom_jig"
            if "_pinger_white_" in prefix
            else f"{prefix}_moving_lower_jig"
        )
        require(
            child_by_name(float_body, "geom", moving_jig_name) is not None,
            f"{prefix}_float missing lower jig at the end of the moving PVC stick",
            failures,
        )
        require(
            rod_top_jig is not None,
            f"{base_name} missing fixed rod top jig",
            failures,
        )
        require(
            child_by_name(float_body, "geom", f"{prefix}_tether_socket_link") is None,
            f"{prefix} old float-side rope link must be removed",
            failures,
        )

        moving_jig = child_by_name(float_body, "geom", moving_jig_name)
        bottom_socket = child_by_name(float_body, "geom", f"{prefix}_bottom_socket")
        magnet_plate = child_by_name(float_body, "geom", f"{prefix}_magnet_lower_plate")
        if all(item is not None for item in (float_geom, bottom_socket, pvc, moving_jig, magnet_plate, magnet, attach_site)):
            assert float_geom is not None and bottom_socket is not None and pvc is not None
            assert moving_jig is not None and magnet_plate is not None and magnet is not None and attach_site is not None
            moored_core_signatures.append(
                (
                    float_geom.get("type"), float_geom.get("size"),
                    bottom_socket.get("pos"), bottom_socket.get("size"),
                    pvc.get("fromto"), pvc.get("size"),
                    moving_jig.get("pos"), moving_jig.get("size"),
                    magnet_plate.get("pos"), magnet_plate.get("size"),
                    magnet.get("pos"), magnet.get("size"),
                    attach_site.get("pos"),
                    tuple(band.get("size") for band in bands),
                )
            )

    require(len(moored_core_signatures) == 15, "all 15 moored buoy cores must expose the common geometry contract", failures)
    require(len(set(moored_core_signatures)) == 1, "yellow/orange/pinger Chupa-Chups buoy cores must use identical dimensions", failures)

    pinger = bodies.get("course_buoy_pinger_white_1_float")
    require(pinger is not None, "missing pinger white float", failures)
    if pinger is not None:
        for geom_name in ["course_buoy_pinger_white_1_pinger_body", "course_buoy_pinger_white_1_pinger_bottom_jig"]:
            require(child_by_name(pinger, "geom", geom_name) is not None, f"pinger missing {geom_name}", failures)
        acoustic_site = child_by_name(pinger, "site", "course_buoy_pinger_white_1_acoustic_site")
        require(acoustic_site is not None, "pinger missing acoustic transducer site", failures)
        if acoustic_site is not None:
            require(
                acoustic_site.get("pos") == "0 0 -0.11",
                "pinger acoustic source must be centered in the blue transducer body",
                failures,
            )
        for geom_name in ["course_buoy_pinger_white_1_pinger_left_rod", "course_buoy_pinger_white_1_pinger_right_rod"]:
            require(child_by_name(pinger, "geom", geom_name) is None, f"pinger must not add a second stick: {geom_name}", failures)


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
