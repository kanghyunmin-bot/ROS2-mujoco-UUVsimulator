# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Keep map-local vehicle copies aligned with the Research pool vehicle."""
from __future__ import annotations

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

CANONICAL_SCENE = Path(__file__).resolve().parents[1] / "scenes/research_pool_slam_scene.xml"
SPAWN_ATTRIBUTES = ("pos", "quat", "euler", "axisangle", "xyaxes", "zaxis")


def vehicle_body(root: ET.Element) -> ET.Element:
    """Return the single top-level vehicle definition."""
    body = root.find("worldbody/body[@name='base_link']")
    if body is None:
        raise ValueError("Scene has no top-level base_link vehicle")
    return body


def vehicle_assets(root: ET.Element) -> list[ET.Element]:
    """Return the transitive mesh, material and texture dependencies."""
    assets = root.find("asset")
    if assets is None:
        raise ValueError("Scene has no assets")
    index = {(x.tag, x.get("name")): x for x in assets}
    pending = [(kind, x.get(kind)) for x in vehicle_body(root).iter()
               for kind in ("mesh", "material") if x.get(kind)]
    found = {}
    while pending:
        key = pending.pop()
        if key in found:
            continue
        if key not in index:
            raise ValueError(f"Missing vehicle asset: {key}")
        asset = index[key]
        found[key] = asset
        if asset.get("texture"):
            pending.append(("texture", asset.get("texture")))
    return [x for x in assets if (x.tag, x.get("name")) in found]


def synchronize_vehicle(root: ET.Element, source: ET.Element | None = None) -> None:
    """Copy canonical vehicle assets, body, actuators and sensors in place.

    Preserve the map's spawn pose and all environment bodies, settings and
    appearance. Maps remain self-contained for existing XML layout editors.
    This does not select hydrodynamic profiles or recalibrate their coefficients.
    """
    if source is None:
        source = ET.parse(CANONICAL_SCENE).getroot()
    old = vehicle_body(root)
    body = copy.deepcopy(vehicle_body(source))
    for key in SPAWN_ATTRIBUTES:
        body.attrib.pop(key, None)
        if key in old.attrib:
            body.set(key, old.get(key))
    world = root.find("worldbody")
    index = list(world).index(old)
    world.remove(old)
    world.insert(index, body)
    # The canonical CAD hand replaces the old net collector. Its inactive
    # capture welds must not point at a body that no longer exists.
    equality = root.find("equality")
    if equality is not None and body.find(".//body[@name='front_open_buoy_collector']") is None:
        for item in list(equality):
            if item.get("body1") == "front_open_buoy_collector" and item.get("name", "").endswith("_collector_weld"):
                if item.get("active") != "false":
                    raise ValueError("Refusing to remove an active legacy collector weld")
                equality.remove(item)
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    for asset in vehicle_assets(source):
        existing = assets.find(f"{asset.tag}[@name='{asset.get('name')}']")
        if existing is not None:
            assets.remove(existing)
        assets.append(copy.deepcopy(asset))
    # All actuators and sensors in the canonical scene belong to the vehicle.
    # Replace matching names but preserve any map-specific sensor or actuator.
    for tag in ("actuator", "sensor"):
        canonical = source.find(tag)
        if canonical is None:
            continue
        section = root.find(tag)
        if section is None:
            section = ET.SubElement(root, tag)
        for item in canonical:
            for existing in list(section):
                if existing.get("name") == item.get("name"):
                    section.remove(existing)
            section.append(copy.deepcopy(item))


def vehicle_signature(root: ET.Element) -> tuple:
    """Return a whitespace-independent definition excluding map spawn pose."""
    def signature(node: ET.Element) -> tuple:
        return (node.tag, tuple(sorted(node.attrib.items())), tuple(signature(x) for x in node))
    body = copy.deepcopy(vehicle_body(root))
    for key in SPAWN_ATTRIBUTES:
        body.attrib.pop(key, None)
    devices = tuple(
        (tag, tuple(sorted(signature(x) for x in root.findall(f"{tag}/*"))))
        for tag in ("actuator", "sensor")
    )
    return (signature(body), tuple(sorted(signature(x) for x in vehicle_assets(root))), devices)
