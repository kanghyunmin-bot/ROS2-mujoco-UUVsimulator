#!/usr/bin/env python3
"""Dependency-light contract check for the SLAM research-pool scene."""

from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "research_pool_slam_scene.xml"
PROFILE = ROOT / "config" / "sim_profiles.json"


def _numbers(value: str | None) -> tuple[float, ...]:
    return tuple(float(item) for item in str(value or "").split())


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    root = ET.parse(SCENE).getroot()
    geoms = {item.get("name"): item for item in root.findall(".//geom") if item.get("name")}
    bodies = {item.get("name"): item for item in root.findall(".//body") if item.get("name")}
    sites = {item.get("name"): item for item in root.findall(".//site") if item.get("name")}

    floor = geoms.get("pool_floor")
    _require(floor is not None, "pool_floor is missing")
    _require(_numbers(floor.get("size")) == (12.5, 6.25, 0.05), "pool must be 25m x 12.5m")
    _require(_numbers(floor.get("pos"))[2] == -3.05, "pool collision floor must define 3m depth")
    water = geoms.get("water_vis")
    _require(water is not None and water.get("group") == "5", "water visual must be non-physics group 5")
    _require(water.get("contype") == "0" and water.get("conaffinity") == "0", "water visual must not collide")

    for name in (
        "pool_lane_left",
        "pool_lane_center",
        "pool_lane_right",
        "slam_landmark_red_panel",
        "slam_landmark_yellow_panel",
        "slam_landmark_column",
        "slam_landmark_low_block",
        "slam_gate_top",
    ):
        _require(name in geoms, f"missing economical SLAM feature: {name}")
    _require(not any(name.startswith("course_buoy_") for name in bodies), "research pool must not load competition buoys")
    _require(len(geoms) <= 50, f"research pool geometry budget exceeded: {len(geoms)}")

    _require("base_link" in bodies, "base_link vehicle is missing")
    for name in ("fluid_center_enclosure", "fluid_port_lower_body", "fluid_starboard_lower_body"):
        geom = geoms.get(name)
        _require(geom is not None and geom.get("fluidshape") == "ellipsoid", f"missing fluid proxy {name}")
    for name in (
        "imu_site",
        "bar30_site",
        "dvl_site",
        "ping360_site",
        "cam_left_site",
        "cam_right_site",
        "hydrophone_center_site",
        "hydrophone_left_site",
        "hydrophone_right_site",
        "course_buoy_pinger_white_1_acoustic_site",
    ):
        _require(name in sites, f"missing runtime site {name}")

    profiles = json.loads(PROFILE.read_text())
    payload = profiles["research_pool"]
    _require(payload.get("extends") == "current", "research profile must inherit current plant")
    current = payload["current_field"]
    scaling = payload["hydrodynamic_state_scaling"]
    _require(current["active"] is True, "research current field must be explicitly active")
    _require(scaling["active"] is True, "research coefficient scaling must be explicitly active")
    _require("uncalibrated" in current["calibration_status"], "current prior must declare uncalibrated status")
    _require("uncalibrated" in scaling["calibration_status"], "coefficient prior must declare uncalibrated status")
    _require(float(current["max_speed_mps"]) <= 0.10, "pool current safety bound is too high")

    distributed = profiles["research_pool_distributed"]
    _require(
        distributed.get("extends") == "research_pool",
        "distributed profile must inherit the research pool",
    )
    patches = distributed["distributed_hydrodynamics"]["patches"]
    _require(
        len(patches) == 33,
        "distributed profile must expose 33 auditable patch definitions",
    )
    expanded_count = sum(
        2 ** len(item.get("quadrature_spans_body_m", []))
        for item in patches
    )
    _require(
        expanded_count == 105,
        "distributed profile must expand to 105 local force samples",
    )
    volume_m3 = sum(float(item["volume_share_m3"]) for item in patches)
    _require(abs(volume_m3 - 0.015008) <= 1.0e-12, "patch volume must match the 15.008 kg neutral plant")
    _require(distributed["free_surface"]["mode"] == "flat", "distributed baseline must use a flat surface")
    _require(distributed["thruster_inflow"]["active"] is True, "distributed profile must enable local inflow")
    _require(
        distributed["hydrodynamic_matrices"]["active"] is True,
        "distributed profile must enable the added-mass matrix",
    )

    hybrid = profiles["research_pool_distributed_hybrid"]
    _require(
        hybrid.get("extends") == "research_pool_distributed",
        "hybrid profile must preserve the distributed baseline",
    )
    calibration = hybrid["distributed_hydrodynamics_calibration"]
    _require(
        calibration["calibration_status"] == "stonefish_ellipsoid_hybrid_prior",
        "hybrid profile must identify its non-measured calibration status",
    )
    _require(
        calibration["buoyancy_position_offset_body_m"] == [0.0, 0.0, -0.0092608788],
        "hybrid profile must preserve the derived CoB offset",
    )
    _require(
        calibration["normal_drag_axis_scale"] == [3.1392846302, 1.8974781726, 2.0601653105],
        "hybrid profile must preserve the derived tow-test scales",
    )

    print(
        "research_pool_scene=PASS "
        f"geoms={len(geoms)} dimensions=25.0x12.5x3.0m "
        "calibration=uncalibrated_pool_prior"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
