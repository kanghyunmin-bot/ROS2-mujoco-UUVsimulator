#!/usr/bin/env python3
"""Check what the measured trim observations do and do not identify."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.sim_launch_preset import _load_resolved_profiles  # noqa: E402
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics  # noqa: E402


OBSERVATIONS = ROOT / "config" / "vehicle_hydrostatic_observations.json"
PROFILES = ROOT / "config" / "sim_profiles.json"


def _component_com(profile: dict) -> tuple[float, np.ndarray]:
    components = profile.get("body_components")
    if not isinstance(components, list) or not components:
        raise ValueError("profile needs body_components")
    masses = np.asarray([item["mass"] for item in components], dtype=np.float64)
    positions = np.asarray([item["mass_pos"] for item in components], dtype=np.float64)
    if np.any(masses < 0.0) or not np.all(np.isfinite(masses)):
        raise ValueError("body component masses must be finite and nonnegative")
    total = float(np.sum(masses))
    if total <= 0.0 or positions.shape != (len(components), 3):
        raise ValueError("invalid body component mass distribution")
    return total, np.sum(masses[:, None] * positions, axis=0) / total


def evaluate_profile(profile: dict, measured_mass: float, density: float) -> dict:
    model = DistributedHullHydrodynamics.from_profile(profile)
    cfg = model.config
    mass, centre_of_mass = _component_com(profile)
    volume = float(np.sum(cfg.volume_shares_m3))
    centre_of_buoyancy = np.sum(
        cfg.positions_body_m * cfg.volume_shares_m3[:, None], axis=0
    ) / volume
    gravity = float(cfg.gravity_mps2)
    buoyancy_n = density * gravity * volume
    weight_n = measured_mass * gravity
    horizontal_offset = centre_of_buoyancy[:2] - centre_of_mass[:2]
    hydrostatic_stiffness = buoyancy_n * (centre_of_buoyancy[2] - centre_of_mass[2])
    residual = cfg.residual_restoring_stiffness_nm_per_rad
    return {
        "profile_mass_kg": mass,
        "configured_displaced_volume_m3": volume,
        "neutral_volume_from_measured_mass_m3": measured_mass / density,
        "net_upward_force_at_full_submergence_n": buoyancy_n - weight_n,
        "centre_of_mass_body_m": centre_of_mass.tolist(),
        "equivalent_centre_of_buoyancy_body_m": centre_of_buoyancy.tolist(),
        "horizontal_cob_minus_com_m": horizontal_offset.tolist(),
        "vertical_cob_minus_com_m": float(centre_of_buoyancy[2] - centre_of_mass[2]),
        "geometric_restoring_stiffness_nm_per_rad": [
            hydrostatic_stiffness,
            hydrostatic_stiffness,
        ],
        "additional_residual_restoring_nm_per_rad": residual[:2].tolist(),
        "total_small_angle_restoring_nm_per_rad": (
            hydrostatic_stiffness + residual[:2]
        ).tolist(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    observations = json.loads(OBSERVATIONS.read_text(encoding="utf-8"))
    measured_mass = float(observations["measured_total_mass_kg"])
    density = float(observations["water_density_kg_m3"])
    if measured_mass <= 0.0 or density <= 0.0:
        raise ValueError("measured mass and water density must be positive")
    profiles = _load_resolved_profiles(PROFILES)
    results = {
        name: evaluate_profile(profiles[name], measured_mass, density)
        for name in ("research_pool_distributed", "research_pool_distributed_hybrid")
    }
    baseline = results["research_pool_distributed"]
    if abs(float(baseline["profile_mass_kg"]) - measured_mass) > 1.0e-9:
        raise AssertionError("profile mass differs from measured total mass")
    if abs(float(baseline["net_upward_force_at_full_submergence_n"])) > 0.2:
        raise AssertionError("baseline is not near neutral for the measured mass")
    if np.linalg.norm(baseline["horizontal_cob_minus_com_m"]) > 1.0e-3:
        raise AssertionError("baseline does not represent level horizontal trim")
    report = {
        "observations": observations,
        "results": results,
        "conclusions": [
            "Measured 15 kg mass and the configured 0.015008 m^3 volume agree within 8 mL.",
            "Configured horizontal centre of buoyancy is aligned with the centre of mass, consistent with level trim.",
            "Manual level/neutral trim does not identify vertical CoB-CoM separation or roll/pitch restoring stiffness.",
            "Do not fit floodable-envelope volume or individual lead/XPS locations without measurements.",
            "Hybrid residual restoring is an independent virtual prior, not supported by the trim observation.",
        ],
    }
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, ensure_ascii=False, indent=2))
    print("vehicle_hydrostatic_observations=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
