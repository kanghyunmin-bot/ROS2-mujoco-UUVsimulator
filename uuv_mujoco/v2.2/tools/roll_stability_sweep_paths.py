"""Path records for roll stability sweeps."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SweepPaths:
    root: Path
    profile_path: Path
    scene_path: Path
    mapping_path: Path
    start_script: Path
    reset_script: Path
    log_root: Path


def default_sweep_paths(script_path: Path) -> SweepPaths:
    root = script_path.resolve().parents[1]
    return SweepPaths(
        root=root,
        profile_path=root / "config" / "sim_profiles.json",
        scene_path=root / "scenes" / "tank_current_scene.xml",
        mapping_path=root / "physics" / "thruster_mapping.py",
        start_script=root / "start_sitl_mujoco_mj311.sh",
        reset_script=root / "reset_uuv_sim.sh",
        log_root=root / "logs",
    )


__all__ = ["SweepPaths", "default_sweep_paths"]
