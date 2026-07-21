"""Temporary candidate file edits for roll stability sweeps."""

from __future__ import annotations

import json
import re
from pathlib import Path

from roll_stability_candidates import Candidate


FLUID_GEOMS = (
    "fluid_center_enclosure",
    "fluid_port_lower_body",
    "fluid_starboard_lower_body",
)


def write_profile(profile_path: Path, original_profile_text: str, candidate: Candidate) -> None:
    profiles = json.loads(original_profile_text)
    current = dict(profiles["current"])
    current.update(candidate.profile_updates)
    profiles["current"] = current
    profile_path.write_text(json.dumps(profiles, indent=2, ensure_ascii=False) + "\n")


def scale_fluid_angular(scene_text: str, scale: float | None) -> str:
    if scale is None:
        return scene_text
    updated = scene_text
    for geom_name in FLUID_GEOMS:
        pattern = re.compile(
            rf'(<geom\b(?=[^>]*\bname="{re.escape(geom_name)}")[^>]*\bfluidcoef=")([^"]+)(")',
            re.DOTALL,
        )

        def repl(match: re.Match[str]) -> str:
            values = [float(part) for part in match.group(2).split()]
            if len(values) < 3:
                raise RuntimeError(f"{geom_name} fluidcoef has fewer than 3 values")
            values[2] *= float(scale)
            return match.group(1) + " ".join(f"{value:.6g}" for value in values) + match.group(3)

        updated, count = pattern.subn(repl, updated, count=1)
        if count != 1:
            raise RuntimeError(f"Could not find fluidcoef for {geom_name}")
    return updated


def write_scene(scene_path: Path, original_scene_text: str, candidate: Candidate) -> None:
    scene_path.write_text(scale_fluid_angular(original_scene_text, candidate.fluid_angular_scale))


def write_mapping(mapping_path: Path, original_mapping_text: str, candidate: Candidate) -> None:
    if candidate.servo_signs is None:
        mapping_path.write_text(original_mapping_text)
        return
    replacement = "ARDUSUB_VECTORED_6DOF_SERVO_SIGNS = (" + ", ".join(str(v) for v in candidate.servo_signs) + ")"
    updated, count = re.subn(
        r"ARDUSUB_VECTORED_6DOF_SERVO_SIGNS\s*=\s*\([^)]+\)",
        replacement,
        original_mapping_text,
        count=1,
    )
    if count != 1:
        raise RuntimeError("Could not replace ARDUSUB_VECTORED_6DOF_SERVO_SIGNS")
    mapping_path.write_text(updated)


def apply_candidate(
    *,
    profile_path: Path,
    scene_path: Path,
    mapping_path: Path,
    original_profile_text: str,
    original_scene_text: str,
    original_mapping_text: str,
    candidate: Candidate,
) -> None:
    write_profile(profile_path, original_profile_text, candidate)
    write_scene(scene_path, original_scene_text, candidate)
    write_mapping(mapping_path, original_mapping_text, candidate)


def restore_files(
    *,
    profile_path: Path,
    scene_path: Path,
    mapping_path: Path,
    original_profile_text: str,
    original_scene_text: str,
    original_mapping_text: str,
) -> None:
    profile_path.write_text(original_profile_text)
    scene_path.write_text(original_scene_text)
    mapping_path.write_text(original_mapping_text)


__all__ = ["apply_candidate", "restore_files", "scale_fluid_angular"]
