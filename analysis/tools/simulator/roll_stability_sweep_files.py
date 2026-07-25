"""Original-file snapshot helpers for roll stability sweeps."""

from __future__ import annotations

from dataclasses import dataclass

from roll_stability_file_edits import restore_files
from roll_stability_sweep_paths import SweepPaths


@dataclass(frozen=True)
class OriginalFileTexts:
    profile_text: str
    scene_text: str
    mapping_text: str


def read_original_file_texts(paths: SweepPaths) -> OriginalFileTexts:
    return OriginalFileTexts(
        profile_text=paths.profile_path.read_text(),
        scene_text=paths.scene_path.read_text(),
        mapping_text=paths.mapping_path.read_text(),
    )


def restore_original_files(paths: SweepPaths, texts: OriginalFileTexts) -> None:
    restore_files(
        profile_path=paths.profile_path,
        scene_path=paths.scene_path,
        mapping_path=paths.mapping_path,
        original_profile_text=texts.profile_text,
        original_scene_text=texts.scene_text,
        original_mapping_text=texts.mapping_text,
    )


__all__ = ["OriginalFileTexts", "read_original_file_texts", "restore_original_files"]
