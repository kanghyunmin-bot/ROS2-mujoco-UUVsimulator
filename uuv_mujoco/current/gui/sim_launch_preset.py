"""Validated scene and water-physics presets for GUI simulator launches."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

from .config_paths import (
    COURSE_SCENE_PATH,
    PHYSICS_PROFILE_PATH,
    RESEARCH_POOL_SCENE_PATH,
    SIM_STACK_DIR,
)


COURSE_CURRENT_PRESET_ID = "course_current"
RESEARCH_POOL_CURRENT_PRESET_ID = "research_pool_current"
RESEARCH_POOL_DISTRIBUTED_PRESET_ID = "research_pool_distributed"
RESEARCH_POOL_DISTRIBUTED_HYBRID_PRESET_ID = "research_pool_distributed_hybrid"
RESEARCH_POOL_DISTRIBUTED_WAVES_PRESET_ID = "research_pool_distributed_waves"
DEFAULT_SIM_LAUNCH_PRESET_ID = COURSE_CURRENT_PRESET_ID


@dataclass(frozen=True)
class SimLaunchPreset:
    """One atomic GUI selection for a scene and its compatible fluid model."""

    preset_id: str
    label: str
    description: str
    scene_path: Path
    profile: str
    fluid_model: str
    viewer_camera_mode: str
    uses_active_course_scene: bool = False


SIM_LAUNCH_PRESETS = (
    SimLaunchPreset(
        preset_id=COURSE_CURRENT_PRESET_ID,
        label="Course / test tank · ellipsoid",
        description="Existing competition or test-tank scene with the accepted current profile.",
        scene_path=COURSE_SCENE_PATH,
        profile="current",
        fluid_model="current",
        viewer_camera_mode="course_side",
        uses_active_course_scene=True,
    ),
    SimLaunchPreset(
        preset_id=RESEARCH_POOL_CURRENT_PRESET_ID,
        label="Research pool · ellipsoid baseline",
        description="SLAM pool using the previous whole-body ellipsoid fluid model.",
        scene_path=RESEARCH_POOL_SCENE_PATH,
        profile="research_pool",
        fluid_model="current",
        viewer_camera_mode="follow",
    ),
    SimLaunchPreset(
        preset_id=RESEARCH_POOL_DISTRIBUTED_PRESET_ID,
        label="Research pool · distributed physics",
        description="Flat surface with distributed buoyancy, drag, added mass, current, and thruster inflow.",
        scene_path=RESEARCH_POOL_SCENE_PATH,
        profile="research_pool_distributed",
        fluid_model="distributed",
        viewer_camera_mode="follow",
    ),
    SimLaunchPreset(
        preset_id=RESEARCH_POOL_DISTRIBUTED_HYBRID_PRESET_ID,
        label="Research pool · hybrid damping prior",
        description="Distributed hybrid with geometry-derived low-speed damping prior; not vehicle calibrated.",
        scene_path=RESEARCH_POOL_SCENE_PATH,
        profile="research_pool_distributed_hybrid",
        fluid_model="distributed",
        viewer_camera_mode="follow",
    ),
    SimLaunchPreset(
        preset_id=RESEARCH_POOL_DISTRIBUTED_WAVES_PRESET_ID,
        label="Research pool · distributed + waves",
        description="Distributed pool plant plus the deterministic bounded surface-wave case.",
        scene_path=RESEARCH_POOL_SCENE_PATH,
        profile="research_pool_distributed_waves",
        fluid_model="distributed",
        viewer_camera_mode="follow",
    ),
)

_PRESETS_BY_ID = {preset.preset_id: preset for preset in SIM_LAUNCH_PRESETS}
_PRESETS_BY_LABEL = {preset.label: preset for preset in SIM_LAUNCH_PRESETS}


def sim_launch_preset_ids() -> tuple[str, ...]:
    """Return stable preset identifiers accepted by the GUI and CLI."""

    return tuple(preset.preset_id for preset in SIM_LAUNCH_PRESETS)


def sim_launch_preset_labels() -> tuple[str, ...]:
    """Return human-readable labels in GUI display order."""

    return tuple(preset.label for preset in SIM_LAUNCH_PRESETS)


def default_sim_launch_preset_id(env: Mapping[str, str] | None = None) -> str:
    """Resolve the optional GUI default without accepting arbitrary launch args."""

    source = os.environ if env is None else env
    raw = str(source.get("UUV_GUI_SIM_PRESET", DEFAULT_SIM_LAUNCH_PRESET_ID)).strip()
    return raw if raw in _PRESETS_BY_ID else DEFAULT_SIM_LAUNCH_PRESET_ID


def resolve_sim_launch_preset(
    value: object | None,
    *,
    default_id: str | None = None,
) -> SimLaunchPreset:
    """Resolve a preset identifier or displayed label.

    Args:
        value: Stable preset identifier or a GUI label.
        default_id: Preset used only when :paramref:`value` is empty.

    Returns:
        The matching immutable launch preset.

    Raises:
        ValueError: If the selection is not one of the known presets.
    """

    text = str(value or "").strip()
    if not text:
        text = default_id or default_sim_launch_preset_id()
    preset = _PRESETS_BY_ID.get(text) or _PRESETS_BY_LABEL.get(text)
    if preset is None:
        raise ValueError(f"unsupported simulator preset: {text}")
    return preset


def sim_launch_presets_payload() -> list[dict[str, object]]:
    """Return browser-safe metadata for all selectable simulator presets."""

    return [
        {
            "id": preset.preset_id,
            "label": preset.label,
            "description": preset.description,
            "scene": str(preset.scene_path),
            "profile": preset.profile,
            "fluid_model": preset.fluid_model,
            "viewer_camera_mode": preset.viewer_camera_mode,
            "uses_active_course_scene": preset.uses_active_course_scene,
        }
        for preset in SIM_LAUNCH_PRESETS
    ]


def validate_sim_launch_preset(
    preset: SimLaunchPreset,
    *,
    profile_path: Path = PHYSICS_PROFILE_PATH,
    sim_stack_dir: Path = SIM_STACK_DIR,
) -> None:
    """Validate that a preset still points at compatible local assets."""

    scenes_root = (Path(sim_stack_dir) / "scenes").resolve()
    scene_path = Path(preset.scene_path).resolve()
    try:
        scene_path.relative_to(scenes_root)
    except ValueError as exc:
        raise ValueError(f"simulator preset scene escapes scenes directory: {scene_path}") from exc
    if not scene_path.is_file():
        raise ValueError(f"simulator preset scene is missing: {scene_path}")

    profiles = _load_resolved_profiles(profile_path)
    profile = profiles.get(preset.profile)
    if profile is None:
        raise ValueError(f"simulator preset profile is missing: {preset.profile}")
    distributed = profile.get("distributed_hydrodynamics")
    distributed_active = isinstance(distributed, dict) and distributed.get("active") is True
    if preset.fluid_model == "distributed" and not distributed_active:
        raise ValueError(
            f"simulator preset {preset.preset_id} requires active distributed_hydrodynamics"
        )
    if preset.fluid_model != "distributed" and distributed_active:
        raise ValueError(
            f"simulator preset {preset.preset_id} would duplicate an active distributed plant"
        )


def build_sim_launch_preset_args(
    preset: SimLaunchPreset,
    *,
    scene_path: Path | None = None,
) -> list[str]:
    """Build the complete scene/profile/fluid argument tuple for one preset."""

    selected_scene = Path(scene_path) if scene_path is not None else preset.scene_path
    return [
        "--scene",
        str(selected_scene),
        "--profile",
        preset.profile,
        "--fluid-model",
        preset.fluid_model,
        "--viewer-camera-mode",
        preset.viewer_camera_mode,
    ]


def merge_sim_launch_preset_args(
    preset_args: Sequence[str],
    extra_args: Sequence[str] | None,
) -> list[str]:
    """Merge internal GUI extras while rejecting a second plant selection."""

    result = list(preset_args)
    extras = list(extra_args or ())
    protected = ("--scene", "--profile", "--fluid-model", "--viewer-camera-mode")
    for arg in extras:
        if any(arg == option or arg.startswith(f"{option}=") for option in protected):
            raise ValueError(f"simulator preset already owns launch option: {arg}")
    result.extend(extras)
    return result


def _load_resolved_profiles(profile_path: Path) -> dict[str, dict[str, Any]]:
    path = Path(profile_path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"cannot read simulator profiles: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid simulator profile JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"simulator profiles must be a JSON object: {path}")

    raw_profiles = {str(name): cfg for name, cfg in payload.items() if isinstance(cfg, dict)}
    resolved: dict[str, dict[str, Any]] = {}
    resolving: set[str] = set()

    def resolve(name: str) -> dict[str, Any]:
        if name in resolved:
            return dict(resolved[name])
        if name in resolving:
            raise ValueError(f"simulator profile inheritance cycle at: {name}")
        raw = raw_profiles.get(name)
        if raw is None:
            raise ValueError(f"simulator profile is missing: {name}")
        resolving.add(name)
        parent_name = raw.get("extends")
        base = resolve(str(parent_name)) if parent_name else {}
        base.update({key: value for key, value in raw.items() if key != "extends"})
        resolving.remove(name)
        resolved[name] = base
        return dict(base)

    for profile_name in raw_profiles:
        resolve(profile_name)
    return resolved


__all__ = [
    "COURSE_CURRENT_PRESET_ID",
    "DEFAULT_SIM_LAUNCH_PRESET_ID",
    "RESEARCH_POOL_CURRENT_PRESET_ID",
    "RESEARCH_POOL_DISTRIBUTED_PRESET_ID",
    "RESEARCH_POOL_DISTRIBUTED_HYBRID_PRESET_ID",
    "RESEARCH_POOL_DISTRIBUTED_WAVES_PRESET_ID",
    "SIM_LAUNCH_PRESETS",
    "SimLaunchPreset",
    "build_sim_launch_preset_args",
    "default_sim_launch_preset_id",
    "merge_sim_launch_preset_args",
    "resolve_sim_launch_preset",
    "sim_launch_preset_ids",
    "sim_launch_preset_labels",
    "sim_launch_presets_payload",
    "validate_sim_launch_preset",
]
