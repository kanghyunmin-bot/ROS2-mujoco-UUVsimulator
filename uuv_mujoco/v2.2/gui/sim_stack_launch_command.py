"""Command construction for GUI-started simulator stacks."""

from __future__ import annotations

import os


DEFAULT_CAMERA_WIDTH = "640"
DEFAULT_CAMERA_HEIGHT = "360"
DEFAULT_CAMERA_HZ = "4"
DEFAULT_CAMERA_PRESET_ID = "balanced"

CAMERA_PRESETS = (
    {"id": "balanced", "label": "640x360 @ 4Hz", "width": 640, "height": 360, "hz": 4.0},
    {"id": "smooth540", "label": "960x540 @ 10Hz", "width": 960, "height": 540, "hz": 10.0},
    {"id": "hd720", "label": "1280x720 @ 5Hz", "width": 1280, "height": 720, "hz": 5.0},
    {"id": "hd720_fast", "label": "1280x720 @ 10Hz", "width": 1280, "height": 720, "hz": 10.0},
)


def _env_value(owner, name: str, default: str) -> str:
    env = getattr(owner, "env", None)
    raw = env.get(name) if isinstance(env, dict) else os.environ.get(name)
    value = str(raw if raw is not None else default).strip()
    return value or default


def _env_value_any(owner, names: tuple[str, ...], default: str) -> str:
    for name in names:
        value = _env_value(owner, name, "")
        if value:
            return value
    return default


def _append_arg_default(owner, args: list[str], option: str, value: str) -> None:
    if not owner._arg_present(args, option):
        args.extend([option, value])


def _preset_by_id(preset_id: str) -> dict[str, object] | None:
    for preset in CAMERA_PRESETS:
        if preset["id"] == preset_id:
            return dict(preset)
    return None


def _matching_preset_id(width: int, height: int, hz: float) -> str:
    for preset in CAMERA_PRESETS:
        if preset["width"] == width and preset["height"] == height and abs(float(preset["hz"]) - hz) < 0.001:
            return str(preset["id"])
    return "custom"


def _format_hz(value: float) -> str:
    return str(int(value)) if float(value).is_integer() else f"{value:g}"


def _coerce_int(value: object, default: int, *, minimum: int, maximum: int) -> int:
    try:
        parsed = int(float(str(value).strip()))
    except (TypeError, ValueError):
        parsed = default
    return max(minimum, min(maximum, parsed))


def _coerce_float(value: object, default: float, *, minimum: float, maximum: float) -> float:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        parsed = default
    return max(minimum, min(maximum, parsed))


def normalize_camera_config(values: dict[str, object] | None = None, *, strict_preset: bool = False) -> dict[str, object]:
    values = values or {}
    preset_id = str(values.get("preset_id", values.get("preset", DEFAULT_CAMERA_PRESET_ID))).strip()
    preset = _preset_by_id(preset_id)
    if preset is None and strict_preset and preset_id:
        raise ValueError(f"unsupported camera preset: {preset_id}")
    base = preset or _preset_by_id(DEFAULT_CAMERA_PRESET_ID) or {}
    width = _coerce_int(values.get("width", base.get("width", DEFAULT_CAMERA_WIDTH)), int(base.get("width", 640)), minimum=64, maximum=1920)
    height = _coerce_int(
        values.get("height", base.get("height", DEFAULT_CAMERA_HEIGHT)),
        int(base.get("height", 360)),
        minimum=64,
        maximum=1080,
    )
    hz = _coerce_float(values.get("hz", base.get("hz", DEFAULT_CAMERA_HZ)), float(base.get("hz", 4.0)), minimum=0.1, maximum=60.0)
    matched_id = _matching_preset_id(width, height, hz)
    matched = _preset_by_id(matched_id) if matched_id != "custom" else None
    label = str(matched["label"]) if matched else f"{width}x{height} @ {_format_hz(hz)}Hz"
    return {
        "preset_id": matched_id,
        "label": label,
        "width": width,
        "height": height,
        "hz": hz,
        "hz_arg": _format_hz(hz),
    }


def camera_config_from_owner(owner) -> dict[str, object]:
    selected = getattr(owner, "_camera_config", None)
    if isinstance(selected, dict):
        return normalize_camera_config(selected)
    return normalize_camera_config(
        {
            "preset_id": "custom",
            "width": _env_value_any(owner, ("UUV_GUI_CAMERA_WIDTH", "UUV_GUI_STEREO_CAMERA_WIDTH"), DEFAULT_CAMERA_WIDTH),
            "height": _env_value_any(owner, ("UUV_GUI_CAMERA_HEIGHT", "UUV_GUI_STEREO_CAMERA_HEIGHT"), DEFAULT_CAMERA_HEIGHT),
            "hz": _env_value_any(owner, ("UUV_GUI_CAMERA_HZ", "UUV_GUI_STEREO_CAMERA_HZ"), DEFAULT_CAMERA_HZ),
        }
    )


def camera_presets_payload() -> list[dict[str, object]]:
    return [
        {
            "id": str(preset["id"]),
            "label": str(preset["label"]),
            "width": int(preset["width"]),
            "height": int(preset["height"]),
            "hz": float(preset["hz"]),
        }
        for preset in CAMERA_PRESETS
    ]


def _append_default_stereo_camera_args(owner, launch_extra_args: list[str]) -> None:
    if not owner._env_flag("UUV_GUI_STEREO_CAMERA_ENABLE", True):
        return
    if not owner._arg_present(launch_extra_args, "--ros2-images"):
        launch_extra_args.append("--ros2-images")
    config = camera_config_from_owner(owner)
    _append_arg_default(
        owner,
        launch_extra_args,
        "--ros2-image-width",
        str(config["width"]),
    )
    _append_arg_default(
        owner,
        launch_extra_args,
        "--ros2-image-height",
        str(config["height"]),
    )
    _append_arg_default(
        owner,
        launch_extra_args,
        "--ros2-image-hz",
        str(config["hz_arg"]),
    )
    owner.node.push_event("Camera feed: ROS2 image topic enabled")


def build_sim_stack_launch_command(owner, *, start_script, backend: str, extra_args: list[str] | None) -> list[str]:
    cmd = [str(start_script)]
    owner._append_mavros_surface_args(cmd)
    requested_args = extra_args or []
    if (
        owner._env_flag("UUV_GUI_SITL_DIRECT_MAVLINK", False)
        and not owner._arg_present(requested_args, "--direct-mavlink")
        and not owner._arg_present(requested_args, "--legacy-mavproxy")
    ):
        cmd.append("--direct-mavlink")
        owner.node.push_event("SITL transport: direct MAVLink outputs")
    if backend != "docker" and not owner._env_flag("UUV_GUI_SITL_REBUILD", False):
        cmd.append("--sitl-no-rebuild")
        owner.node.push_event("SITL rebuild skipped: using existing ArduSub binary")
    launch_extra_args = owner._normalized_sim_extra_args(extra_args)
    _append_default_stereo_camera_args(owner, launch_extra_args)
    owner._append_initial_depth_args(cmd, launch_extra_args)
    if launch_extra_args:
        cmd.extend(launch_extra_args)
    return cmd


__all__ = [
    "CAMERA_PRESETS",
    "build_sim_stack_launch_command",
    "camera_config_from_owner",
    "camera_presets_payload",
    "normalize_camera_config",
]
