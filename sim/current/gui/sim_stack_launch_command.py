"""Command construction for GUI-started simulator stacks."""

from __future__ import annotations

DEFAULT_CAMERA_WIDTH = "1280"
DEFAULT_CAMERA_HEIGHT = "720"
DEFAULT_CAMERA_HZ = "10"
DEFAULT_CAMERA_PRESET_ID = "competition_fixed"
FIXED_CAMERA_CONFIG = {
    "preset_id": DEFAULT_CAMERA_PRESET_ID,
    "label": "1280x720 @ 10Hz (fixed)",
    "width": 1280,
    "height": 720,
    "hz": 10.0,
    "hz_arg": "10",
}
CAMERA_PRESETS: tuple[dict[str, object], ...] = ()


def _append_arg_default(owner, args: list[str], option: str, value: str) -> None:
    if not owner._arg_present(args, option):
        args.extend([option, value])


def _remove_arg(args: list[str], option: str) -> None:
    index = 0
    while index < len(args):
        if args[index] == option:
            del args[index : index + 2]
            continue
        if args[index].startswith(option + "="):
            del args[index]
            continue
        index += 1


def normalize_camera_config(values: dict[str, object] | None = None, *, strict_preset: bool = False) -> dict[str, object]:
    del values, strict_preset
    return dict(FIXED_CAMERA_CONFIG)


def camera_config_from_owner(owner) -> dict[str, object]:
    del owner
    return dict(FIXED_CAMERA_CONFIG)


def camera_presets_payload() -> list[dict[str, object]]:
    return []


def _append_default_stereo_camera_args(
    owner,
    launch_extra_args: list[str],
    *,
    enabled: bool | None = None,
) -> None:
    if enabled is None:
        enabled = owner._env_flag("UUV_GUI_STEREO_CAMERA_ENABLE", True)
    if not enabled:
        return
    if not owner._arg_present(launch_extra_args, "--ros2-images"):
        launch_extra_args.append("--ros2-images")
    config = camera_config_from_owner(owner)
    for option in ("--ros2-image-width", "--ros2-image-height", "--ros2-image-hz"):
        _remove_arg(launch_extra_args, option)
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


def build_sim_stack_launch_command(
    owner,
    *,
    start_script,
    backend: str,
    extra_args: list[str] | None,
    enable_stereo_camera: bool | None = None,
) -> list[str]:
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
    if not owner._env_flag("UUV_GUI_SITL_REBUILD", False):
        cmd.append("--sitl-no-rebuild")
        owner.node.push_event("SITL rebuild skipped: using existing ArduSub binary")
    launch_extra_args = owner._normalized_sim_extra_args(extra_args)
    _append_default_stereo_camera_args(
        owner,
        launch_extra_args,
        enabled=enable_stereo_camera,
    )
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
