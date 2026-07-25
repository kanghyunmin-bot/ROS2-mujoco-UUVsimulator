"""Viewer/headless argument policy for GUI simulator launchers."""

from __future__ import annotations

from typing import Mapping

from .sim_stack_env_flags import any_arg_present, arg_present, env_bool, env_flag_default


def display_available(base_env: Mapping[str, str], *, platform_name: str) -> bool:
    return platform_name == "darwin" or bool(base_env.get("DISPLAY")) or bool(base_env.get("WAYLAND_DISPLAY"))


def apply_viewer_args(args: list[str], events: list[str], base_env: Mapping[str, str], *, platform_name: str) -> None:
    # A local GUI start is expected to show the MuJoCo simulation.  Headless is
    # still available for explicit stability runs or display-less sessions.
    default_viewer = "1" if display_available(base_env, platform_name=platform_name) else "0"
    viewer_enabled = env_bool(base_env, "UUV_GUI_MUJOCO_VIEWER", default_viewer)
    if arg_present(args, "--headless"):
        events.append("sim viewer: headless MuJoCo runtime")
    elif not viewer_enabled:
        args.append("--headless")
        events.append("sim viewer: headless MuJoCo runtime")
    else:
        events.append("sim viewer: MuJoCo GLFW viewer enabled")
    if not any_arg_present(args, ("--qgc-video", "--no-qgc-video")):
        if env_flag_default(base_env, "UUV_GUI_QGC_VIDEO", False):
            args.append("--qgc-video")
            events.append("QGC video stream enabled")
        else:
            args.append("--no-qgc-video")
            events.append("QGC video stream disabled")


__all__ = ["apply_viewer_args", "display_available"]
