"""Base-link debug initial-depth contract helpers."""

from __future__ import annotations

from typing import Mapping, Sequence

from .sim_stack_env_flags import STRICT_TRUE_VALUES, arg_present, env_bool
from .sim_stack_env_types import InitialDepthArgs


def base_link_initial_depth_event(gui_initial_depth_m: str) -> str:
    try:
        depth_value = float(gui_initial_depth_m)
    except ValueError:
        depth_value = 0.0
    if depth_value < 0.0:
        return f"sim drop start: base_link={-depth_value:.3f} m above water"
    return f"sim initial depth: base_link={gui_initial_depth_m} m"


def base_link_hold_target(base_env: Mapping[str, str], gui_initial_depth_m: str, hold_until_release: bool) -> str:
    return (
        str(base_env.get("UUV_GUI_INITIAL_DEPTH_HOLD_TARGET_M", gui_initial_depth_m)).strip()
        if hold_until_release
        else ""
    )


def base_link_debug_initial_depth_contract(
    base_env: Mapping[str, str],
    launch_extra_args: Sequence[str],
) -> InitialDepthArgs:
    default_initial_depth_m = str(base_env.get("UUV_GUI_DEFAULT_INITIAL_DEPTH_M", "")).strip()
    gui_initial_depth_m = str(base_env.get("UUV_GUI_INITIAL_DEPTH_M", default_initial_depth_m)).strip()
    if not gui_initial_depth_m:
        return InitialDepthArgs()

    hold_until_release = env_bool(
        base_env,
        "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE",
        true_values=STRICT_TRUE_VALUES,
    )
    hold_target_m = base_link_hold_target(base_env, gui_initial_depth_m, hold_until_release)

    args = ["--initial-depth-m", gui_initial_depth_m]
    if hold_until_release and hold_target_m and not arg_present(launch_extra_args, "--initial-depth-hold-target-m"):
        args.extend(["--initial-depth-hold-target-m", hold_target_m])
    if hold_until_release and not arg_present(launch_extra_args, "--hold-initial-depth-until-release"):
        args.append("--hold-initial-depth-until-release")
    return InitialDepthArgs(args=tuple(args), events=(base_link_initial_depth_event(gui_initial_depth_m),))


__all__ = [
    "base_link_debug_initial_depth_contract",
    "base_link_hold_target",
    "base_link_initial_depth_event",
]
