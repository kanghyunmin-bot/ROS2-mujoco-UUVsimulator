"""Initial-depth source-selection contracts for GUI-started simulator stacks."""

from __future__ import annotations

from typing import Mapping, Sequence

from .sim_stack_env_flags import arg_present, env_bool
from .sim_stack_env_types import InitialDepthArgs


def initial_depth_args_explicitly_set(launch_extra_args: Sequence[str]) -> bool:
    return arg_present(launch_extra_args, "--initial-depth-m") or arg_present(
        launch_extra_args,
        "--initial-bar30-depth-m",
    )


def real_start_initial_depth_contract(base_env: Mapping[str, str]) -> InitialDepthArgs | None:
    if env_bool(base_env, "UUV_REAL_START_STATE"):
        return InitialDepthArgs(events=("sim initial state: real CSV contract handled by launcher",))
    return None


def bar30_initial_depth_contract(base_env: Mapping[str, str]) -> InitialDepthArgs | None:
    default_bar30_depth_m = str(
        base_env.get(
            "UUV_GUI_DEFAULT_INITIAL_BAR30_DEPTH_M",
            base_env.get("UUV_SITL_INITIAL_BAR30_DEPTH_M", "auto"),
        )
    ).strip()
    gui_bar30_depth_m = str(base_env.get("UUV_GUI_INITIAL_BAR30_DEPTH_M", default_bar30_depth_m)).strip()
    if gui_bar30_depth_m and gui_bar30_depth_m.lower() not in {"off", "none", "false"}:
        return InitialDepthArgs(
            args=("--initial-bar30-depth-m", gui_bar30_depth_m),
            events=(f"sim initial depth: bar30={gui_bar30_depth_m} m",),
        )
    return None


__all__ = [
    "bar30_initial_depth_contract",
    "initial_depth_args_explicitly_set",
    "real_start_initial_depth_contract",
]
