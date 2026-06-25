"""Initial-depth launch argument defaults for GUI-started simulator stacks."""

from __future__ import annotations

from typing import Mapping, Sequence

from .sim_stack_initial_depth_contract import (
    bar30_initial_depth_contract,
    base_link_debug_initial_depth_contract,
    initial_depth_args_explicitly_set,
    real_start_initial_depth_contract,
)
from .sim_stack_env_types import InitialDepthArgs


def build_initial_depth_args(
    base_env: Mapping[str, str],
    *,
    launch_extra_args: Sequence[str],
) -> InitialDepthArgs:
    """Build GUI default initial-depth args without touching GUI state."""

    if initial_depth_args_explicitly_set(launch_extra_args):
        return InitialDepthArgs()

    real_start_contract = real_start_initial_depth_contract(base_env)
    if real_start_contract is not None:
        return real_start_contract

    # ALT_HOLD uses pressure, so GUI SITL defaults start from Bar30 depth, not
    # base_link depth. Explicit launch args still win.
    bar30_contract = bar30_initial_depth_contract(base_env)
    if bar30_contract is not None:
        return bar30_contract

    # Base-link depth remains available only as an explicit debug path.
    return base_link_debug_initial_depth_contract(base_env, launch_extra_args)


__all__ = ["build_initial_depth_args"]
