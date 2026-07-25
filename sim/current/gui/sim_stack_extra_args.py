"""Simulator extra-argument normalization for GUI launchers."""

from __future__ import annotations

from typing import Mapping, Sequence

from .sim_stack_env_types import NormalizedSimExtraArgs
from .sim_stack_viewer_args import apply_viewer_args, display_available
from .sim_stack_wrapper_args import WRAPPER_ONLY_ARGS, filter_wrapper_only_args


def normalize_sim_extra_args(
    extra_args: Sequence[str] | None,
    base_env: Mapping[str, str],
    *,
    platform_name: str,
) -> NormalizedSimExtraArgs:
    """Normalize GUI simulator launch args without mutating GUI state."""

    args, dropped = filter_wrapper_only_args(extra_args)
    events: list[str] = []
    if dropped:
        events.append("sim launch ignored wrapper-only args: " + " ".join(dropped))

    apply_viewer_args(args, events, base_env, platform_name=platform_name)
    return NormalizedSimExtraArgs(args=tuple(args), events=tuple(events))


__all__ = [
    "WRAPPER_ONLY_ARGS",
    "display_available",
    "filter_wrapper_only_args",
    "normalize_sim_extra_args",
]
