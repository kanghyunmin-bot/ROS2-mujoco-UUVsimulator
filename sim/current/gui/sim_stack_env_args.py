"""Compatibility exports for GUI simulator stack launch-argument helpers."""

from __future__ import annotations

from .sim_stack_extra_args import normalize_sim_extra_args
from .sim_stack_initial_depth_args import build_initial_depth_args


__all__ = ["build_initial_depth_args", "normalize_sim_extra_args"]
