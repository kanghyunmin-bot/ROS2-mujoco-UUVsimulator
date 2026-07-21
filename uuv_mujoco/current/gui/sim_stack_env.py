"""Compatibility facade for GUI-started MuJoCo/SITL environment helpers."""

from __future__ import annotations

from .sim_stack_env_args import build_initial_depth_args, normalize_sim_extra_args
from .sim_stack_env_contract import (
    build_gui_sim_stack_env,
    normalize_ekf_contract,
    normalize_run_mode,
    profile_defaults,
)
from .sim_stack_env_flags import (
    STRICT_TRUE_VALUES,
    TRUE_VALUES,
    any_arg_present,
    arg_present,
    env_bool,
    env_flag_default,
)
from .sim_stack_env_types import InitialDepthArgs, NormalizedSimExtraArgs

_env_bool = env_bool
_env_flag_default = env_flag_default
_arg_present = arg_present
_any_arg_present = any_arg_present
_normalize_run_mode = normalize_run_mode
_normalize_ekf_contract = normalize_ekf_contract
_profile_defaults = profile_defaults

__all__ = [
    "InitialDepthArgs",
    "NormalizedSimExtraArgs",
    "STRICT_TRUE_VALUES",
    "TRUE_VALUES",
    "_any_arg_present",
    "_arg_present",
    "_env_bool",
    "_env_flag_default",
    "_normalize_ekf_contract",
    "_normalize_run_mode",
    "_profile_defaults",
    "any_arg_present",
    "arg_present",
    "build_gui_sim_stack_env",
    "build_initial_depth_args",
    "env_bool",
    "env_flag_default",
    "normalize_ekf_contract",
    "normalize_run_mode",
    "normalize_sim_extra_args",
    "profile_defaults",
]
