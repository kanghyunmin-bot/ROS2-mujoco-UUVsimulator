"""Environment direct-gain override application."""

from __future__ import annotations

from typing import Callable, MutableMapping, Optional, Sequence

from .thruster_direct_gain_values import read_env_direct_gain_scale
from .thruster_param_common import log_optional


def apply_group_direct_gain_override(
    *,
    env_name: str,
    thruster_names: Sequence[str],
    thruster_direct_scale: MutableMapping[str, float],
    env_get: Callable[[str, str], str],
    log: Optional[Callable[[str], None]],
    label: str,
) -> None:
    raw_value, value = read_env_direct_gain_scale(env_get, env_name)
    if not raw_value:
        return
    if value is None:
        log_optional(log, f"[thruster] ignoring invalid {env_name}={raw_value!r}")
        return
    for name in thruster_names:
        if name in thruster_direct_scale:
            thruster_direct_scale[name] = value
    log_optional(log, f"[thruster] {label} direct gain override: {env_name}={value:.4f}")


def apply_per_thruster_direct_gain_overrides(
    *,
    per_thruster_env: Sequence[tuple[str, str]],
    thruster_direct_scale: MutableMapping[str, float],
    env_get: Callable[[str, str], str],
    log: Optional[Callable[[str], None]],
) -> None:
    for env_name, thr_name in per_thruster_env:
        raw_value, value = read_env_direct_gain_scale(env_get, env_name)
        if not raw_value:
            continue
        if value is None:
            log_optional(log, f"[thruster] ignoring invalid {env_name}={raw_value!r}")
            continue
        if thr_name in thruster_direct_scale:
            thruster_direct_scale[thr_name] = value
            log_optional(
                log,
                f"[thruster] per-thruster direct gain override: {env_name}={value:.4f}",
            )


__all__ = ["apply_group_direct_gain_override", "apply_per_thruster_direct_gain_overrides"]
