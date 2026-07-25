"""Active-profile reporting helpers for closed-loop contract audits."""

from __future__ import annotations

from typing import Any

from audit_closed_loop_profile_keys import CURRENT_INACTIVE_KEYS, PROFILE_KEYS


def active_profile_report(profile: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    current_profile = {key: profile.get(key) for key in PROFILE_KEYS}
    active_keys = [
        key
        for key in PROFILE_KEYS
        if key not in CURRENT_INACTIVE_KEYS and profile_key_is_active(profile, key)
    ]
    return current_profile, active_keys


def profile_key_is_active(profile: dict[str, Any], key: str) -> bool:
    value = profile.get(key)
    if value is None:
        return False
    if key == "dynamic_fluidcoef":
        return isinstance(value, dict) and bool(value.get("active", False))
    return True


__all__ = ["active_profile_report", "profile_key_is_active"]
