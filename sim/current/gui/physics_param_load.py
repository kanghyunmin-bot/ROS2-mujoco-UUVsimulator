"""Load GUI physics profile values into Tk fields."""

from __future__ import annotations

import json

from .config import PHYSICS_PARAM_SPECS, PHYSICS_PROFILE_PATH
from .physics_param_format import _current_physics_profile, _format_physics_value, _get_nested_value


def _load_physics_params_into_fields(self, silent: bool = False) -> None:
    try:
        payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
        profile = _current_physics_profile(payload)
        for spec in PHYSICS_PARAM_SPECS:
            _load_spec_into_field(self, profile, spec)
    except Exception as exc:
        self._set_physics_status(f"physics params: load failed: {exc}")
        return
    if not silent:
        self._set_physics_status("physics params: loaded")


def _load_spec_into_field(self, profile: dict, spec: dict) -> None:
    key = str(spec["key"])
    value = _get_nested_value(profile, key)
    if value is None:
        value = spec["default"]
    self.physics_param_vars[key].set(_format_physics_value(value))


__all__ = ["_load_physics_params_into_fields"]
