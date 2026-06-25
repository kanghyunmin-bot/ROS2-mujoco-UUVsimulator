"""Persist GUI physics profile edits."""

from __future__ import annotations

import datetime as _dt
import json
import shutil

from .config import PHYSICS_PARAM_SPECS, PHYSICS_PROFILE_PATH
from .physics_param_format import _current_physics_profile, _set_nested_value
from .physics_param_parse import _parse_physics_value
from .physics_param_status import _physics_param_inactive_in_current


def apply_specs_to_profile(self, profile: dict) -> None:
    for spec in PHYSICS_PARAM_SPECS:
        key = str(spec["key"])
        if _physics_param_inactive_in_current(key):
            continue
        _set_nested_value(profile, key, _parse_physics_value(self, spec))


def read_profile_payload() -> tuple[dict, dict]:
    payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
    return payload, _current_physics_profile(payload)


def backup_profile_file():
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = PHYSICS_PROFILE_PATH.with_name(f"{PHYSICS_PROFILE_PATH.name}.bak_gui_phys_{stamp}")
    shutil.copy2(PHYSICS_PROFILE_PATH, backup_path)
    return backup_path


def write_profile_payload(payload: dict) -> None:
    PHYSICS_PROFILE_PATH.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


__all__ = [
    "apply_specs_to_profile",
    "backup_profile_file",
    "read_profile_payload",
    "write_profile_payload",
]
