"""Physics profile apply compatibility surface for the GUI."""

from __future__ import annotations

from .physics_param_error_log import _log_physics_apply_error
from .physics_param_load import _load_physics_params_into_fields
from .physics_param_persist import (
    apply_specs_to_profile,
    backup_profile_file,
    read_profile_payload,
    write_profile_payload,
)


def _apply_physics_params(self, restart: bool = False) -> None:
    try:
        payload, profile = read_profile_payload()
        apply_specs_to_profile(self, profile)
        backup_path = backup_profile_file()
        write_profile_payload(payload)
    except Exception as exc:
        self._set_physics_status(f"physics params: apply failed: {exc}")
        self.node.push_event(f"physics params apply failed: {exc}")
        _log_physics_apply_error(self, exc)
        return

    _load_physics_params_into_fields(self, silent=True)
    self.node.push_event(f"physics params applied: backup {backup_path.name}")
    _finish_physics_apply(self, restart=restart)


def _finish_physics_apply(self, *, restart: bool) -> None:
    if restart:
        self._set_physics_status("physics params: applied; restarting sim")
        self._restart_sim_stack_after_physics_apply()
    else:
        self._set_physics_status("physics params: applied to file; restart required for running sim")


__all__ = ["_apply_physics_params", "_load_physics_params_into_fields", "_log_physics_apply_error"]
