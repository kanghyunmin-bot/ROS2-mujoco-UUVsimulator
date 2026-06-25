"""Error logging for GUI physics profile apply failures."""

from __future__ import annotations

import datetime as _dt

from .config import PHYSICS_PARAM_SPECS, SIM_STACK_DIR


def _log_physics_apply_error(self, exc: Exception) -> None:
    try:
        log_dir = SIM_STACK_DIR / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / "gui_physics_apply_errors.log"
        with log_path.open("a", encoding="utf-8") as log_file:
            _write_apply_error_log(self, log_file, exc)
    except Exception:
        pass


def _write_apply_error_log(self, log_file, exc: Exception) -> None:
    stamp = _dt.datetime.now().isoformat(timespec="seconds")
    log_file.write(f"[{stamp}] apply failed: {exc!r}\n")
    for spec in PHYSICS_PARAM_SPECS:
        key = str(spec["key"])
        try:
            raw = self.physics_param_vars[key].get()
        except Exception:
            raw = "<unreadable>"
        log_file.write(f"  {key}={raw!r}\n")


__all__ = ["_log_physics_apply_error"]
