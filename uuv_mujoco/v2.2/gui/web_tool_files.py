"""Physics and course file access for the web GUI tools panel."""

from __future__ import annotations

import json
import math
import re
import shutil
import time
from pathlib import Path
from typing import Any

from .buoy_layout_model import (
    ROBOT_SPAWN_BODY_NAME,
    TANK_X_HALF_M,
    TANK_Y_HALF_M,
    clamp_xy,
    load_buoy_layout,
    load_robot_spawn,
    save_buoy_layout,
)
from .config import COURSE_SCENE_PATH, PHYSICS_PARAM_SPECS, PHYSICS_PROFILE_PATH
from .physics_param_format import _current_physics_profile, _format_physics_value, _get_nested_value, _set_nested_value
from .physics_param_status import _physics_current_mode_status, _physics_param_inactive_in_current


class WebToolFileManager:
    """Keeps web tool file operations constrained to known simulator files."""

    def __init__(self, node: Any) -> None:
        self.node = node
        self.physics_status = "physics params: idle"
        self.course_status = "course layout: idle"

    def status_payload(self) -> dict[str, Any]:
        return {
            "physics_status": self.physics_status,
            "buoy_layout_status": self.course_status,
            "physics_profile_path": str(PHYSICS_PROFILE_PATH),
            "course_scene_path": str(COURSE_SCENE_PATH),
        }

    def open_physics_params(self) -> dict[str, Any]:
        self.physics_status = f"physics params: opened {PHYSICS_PROFILE_PATH.name}"
        self.node.push_event(f"web physics params opened: {PHYSICS_PROFILE_PATH}")
        return {"status": self.physics_status, "path": str(PHYSICS_PROFILE_PATH)}

    def open_course_layout(self) -> dict[str, Any]:
        self.course_status = f"course layout: opened {COURSE_SCENE_PATH.name}"
        self.node.push_event(f"web course layout opened: {COURSE_SCENE_PATH}")
        return {"status": self.course_status, "path": str(COURSE_SCENE_PATH)}

    def load_physics_params(self) -> dict[str, Any]:
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = _current_physics_profile(payload)
            rows = [_physics_row_payload(profile, spec) for spec in PHYSICS_PARAM_SPECS]
        except Exception as exc:
            self.physics_status = f"physics params: load failed: {exc}"
            raise ValueError(self.physics_status) from exc
        self.physics_status = f"physics params: loaded {len(rows)} rows"
        return {"path": str(PHYSICS_PROFILE_PATH), "status": self.physics_status, "rows": rows}

    def apply_physics_params(self, values: dict[str, Any], *, restart: bool = False) -> dict[str, Any]:
        if not isinstance(values, dict):
            raise ValueError("values must be an object")
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = _current_physics_profile(payload)
            for spec in PHYSICS_PARAM_SPECS:
                key = str(spec["key"])
                if _physics_param_inactive_in_current(key):
                    continue
                if key not in values:
                    continue
                _set_nested_value(profile, key, _parse_web_physics_value(spec, values[key]))
            backup_path = _backup_physics_profile_file()
            PHYSICS_PROFILE_PATH.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            self.physics_status = f"physics params: apply failed: {exc}"
            self.node.push_event(f"web physics params apply failed: {exc}")
            raise ValueError(self.physics_status) from exc
        applied_status = (
            "physics params: applied; resetting sim"
            if restart
            else "physics params: applied to file; restart required for running sim"
        )
        self.physics_status = applied_status
        self.node.push_event(f"web physics params applied: backup {backup_path.name}")
        loaded = self.load_physics_params()
        self.physics_status = applied_status
        loaded.update({"backup_path": str(backup_path), "status": applied_status, "restart": bool(restart)})
        return loaded

    def load_course_layout(self) -> dict[str, Any]:
        try:
            items = load_buoy_layout(COURSE_SCENE_PATH)
            robot = load_robot_spawn(COURSE_SCENE_PATH)
        except Exception as exc:
            self.course_status = f"course layout load failed: {exc}"
            raise ValueError(self.course_status) from exc
        self.course_status = f"course layout: loaded {len(items)} buoys + robot"
        return {
            "path": str(COURSE_SCENE_PATH),
            "status": self.course_status,
            "tank": {"x_half_m": TANK_X_HALF_M, "y_half_m": TANK_Y_HALF_M},
            "robot_target_id": ROBOT_SPAWN_BODY_NAME,
            "robot": {
                "id": ROBOT_SPAWN_BODY_NAME,
                "label": robot.label,
                "color_hex": robot.color_hex,
                "layer": robot.layer,
                "x": robot.x,
                "y": robot.y,
                "z": robot.z,
            },
            "items": [
                {
                    "id": item.prefix,
                    "prefix": item.prefix,
                    "label": item.label,
                    "course": item.course,
                    "color_name": item.color_name,
                    "color_hex": item.color_hex,
                    "layer": item.layer,
                    "x": item.x,
                    "y": item.y,
                    "z": item.z,
                    "fixed_underwater": item.fixed_underwater,
                }
                for item in items
            ],
        }

    def save_course_layout(
        self,
        positions: dict[str, Any],
        *,
        robot_xy: Any = None,
        reset: bool = False,
    ) -> dict[str, Any]:
        if not isinstance(positions, dict):
            raise ValueError("positions must be an object")
        clean_positions = {
            str(prefix): _parse_xy(value, label=str(prefix))
            for prefix, value in positions.items()
        }
        clean_robot_xy = None if robot_xy is None else _parse_xy(robot_xy, label="robot")
        try:
            backup_path = save_buoy_layout(COURSE_SCENE_PATH, clean_positions, robot_xy=clean_robot_xy)
        except Exception as exc:
            self.course_status = f"course layout save failed: {exc}"
            self.node.push_event(f"web course layout save failed: {exc}")
            raise ValueError(self.course_status) from exc
        saved_status = "course layout: saved; resetting sim" if reset else "course layout: saved; restart sim to reload"
        self.course_status = saved_status
        self.node.push_event(f"web course layout saved: backup {backup_path.name}")
        loaded = self.load_course_layout()
        self.course_status = saved_status
        loaded.update({"backup_path": str(backup_path), "status": saved_status, "reset": bool(reset)})
        return loaded

    def read_tool_file(self, kind: str) -> dict[str, Any]:
        path = _tool_file_path(kind)
        try:
            content = path.read_text(encoding="utf-8")
        except Exception as exc:
            raise ValueError(f"{kind} read failed: {exc}") from exc
        return {"kind": kind, "path": str(path), "content": content}

    def save_tool_file(self, kind: str, content: str) -> dict[str, Any]:
        path = _tool_file_path(kind)
        if not isinstance(content, str):
            raise ValueError("content must be a string")
        try:
            backup_path = path.with_name(f"{path.name}.{time.strftime('%Y%m%d_%H%M%S')}.bak")
            if path.exists():
                backup_path.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
            path.write_text(content, encoding="utf-8")
        except Exception as exc:
            raise ValueError(f"{kind} save failed: {exc}") from exc
        status = self._set_saved_status(kind, path)
        self.node.push_event(f"web {kind} saved: {path}")
        return {"kind": kind, "path": str(path), "backup_path": str(backup_path), "status": status}

    def _set_saved_status(self, kind: str, path: Path) -> str:
        if kind == "physics":
            self.physics_status = f"physics params: saved {path.name}"
            return self.physics_status
        self.course_status = f"course layout: saved {path.name}"
        return self.course_status


def _tool_file_path(kind: str) -> Path:
    if kind == "physics":
        return PHYSICS_PROFILE_PATH
    if kind in {"course", "course_layout"}:
        return COURSE_SCENE_PATH
    raise ValueError(f"unsupported tool file: {kind}")


def _physics_row_payload(profile: dict[str, Any], spec: dict[str, Any]) -> dict[str, Any]:
    key = str(spec["key"])
    value = _get_nested_value(profile, key)
    if value is None:
        value = spec["default"]
    return {
        "key": key,
        "label": str(spec["label"]),
        "kind": str(spec.get("kind", "scalar")),
        "value": _format_physics_value(value),
        "default": _format_physics_value(spec["default"]),
        "description": str(spec["description"]),
        "inactive": bool(_physics_param_inactive_in_current(key)),
        "mode_status": _physics_current_mode_status(key),
    }


def _parse_web_physics_value(spec: dict[str, Any], value: Any) -> Any:
    raw = str(value).strip()
    kind = str(spec.get("kind", "scalar"))
    vector_match = re.fullmatch(r"vector(\d+)", kind)
    if not vector_match:
        return _finite_float(raw, str(spec["label"]))
    expected_len = int(vector_match.group(1))
    pieces = [piece for piece in re.split(r"[,\s]+", raw) if piece]
    if len(pieces) != expected_len:
        raise ValueError(f"{spec['label']} must have exactly {expected_len} numbers")
    return [_finite_float(piece, f"{spec['label']}[{idx}]") for idx, piece in enumerate(pieces)]


def _finite_float(value: Any, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _parse_xy(value: Any, *, label: str) -> tuple[float, float]:
    if isinstance(value, dict):
        x_raw = value.get("x")
        y_raw = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        x_raw = value[0]
        y_raw = value[1]
    else:
        raise ValueError(f"{label} XY must have x and y")
    return clamp_xy(_finite_float(x_raw, f"{label}.x"), _finite_float(y_raw, f"{label}.y"))


def _backup_physics_profile_file() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_path = PHYSICS_PROFILE_PATH.with_name(f"{PHYSICS_PROFILE_PATH.name}.bak_gui_phys_web_{stamp}")
    shutil.copy2(PHYSICS_PROFILE_PATH, backup_path)
    return backup_path


__all__ = ["WebToolFileManager"]
