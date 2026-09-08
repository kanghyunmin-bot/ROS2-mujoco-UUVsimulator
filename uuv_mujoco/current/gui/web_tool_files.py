"""Physics and course file access for the web GUI tools panel."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

from physics.sim_profile_validation import SIM_PROFILE_PARAM_LIMITS, validate_profile_param_value

from .buoy_layout_model import (
    ROBOT_SPAWN_BODY_NAME,
    TANK_X_HALF_M,
    TANK_Y_HALF_M,
    clamp_xy,
    load_buoy_layout,
    load_robot_spawn,
    save_buoy_layout,
)
from .config import (
    COURSE_LAYOUT_CONFIG_PATH,
    COURSE_SCENE_PATH,
    PHYSICS_PARAM_SPECS,
    PHYSICS_PROFILE_PATH,
    TEST_TANK_SCENE_PATH,
)
from .file_persistence import atomic_write_text, backup_file
from .physics_param_format import _current_physics_profile, _format_physics_value, _get_nested_value, _set_nested_value
from .physics_param_status import _physics_current_mode_status, _physics_param_inactive_in_current
from .test_tank_layout_model import (
    COURSE_MODE_COMPETITION,
    COURSE_MODE_TEST_TANK,
    TEST_TANK_PINGER_ID,
    TEST_TANK_YELLOW_ID,
    generate_test_tank_scene,
    load_course_layout_config,
    normalize_course_mode,
    save_course_layout_config,
    test_tank_payload,
    test_tank_positions,
)


class WebToolFileManager:
    """Keeps web tool file operations constrained to known simulator files."""

    def __init__(self, node: Any) -> None:
        self.node = node
        self.physics_status = "physics params [current]: idle"
        self.course_status = "course layout: idle"

    def status_payload(self) -> dict[str, Any]:
        config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
        mode = normalize_course_mode(config.get("active_mode"))
        return {
            "physics_status": self.physics_status,
            "buoy_layout_status": self.course_status,
            "physics_profile_path": str(PHYSICS_PROFILE_PATH),
            "course_scene_path": str(TEST_TANK_SCENE_PATH if mode == COURSE_MODE_TEST_TANK else COURSE_SCENE_PATH),
            "course_layout_mode": mode,
        }

    def open_physics_params(self) -> dict[str, Any]:
        self.physics_status = f"physics params: opened {PHYSICS_PROFILE_PATH.name}"
        self.node.push_event(f"web physics params opened: {PHYSICS_PROFILE_PATH}")
        return {"status": self.physics_status, "path": str(PHYSICS_PROFILE_PATH)}

    def open_course_layout(self) -> dict[str, Any]:
        config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
        mode = normalize_course_mode(config.get("active_mode"))
        path = TEST_TANK_SCENE_PATH if mode == COURSE_MODE_TEST_TANK else COURSE_SCENE_PATH
        self.course_status = f"course layout: opened {mode}"
        self.node.push_event(f"web course layout opened: {mode} ({path})")
        return {"status": self.course_status, "path": str(path), "mode": mode}

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
            atomic_write_text(
                PHYSICS_PROFILE_PATH,
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
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

    def load_course_layout(self, mode: str | None = None) -> dict[str, Any]:
        config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
        active_mode = normalize_course_mode(config.get("active_mode"))
        selected_mode = normalize_course_mode(mode, default=active_mode) if mode else active_mode
        if selected_mode == COURSE_MODE_TEST_TANK:
            payload = test_tank_payload(config)
            self.course_status = "course layout: loaded test tank physical buoy + pinger + robot"
            return {
                "path": str(TEST_TANK_SCENE_PATH),
                "config_path": str(COURSE_LAYOUT_CONFIG_PATH),
                "status": self.course_status,
                "mode": selected_mode,
                "active_mode": active_mode,
                "mode_options": _course_mode_options(),
                "robot_target_id": ROBOT_SPAWN_BODY_NAME,
                **payload,
            }
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
            "mode": selected_mode,
            "active_mode": active_mode,
            "mode_options": _course_mode_options(),
            "tank": {
                "x_half_m": TANK_X_HALF_M,
                "y_half_m": TANK_Y_HALF_M,
                "length_m": TANK_X_HALF_M * 2.0,
                "width_m": TANK_Y_HALF_M * 2.0,
                "depth_m": 11.0,
            },
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
        mode: str | None = None,
        robot_xy: Any = None,
        reset: bool = False,
    ) -> dict[str, Any]:
        if not isinstance(positions, dict):
            raise ValueError("positions must be an object")
        config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
        active_mode = normalize_course_mode(config.get("active_mode"))
        selected_mode = normalize_course_mode(mode, default=active_mode) if mode else active_mode
        if selected_mode == COURSE_MODE_TEST_TANK:
            return self._save_test_tank_layout(positions, robot_xy=robot_xy, reset=reset)
        clean_positions = {
            str(prefix): _parse_xy(value, label=str(prefix))
            for prefix, value in positions.items()
        }
        clean_robot_xy = None if robot_xy is None else _parse_xy(robot_xy, label="robot")
        try:
            backup_path = save_buoy_layout(COURSE_SCENE_PATH, clean_positions, robot_xy=clean_robot_xy)
            config_backup = save_course_layout_config(
                COURSE_LAYOUT_CONFIG_PATH,
                mode=COURSE_MODE_COMPETITION,
            )
        except Exception as exc:
            self.course_status = f"course layout save failed: {exc}"
            self.node.push_event(f"web course layout save failed: {exc}")
            raise ValueError(self.course_status) from exc
        saved_status = "course layout: saved; restarting sim" if reset else "course layout: saved; restart sim to reload"
        self.course_status = saved_status
        self.node.push_event(f"web course layout saved: backup {backup_path.name}")
        loaded = self.load_course_layout(COURSE_MODE_COMPETITION)
        self.course_status = saved_status
        loaded.update(
            {
                "backup_path": str(backup_path),
                "config_backup_path": str(config_backup) if config_backup else "",
                "status": saved_status,
                "reset": bool(reset),
            }
        )
        return loaded

    def _save_test_tank_layout(
        self,
        positions: dict[str, Any],
        *,
        robot_xy: Any,
        reset: bool,
    ) -> dict[str, Any]:
        config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
        current = test_tank_positions(config)
        clean_robot_xy = current["robot"] if robot_xy is None else _parse_xy_unclamped(robot_xy, label="robot")
        yellow_xy = _parse_xy_unclamped(
            positions.get(TEST_TANK_YELLOW_ID, current["yellow"]),
            label=TEST_TANK_YELLOW_ID,
        )
        pinger_xy = _parse_xy_unclamped(
            positions.get(TEST_TANK_PINGER_ID, current["pinger"]),
            label=TEST_TANK_PINGER_ID,
        )
        try:
            config_backup = save_course_layout_config(
                COURSE_LAYOUT_CONFIG_PATH,
                mode=COURSE_MODE_TEST_TANK,
                robot_xy=clean_robot_xy,
                yellow_xy=yellow_xy,
                pinger_xy=pinger_xy,
            )
            config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
            generate_test_tank_scene(
                base_scene_path=COURSE_SCENE_PATH,
                output_scene_path=TEST_TANK_SCENE_PATH,
                config=config,
            )
        except Exception as exc:
            self.course_status = f"test tank layout save failed: {exc}"
            self.node.push_event(f"web test tank layout save failed: {exc}")
            raise ValueError(self.course_status) from exc
        saved_status = "test tank layout: saved; restarting sim" if reset else "test tank layout: saved; restart sim to reload"
        self.course_status = saved_status
        self.node.push_event(f"web test tank layout saved: {COURSE_LAYOUT_CONFIG_PATH}")
        loaded = self.load_course_layout(COURSE_MODE_TEST_TANK)
        self.course_status = saved_status
        loaded.update(
            {
                "backup_path": str(config_backup) if config_backup else "",
                "status": saved_status,
                "reset": bool(reset),
            }
        )
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
            backup_path = backup_file(path, category="physics" if kind == "physics" else "course")
            atomic_write_text(path, content)
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
    row = {
        "key": key,
        "label": str(spec["label"]),
        "kind": str(spec.get("kind", "scalar")),
        "value": _format_physics_value(value),
        "default": _format_physics_value(spec["default"]),
        "description": str(spec["description"]),
        "inactive": bool(_physics_param_inactive_in_current(key)),
        "mode_status": _physics_current_mode_status(key),
    }
    limits = SIM_PROFILE_PARAM_LIMITS.get(key)
    if limits is not None:
        row.update({"minimum": limits[0], "maximum": limits[1]})
    return row


def _parse_web_physics_value(spec: dict[str, Any], value: Any) -> Any:
    key = str(spec["key"])
    label = str(spec["label"])
    raw = str(value).strip()
    kind = str(spec.get("kind", "scalar"))
    vector_match = re.fullmatch(r"vector(\d+)", kind)
    if not vector_match:
        return validate_profile_param_value(key, _finite_float(raw, label), label=label)
    expected_len = int(vector_match.group(1))
    pieces = [piece for piece in re.split(r"[,\s]+", raw) if piece]
    if len(pieces) != expected_len:
        raise ValueError(f"{spec['label']} must have exactly {expected_len} numbers")
    parsed = [_finite_float(piece, f"{label}[{idx}]") for idx, piece in enumerate(pieces)]
    return validate_profile_param_value(key, parsed, label=label)


def _finite_float(value: Any, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _parse_xy(value: Any, *, label: str) -> tuple[float, float]:
    x, y = _parse_xy_unclamped(value, label=label)
    return clamp_xy(x, y)


def _parse_xy_unclamped(value: Any, *, label: str) -> tuple[float, float]:
    if isinstance(value, dict):
        x_raw = value.get("x")
        y_raw = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        x_raw = value[0]
        y_raw = value[1]
    else:
        raise ValueError(f"{label} XY must have x and y")
    return _finite_float(x_raw, f"{label}.x"), _finite_float(y_raw, f"{label}.y")


def _course_mode_options() -> list[dict[str, str]]:
    return [
        {"id": COURSE_MODE_TEST_TANK, "label": "Test tank"},
        {"id": COURSE_MODE_COMPETITION, "label": "Competition course"},
    ]


def _backup_physics_profile_file() -> Path:
    return backup_file(PHYSICS_PROFILE_PATH, category="physics")


__all__ = ["WebToolFileManager"]
