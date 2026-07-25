"""JSON payload IO for thruster performance curves."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def read_performance_payload(perf_path: Path) -> dict[str, Any] | None:
    if not perf_path.exists():
        print(f"[thruster perf] file not found: {perf_path}", flush=True)
        return None
    try:
        payload = json.loads(perf_path.read_text())
    except (OSError, json.JSONDecodeError):
        print(f"[thruster perf] invalid json: {perf_path}", flush=True)
        return None
    if not isinstance(payload, dict):
        print(f"[thruster perf] missing curves in: {perf_path}", flush=True)
        return None
    if not isinstance(payload.get("curves"), list):
        print(f"[thruster perf] missing curves in: {perf_path}", flush=True)
        return None
    return payload


def log_loaded_curve(perf_path: Path, *, selected_voltage: float, requested: float, direct: bool) -> None:
    print(
        f"[thruster perf] loaded curve {selected_voltage}V from {perf_path} "
        f"(requested {requested}V)",
        flush=True,
    )
    if direct:
        print(
            "[thruster perf] direct mode: raw PWM command -> T200 curve; "
            "profile polynomial shaping and thruster gain scaling bypassed",
            flush=True,
        )


__all__ = ["log_loaded_curve", "read_performance_payload"]
