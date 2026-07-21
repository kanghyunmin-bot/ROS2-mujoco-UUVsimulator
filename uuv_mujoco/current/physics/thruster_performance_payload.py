"""JSON payload loading for offline thruster performance curves."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def read_thruster_performance_payload(perf_path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not perf_path.exists():
        return None, f"[thruster perf] file not found: {perf_path}"
    try:
        payload = json.loads(perf_path.read_text())
    except (OSError, json.JSONDecodeError):
        return None, f"[thruster perf] invalid json: {perf_path}"
    if not isinstance(payload, dict) or not isinstance(payload.get("curves"), list):
        return None, f"[thruster perf] missing curves in: {perf_path}"
    return payload, None


__all__ = ["read_thruster_performance_payload"]
