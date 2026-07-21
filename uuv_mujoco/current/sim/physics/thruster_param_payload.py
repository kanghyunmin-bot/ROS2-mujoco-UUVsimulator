"""JSON payload IO for thruster parameter files."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def read_thruster_param_payload(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text())
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def per_thruster_payload(payload: dict[str, Any]) -> dict[str, Any] | None:
    per_thruster = payload.get("per_thruster", {})
    return per_thruster if isinstance(per_thruster, dict) else None


def global_thruster_payload(payload: dict[str, Any]) -> dict[str, Any]:
    global_cfg = payload.get("global", {})
    return global_cfg if isinstance(global_cfg, dict) else {}


__all__ = ["global_thruster_payload", "per_thruster_payload", "read_thruster_param_payload"]
