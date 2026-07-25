"""Fingerprint construction for control-loop golden checks."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from control_loop_golden_fingerprint_metadata import build_fingerprint_metadata
from control_loop_golden_fingerprint_phase import build_phase_fingerprint
from control_loop_golden_thrusters import load_thruster_csv, summarize_thrusters


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def build_fingerprint(payload: dict[str, Any], thruster_csv: Path | None = None) -> dict[str, Any]:
    thruster_rows, fieldnames = load_thruster_csv(thruster_csv)
    thruster_by_phase = summarize_thrusters(payload, thruster_rows, fieldnames)
    return {
        "metadata": build_fingerprint_metadata(payload),
        "sample_count": payload.get("sample_count"),
        "health": payload.get("health", {}),
        "phases": build_phase_fingerprint(payload, thruster_by_phase),
        "has_thruster_debug": bool(thruster_by_phase),
    }
