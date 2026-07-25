"""Shared constants and helpers for actuator wrench audits."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[3] / "sim" / "current"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

AXES = ("roll", "pitch", "yaw", "heave", "forward", "lateral")
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    return out if math.isfinite(out) else float(default)


__all__ = ["AXES", "FLU_TO_FRD", "ROOT", "finite_float", "load_json"]
