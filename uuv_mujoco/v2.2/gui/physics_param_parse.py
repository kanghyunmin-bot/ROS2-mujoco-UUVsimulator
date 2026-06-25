"""User-input parsing helpers for GUI physics parameters."""

from __future__ import annotations

import math
import re
from typing import Any

from .physics_param_format import _format_physics_value


def _parse_physics_number(value: float, label: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def _parse_physics_value(self, spec: dict[str, Any]) -> Any:
    key = str(spec["key"])
    raw = self.physics_param_vars[key].get().strip()
    label = str(spec["label"])
    kind = str(spec.get("kind", "scalar"))
    vector_len = _vector_length(kind)
    if vector_len is not None:
        return _parse_physics_vector(self, key=key, raw=raw, label=label, expected_len=vector_len)
    return _parse_physics_scalar(self, key=key, raw=raw, label=label)


def _vector_length(kind: str) -> int | None:
    vector_match = re.fullmatch(r"vector(\d+)", kind)
    if not vector_match:
        return None
    return int(vector_match.group(1))


def _parse_physics_vector(self, *, key: str, raw: str, label: str, expected_len: int) -> list[float]:
    pieces = [piece for piece in re.split(r"[,\s]+", raw) if piece]
    if len(pieces) != expected_len:
        raise ValueError(f"{label} must have exactly {expected_len} numbers")
    values = [_parse_physics_number(float(piece), f"{label}[{idx}]") for idx, piece in enumerate(pieces)]
    self.physics_param_vars[key].set(_format_physics_value(values))
    return values


def _parse_physics_scalar(self, *, key: str, raw: str, label: str) -> float:
    value = _parse_physics_number(float(raw), label)
    self.physics_param_vars[key].set(_format_physics_value(value))
    return value


__all__ = ["_parse_physics_number", "_parse_physics_value"]
