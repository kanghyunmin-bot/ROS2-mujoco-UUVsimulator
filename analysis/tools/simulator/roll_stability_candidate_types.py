"""Typed records for roll stability sweep candidates."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class Candidate:
    name: str
    profile_updates: dict[str, Any] = field(default_factory=dict)
    fluid_angular_scale: float | None = None
    servo_signs: tuple[int, ...] | None = None
    note: str = ""


__all__ = ["Candidate"]
