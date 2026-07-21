"""Types for initial-depth runtime contracts."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class InitialBar30Depth:
    value_m: float | None
    label: str


__all__ = ["InitialBar30Depth"]
