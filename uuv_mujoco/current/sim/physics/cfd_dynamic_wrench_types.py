"""Typed contracts for CFD-derived dynamic wrench runtime."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CfdDynamicWrenchRuntime:
    enabled: bool
    scale: float
    debug: bool
    axes: dict[str, dict[str, object]]
    source: str

    @property
    def owns_z(self) -> bool:
        return bool(self.enabled and "z" in self.axes)


__all__ = ["CfdDynamicWrenchRuntime"]
