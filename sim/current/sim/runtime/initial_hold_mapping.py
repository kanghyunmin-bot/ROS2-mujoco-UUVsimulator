"""Dict-style compatibility methods for initial-depth hold state."""

from __future__ import annotations

from typing import Any


class InitialHoldMappingMixin:
    def __getitem__(self, key: str) -> Any:
        return getattr(self, key)

    def __setitem__(self, key: str, value: Any) -> None:
        setattr(self, key, value)

    def get(self, key: str, default: Any = None) -> Any:
        return getattr(self, key, default)


__all__ = ["InitialHoldMappingMixin"]
