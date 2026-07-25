"""Shared config helpers for Fossen residual runtime builders."""

from __future__ import annotations

from typing import Any, Callable, Optional


def config_group(cfg: dict[str, Any], name: str) -> dict[str, Any]:
    raw = cfg.get(name, {})
    return raw if isinstance(raw, dict) else {}


def log_message(log: Optional[Callable[[str], None]], message: str) -> None:
    if log is not None:
        log(message)


__all__ = ["config_group", "log_message"]
