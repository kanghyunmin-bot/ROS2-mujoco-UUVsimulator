"""Common types and paths for development OS compatibility checks."""

from __future__ import annotations

import platform
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
HOME = Path.home()


@dataclass
class CheckResult:
    name: str
    status: str
    detail: str


def normalize_target_os(raw: str | None) -> str:
    text = (raw or "auto").strip().lower()
    if text in {"", "auto", "host", "current"}:
        return platform.system()
    if text in {"mac", "macos", "darwin"}:
        return "Darwin"
    if text in {"linux", "ubuntu"}:
        return "Linux"
    return text


__all__ = ["CheckResult", "HOME", "ROOT", "WORKSPACE", "normalize_target_os"]
