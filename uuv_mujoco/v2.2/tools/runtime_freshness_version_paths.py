"""Path payload helpers for active-runtime version metadata."""

from __future__ import annotations

from typing import Any

from runtime_freshness_version_constants import ROOT_LAUNCHERS


def backing_directory(active_alias_text: str, existing: dict[str, Any]) -> str:
    if active_alias_text and not active_alias_text.startswith("<") and not active_alias_text.startswith("/"):
        return f"uuv_mujoco/{active_alias_text}"
    return str(existing.get("backing_directory") or "")


def root_launchers() -> dict[str, str]:
    return dict(ROOT_LAUNCHERS)


__all__ = ["backing_directory", "root_launchers"]
