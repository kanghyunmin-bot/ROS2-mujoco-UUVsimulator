"""Dirty working-tree payload helpers for runtime version metadata."""

from __future__ import annotations

from typing import Any


def runtime_dirty_state(inputs: dict[str, Any]) -> dict[str, Any]:
    return {
        "working_tree_dirty_count": int(inputs.get("working_tree_dirty_count") or 0),
        "active_runtime_dirty_count": int(inputs.get("active_runtime_dirty_count") or 0),
        "working_tree_dirty_sample": list(inputs.get("working_tree_dirty_sample") or []),
        "active_runtime_dirty_sample": list(inputs.get("active_runtime_dirty_sample") or []),
        "ardupilot_submodule_status": str(inputs.get("ardupilot_submodule_status") or ""),
    }


def runtime_status_from_dirty_state(dirty_state: dict[str, Any]) -> str:
    return "current-dirty" if int(dirty_state.get("working_tree_dirty_count") or 0) else "current"


__all__ = ["runtime_dirty_state", "runtime_status_from_dirty_state"]
