"""Source probes for active-runtime identity checks."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from audit_code_contract_common import (
    ACTIVE_RUNTIME_ALIAS,
    COMPAT_V22_ROOT,
    REPO_ROOT,
    git_output,
)


def root_launchers() -> list[Path]:
    return [
        REPO_ROOT / "uuv_mujoco" / "run_mujoco.sh",
        REPO_ROOT / "uuv_mujoco" / "start_sitl_mujoco.sh",
        REPO_ROOT / "uuv_mujoco" / "start_docker_sitl_mujoco.sh",
        REPO_ROOT / "uuv_mujoco" / "reset_sim.sh",
    ]


def runtime_dirty_paths() -> str:
    return git_output(
        [
            "status",
            "--short",
            "--",
            "uuv_mujoco",
            ".uuv_mujoco_env.sh",
            "README.md",
            "setup",
            "uuv_control_gui.py",
            "docker",
        ],
        REPO_ROOT,
    )


def alias_text() -> str:
    if not ACTIVE_RUNTIME_ALIAS.exists():
        return "<missing>"
    try:
        target = os.readlink(ACTIVE_RUNTIME_ALIAS)
        return f"uuv_mujoco/current -> {target}"
    except OSError:
        return f"uuv_mujoco/current -> {ACTIVE_RUNTIME_ALIAS.resolve()}"


def alias_status(current_runner: Path) -> str:
    if not ACTIVE_RUNTIME_ALIAS.exists():
        return "FAIL"
    try:
        same_runtime = ACTIVE_RUNTIME_ALIAS.resolve() == COMPAT_V22_ROOT.resolve()
    except OSError:
        same_runtime = False
    return "PASS" if same_runtime and current_runner.exists() else "FAIL"


def runtime_version_payload(runtime_version_path: Path) -> dict[str, Any]:
    if not runtime_version_path.exists():
        return {}
    try:
        payload = json.loads(runtime_version_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


__all__ = [
    "alias_status",
    "alias_text",
    "root_launchers",
    "runtime_dirty_paths",
    "runtime_version_payload",
]
