"""Input collection for active-runtime identity checks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from audit_code_contract_common import (
    ACTIVE_RUNTIME_ALIAS,
    ACTIVE_RUNTIME_ROOT,
    REPO_ROOT,
    git_output,
)
from audit_code_contract_runtime_identity_sources import (
    alias_status,
    alias_text,
    root_launchers,
    runtime_dirty_paths,
    runtime_version_payload,
)


@dataclass(frozen=True)
class RuntimeIdentityInputs:
    active_root: str
    current_runner: Path
    runtime_version_path: Path
    root_launchers: list[Path]
    freshness_script: Path
    repo_branch: str
    repo_head: str
    origin_uuv_sim: str
    repo_runtime_dirty: str
    alias_text: str
    alias_status: str
    runtime_version: dict[str, Any]


def collect_runtime_identity_inputs() -> RuntimeIdentityInputs:
    current_runner = ACTIVE_RUNTIME_ALIAS / "run_uuv_mujoco.py"
    runtime_version_path = REPO_ROOT / "sim" / "current" / "RUNTIME_VERSION.json"
    return RuntimeIdentityInputs(
        active_root=str(ACTIVE_RUNTIME_ROOT),
        current_runner=current_runner,
        runtime_version_path=runtime_version_path,
        root_launchers=root_launchers(),
        freshness_script=ACTIVE_RUNTIME_ALIAS / "tools" / "check_runtime_freshness.py",
        repo_branch=git_output(["rev-parse", "--abbrev-ref", "HEAD"], REPO_ROOT),
        repo_head=git_output(["rev-parse", "HEAD"], REPO_ROOT),
        origin_uuv_sim=git_output(["rev-parse", "origin/uuv_sim"], REPO_ROOT),
        repo_runtime_dirty=runtime_dirty_paths(),
        alias_text=alias_text(),
        alias_status=alias_status(current_runner),
        runtime_version=runtime_version_payload(runtime_version_path),
    )
