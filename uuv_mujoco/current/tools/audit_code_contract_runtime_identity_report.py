"""Evidence and metadata builders for active-runtime identity checks."""

from __future__ import annotations

import json
import os

from audit_code_contract_common import rel
from audit_code_contract_runtime_identity_inputs import RuntimeIdentityInputs
from audit_code_contract_types import Evidence


def active_runtime_identity_evidence(inputs: RuntimeIdentityInputs, result: dict[str, bool | str]) -> list[Evidence]:
    return [
        Evidence(path="uuv_mujoco/current", line=None, snippet=inputs.alias_text),
        Evidence(
            path=rel(inputs.current_runner),
            line=None,
            snippet=f"primary runner exists: {inputs.current_runner.exists()}",
        ),
        *_launcher_evidence(inputs.root_launchers),
        Evidence(
            path=rel(inputs.runtime_version_path),
            line=None,
            snippet=f"runtime version: {json.dumps(inputs.runtime_version, ensure_ascii=False, sort_keys=True)}",
        ),
        Evidence(
            path=rel(inputs.freshness_script),
            line=None,
            snippet=f"freshness checker exists: {bool(result['freshness_ready'])}",
        ),
        Evidence(
            path="uuv_mujoco/*.sh",
            line=None,
            snippet=f"root launchers call freshness checker: {bool(result['launchers_call_freshness'])}",
        ),
        Evidence(path=".", line=None, snippet=f"active git branch: {inputs.repo_branch or '<unknown>'}"),
        Evidence(path=".", line=None, snippet=f"active git HEAD: {inputs.repo_head or '<unknown>'}"),
        Evidence(path=".", line=None, snippet=f"origin/uuv_sim HEAD: {inputs.origin_uuv_sim or '<unknown>'}"),
        Evidence(
            path=".",
            line=None,
            snippet=f"active runtime dirty paths: {inputs.repo_runtime_dirty or '<clean>'}",
        ),
    ]


def active_runtime_identity_metadata(inputs: RuntimeIdentityInputs) -> dict[str, str]:
    return {
        "active_runtime_alias": inputs.alias_text,
        "active_runtime_root": inputs.active_root,
        "runtime_version_file": str(inputs.runtime_version_path),
        "active_git_branch": inputs.repo_branch,
        "active_git_head": inputs.repo_head,
        "origin_uuv_sim_head": inputs.origin_uuv_sim,
    }


def _launcher_evidence(root_launchers) -> list[Evidence]:
    return [
        Evidence(
            path=rel(path),
            line=None,
            snippet=f"root launcher exists/executable: {path.exists() and os.access(path, os.X_OK)}",
        )
        for path in root_launchers
    ]


__all__ = [
    "active_runtime_identity_evidence",
    "active_runtime_identity_metadata",
]
