"""ArduPilot source identity Check builders."""

from __future__ import annotations

from audit_code_contract_ardupilot_identity_info import ArduPilotIdentity
from audit_code_contract_types import Check, Evidence


def build_source_tag_check(identity: ArduPilotIdentity) -> Check:
    clean = identity.status_short == ""
    return Check(
        check_id="ardupilot_source_tag",
        status="PASS" if identity.describe == "ArduSub-4.1.2" and clean else "WARN",
        title="Local ArduPilot source identity",
        conclusion=(
            f"Local ArduPilot reports {identity.describe!r}; inner worktree status is "
            f"{'clean' if clean else 'dirty'}. Use a fresh clone only if this changes."
        ),
        evidence=[
            Evidence(path="ardupilot", line=None, snippet=f"git describe: {identity.describe}"),
            Evidence(path="ardupilot", line=None, snippet=f"git status --short: {identity.status_short or '<clean>'}"),
        ],
        official_refs=[],
    )


def build_gitlink_check(identity: ArduPilotIdentity) -> Check:
    return Check(
        check_id="top_level_ardupilot_gitlink",
        status="PASS" if identity.root_gitlink_commit == identity.inner_commit else "WARN",
        title="Top-level ArduPilot gitlink matches the working checkout",
        conclusion=(
            "The ArduPilot source checkout used for this audit is "
            f"{identity.inner_commit or '<unknown>'}. The top-level repository records "
            f"{identity.root_gitlink_commit or '<unknown>'}. "
            "If these differ, do not commit the gitlink change unless the project "
            "intentionally updates the submodule pointer."
        ),
        evidence=[
            Evidence(path="ardupilot", line=None, snippet=f"working checkout commit: {identity.inner_commit or '<unknown>'}"),
            Evidence(path=".", line=None, snippet=f"top-level gitlink: {identity.root_gitlink_commit or '<unknown>'}"),
        ],
        official_refs=[],
    )


__all__ = ["build_gitlink_check", "build_source_tag_check"]
