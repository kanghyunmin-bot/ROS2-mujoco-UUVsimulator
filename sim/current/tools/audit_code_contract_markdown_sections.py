"""Markdown section builders for source contract audit reports."""

from __future__ import annotations

from audit_code_contract_types import Check


def metadata_lines(metadata: dict[str, object]) -> list[str]:
    runtime_root = metadata.get("active_runtime_root", metadata.get("runtime_root", ""))
    lines = ["## Metadata", ""]
    lines.append(f"- repo_root: `{metadata['repo_root']}`")
    lines.append(f"- active_runtime_root: `{runtime_root}`")
    if metadata.get("compat_v22_root"):
        lines.append(f"- compat_v22_root: `{metadata['compat_v22_root']}`")
    lines.append(f"- ardupilot_describe: `{metadata['ardupilot_describe']}`")
    lines.append(f"- ardupilot_status_short: `{metadata['ardupilot_status_short'] or '<clean>'}`")
    return lines


def summary_lines(checks: list[Check]) -> list[str]:
    lines = ["## Summary", "", "| status | check | conclusion |", "| --- | --- | --- |"]
    for check in checks:
        lines.append(f"| {check.status} | `{check.check_id}` | {check.conclusion} |")
    return lines


def evidence_lines(checks: list[Check]) -> list[str]:
    lines = ["## Evidence", ""]
    for check in checks:
        lines.extend(check_evidence_lines(check))
    return lines


def check_evidence_lines(check: Check) -> list[str]:
    lines = [
        f"### {check.status}: {check.title}",
        "",
        f"- id: `{check.check_id}`",
        f"- conclusion: {check.conclusion}",
    ]
    if check.official_refs:
        refs = ", ".join(f"<{ref}>" for ref in check.official_refs)
        lines.append(f"- official refs: {refs}")
    lines.append("- local evidence:")
    for item in check.evidence:
        suffix = f":{item.line}" if item.line is not None else ""
        lines.append(f"  - `{item.path}{suffix}` - {item.snippet}")
    lines.append("")
    return lines


__all__ = ["evidence_lines", "metadata_lines", "summary_lines"]
