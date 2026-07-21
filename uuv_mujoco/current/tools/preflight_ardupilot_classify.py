"""Classify ArduPilot preflight payloads."""

from __future__ import annotations

from typing import Any

from preflight_ardupilot_git import WATCHED_PATHS


def classify(payload: dict[str, Any]) -> tuple[bool, list[str]]:
    issues: list[str] = []
    failed = _classify_git_status(payload, issues)
    failed = _classify_submodule_pointer(payload, issues) or failed
    failed = _classify_althold_diff(payload, issues) or failed
    _classify_watched_dependency_status(payload, issues)
    return failed, issues


def _classify_git_status(payload: dict[str, Any], issues: list[str]) -> bool:
    status = payload.get("git_status", {})
    if status.get("returncode") == 0:
        return False
    issues.append("P0: ArduPilot git status failed; closed-loop validation cannot prove the controller baseline.")
    return True


def _classify_submodule_pointer(payload: dict[str, Any], issues: list[str]) -> bool:
    pointer = payload.get("submodule_pointer", {})
    recorded = str(pointer.get("recorded_sha", ""))
    current = str(pointer.get("current_sha", ""))
    if not recorded:
        issues.append("P0: ArduPilot submodule recorded SHA could not be read from the parent repository.")
        return True
    if not current:
        issues.append("P0: ArduPilot checkout HEAD could not be read.")
        return True
    if recorded == current:
        return False
    issues.append(
        "P0: ArduPilot checkout does not match the parent submodule pointer; "
        f"recorded={recorded[:12]}, current={current[:12]}. "
        "Closed-loop validation cannot prove firmware provenance."
    )
    return True


def _classify_althold_diff(payload: dict[str, Any], issues: list[str]) -> bool:
    althold = payload.get("watched_diffs", {}).get("ArduSub/control_althold.cpp", {})
    if not althold.get("dirty"):
        return False
    diff = str(althold.get("diff", ""))
    if "motors.set_throttle" in diff or "channel_throttle" in diff or "raw_throttle_factor" in diff:
        issues.append(
            "P0: ArduSub/control_althold.cpp modifies ALT_HOLD throttle/heave mapping; "
            "depth/heave closed-loop results are not trustworthy."
        )
    else:
        issues.append("P0: ArduSub/control_althold.cpp is modified; ALT_HOLD baseline is dirty.")
    return True


def _classify_watched_dependency_status(payload: dict[str, Any], issues: list[str]) -> None:
    status = payload.get("git_status", {})
    status_lines = [str(line) for line in status.get("short", [])]
    for rel_path in WATCHED_PATHS:
        if any(line.endswith(rel_path) for line in status_lines):
            if rel_path != "ArduSub/control_althold.cpp":
                issues.append(
                    f"P1: ArduPilot dependency file is modified: {rel_path}; "
                    "allowed for host build compatibility, but record it in the run report."
                )


__all__ = ["classify"]
