"""Collect active-runtime freshness evidence."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from runtime_freshness_dirty_paths import active_runtime_dirty_paths, dirty_paths
from runtime_freshness_git_probe import git_text, run_git
from runtime_freshness_json_io import load_json
from runtime_freshness_runtime_resolve import alias_text, resolve_runtime_dir, same_path_text


DEFAULT_REMOTE_REF = "origin/uuv_sim"


def default_workspace() -> Path:
    return Path(__file__).resolve().parents[3]


def collect_freshness_inputs(
    *,
    workspace: Path,
    remote_ref: str,
    runtime_dir: Path | None,
    fetch: bool,
) -> dict[str, Any]:
    fetch_status: dict[str, Any] = {"attempted": fetch, "code": None, "output": ""}
    if fetch:
        code, output = run_git(workspace, ["fetch", "origin", "--prune", "--quiet"], timeout_s=15.0)
        fetch_status = {"attempted": True, "code": code, "output": output}

    active_alias = workspace / "sim" / "current"
    resolved_runtime = resolve_runtime_dir(workspace, runtime_dir)
    runtime_dir_uses_active_alias = same_path_text(resolved_runtime, active_alias)
    runtime_version_path = workspace / "sim" / "current" / "RUNTIME_VERSION.json"
    status_text = git_text(workspace, ["status", "--porcelain=v1"])
    dirty_path_list = dirty_paths(status_text)
    active_dirty_path_list = active_runtime_dirty_paths(dirty_path_list)
    ardupilot_status = git_text(workspace, ["submodule", "status", "--", "sim/ardupilot"])
    return {
        "workspace": str(workspace),
        "resolved_runtime": str(resolved_runtime),
        "runtime_dir_uses_active_alias": runtime_dir_uses_active_alias,
        "active_alias_exists": active_alias.exists(),
        "active_alias_path": str(active_alias),
        "active_alias_text": alias_text(active_alias),
        "runner_exists": (resolved_runtime / "run_uuv_mujoco.py").exists(),
        "runtime_version_path": str(runtime_version_path),
        "runtime_version": load_json(runtime_version_path),
        "branch": git_text(workspace, ["rev-parse", "--abbrev-ref", "HEAD"]),
        "head": git_text(workspace, ["rev-parse", "HEAD"]),
        "source_remote": git_text(workspace, ["config", "--get", "remote.origin.url"]),
        "remote_ref": remote_ref,
        "remote_head": git_text(workspace, ["rev-parse", remote_ref]),
        "working_tree_dirty_count": len(dirty_path_list),
        "working_tree_dirty_sample": dirty_path_list[:20],
        "active_runtime_dirty_count": len(active_dirty_path_list),
        "active_runtime_dirty_sample": active_dirty_path_list[:20],
        "ardupilot_submodule_status": ardupilot_status,
        "fetch": fetch_status,
    }
