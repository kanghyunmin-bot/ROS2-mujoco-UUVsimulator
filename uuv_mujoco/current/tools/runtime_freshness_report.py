"""Report active-runtime freshness results."""

from __future__ import annotations

import json
from typing import Any


def print_text_report(report: dict[str, Any]) -> None:
    status = report["status"].upper()
    print(f"[uuv_mujoco] runtime freshness: {status}")
    print(f"[uuv_mujoco] {report['active_alias']}")
    print(f"[uuv_mujoco] branch={report['branch'] or '<unknown>'}")
    print(f"[uuv_mujoco] HEAD={report['head'] or '<unknown>'}")
    print(f"[uuv_mujoco] {report['remote_ref']}={report['remote_head'] or '<unknown>'}")
    print(f"[uuv_mujoco] dirty_paths={report.get('working_tree_dirty_count', 0)}")
    print(f"[uuv_mujoco] active_runtime_dirty_paths={report.get('active_runtime_dirty_count', 0)}")
    ardupilot_status = str(report.get("ardupilot_submodule_status") or "").strip()
    if ardupilot_status:
        print(f"[uuv_mujoco] ardupilot_submodule={ardupilot_status}")
    refresh = report.get("version_refresh")
    if refresh:
        action = "updated" if refresh.get("changed") else "current"
        if refresh.get("skipped"):
            action = f"skipped ({refresh['skipped']})"
        print(f"[uuv_mujoco] RUNTIME_VERSION={action}: {refresh.get('path')}")
    for issue in report["issues"]:
        print(f"[uuv_mujoco] {issue['level'].upper()} {issue['id']}: {issue['detail']}")


def print_json_report(report: dict[str, Any]) -> None:
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
