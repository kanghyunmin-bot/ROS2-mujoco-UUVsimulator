#!/usr/bin/env python3
"""Keep vehicle pinger control and hydrophone DSP in separate Git repositories."""

from __future__ import annotations

import re
from pathlib import Path

import yaml


EXPECTED_HYDROPHONE_URL = (
    "https://github.com/2026-kmu-underwater-robot/kmu26_auv_hydrophone.git"
)


def main() -> int:
    package_root = Path(__file__).resolve().parents[1]
    # The Git root is intentionally the ROS package root.  Keeping this
    # identical avoids a clone/src/kmu26_pinger_homing/kmu26_pinger_homing
    # double directory while retaining the external hydrophone boundary.
    manifest_path = package_root / "hydrophone.repos"

    assert manifest_path.is_file(), "root hydrophone.repos is missing"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    hydrophone = manifest["repositories"]["kmu26_auv_hydrophone"]
    assert hydrophone["type"] == "git"
    assert hydrophone["url"] == EXPECTED_HYDROPHONE_URL
    assert re.fullmatch(r"[0-9a-f]{40}", str(hydrophone["version"]))

    # The pinger package consumes the DSP result over ROS topics. It must not
    # grow a nested copy of the external hydrophone packages or their estimator.
    forbidden_directories = {"audio_capture", "audio_common", "audio_common_msgs"}
    nested_directories = {
        path.name for path in package_root.rglob("*") if path.is_dir()
    }
    assert not forbidden_directories.intersection(nested_directories)
    assert not list(package_root.rglob("audio_phase_estimator.cpp"))
    nested_git_dirs = [
        path for path in package_root.rglob(".git")
        if path != package_root / ".git"
    ]
    assert not nested_git_dirs

    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>audio_capture</exec_depend>" in package_xml

    print(
        "pinger_hydrophone_repository_boundary=PASS "
        "pinger=vehicle_control hydrophone=external_team_fork"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
