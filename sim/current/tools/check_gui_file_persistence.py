#!/usr/bin/env python3
"""Regression checks for atomic GUI writes and bounded generated backups."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui import file_persistence  # noqa: E402


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="uuv_gui_persistence_") as temp_dir:
        temp = Path(temp_dir)
        runtime = temp / "current"
        generated_backups = runtime / "generated" / "backups"
        profile = runtime / "config" / "sim_profiles.json"

        with (
            mock.patch.object(file_persistence, "SIM_STACK_DIR", runtime),
            mock.patch.object(file_persistence, "GENERATED_BACKUP_ROOT", generated_backups),
        ):
            file_persistence.atomic_write_text(profile, "revision=0\n")
            profile.chmod(0o640)
            for revision in range(1, 8):
                backup = file_persistence.backup_file(
                    profile,
                    category="physics",
                    retention=3,
                )
                if backup.parent != generated_backups / "physics":
                    raise AssertionError(f"active profile backup escaped generated tree: {backup}")
                file_persistence.atomic_write_text(profile, f"revision={revision}\n")

            backups = sorted((generated_backups / "physics").glob("sim_profiles.json.*.bak"))
            if len(backups) != 3:
                raise AssertionError(f"backup retention expected 3 files, got {len(backups)}")
            if profile.read_text(encoding="utf-8") != "revision=7\n":
                raise AssertionError("atomic profile write lost the final payload")
            if profile.stat().st_mode & 0o777 != 0o640:
                raise AssertionError("atomic profile write changed file permissions")
            if list(profile.parent.glob("*.bak*")):
                raise AssertionError("GUI backup leaked into active config directory")
            if list(profile.parent.glob(".*.tmp")):
                raise AssertionError("atomic profile write left a temporary file")
            if {path.read_text(encoding="utf-8") for path in backups} != {
                "revision=4\n",
                "revision=5\n",
                "revision=6\n",
            }:
                raise AssertionError("bounded backups do not contain the newest complete revisions")

        external = temp / "external" / "course_layout.json"
        file_persistence.atomic_write_text(external, "{}\n")
        external_backup = file_persistence.backup_file(external, category="course")
        if external_backup.parent != external.parent / ".backups" / "course":
            raise AssertionError(f"external fixture backup used wrong directory: {external_backup}")

    print("gui_file_persistence=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
