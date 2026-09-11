"""Snapshot local code/config identity without modifying the worktree.

Pass the actual scene, sensor, physical parameter and launch/YAML files used.
A snapshot records assumptions, not a certification of hardware calibration.
"""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--repository", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--files", type=Path, nargs="+", required=True)
p.add_argument("--launch_description", required=True)
a = p.parse_args()
root = a.repository.resolve()


def git(*args):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


result = {
    "git_commit": git("rev-parse", "HEAD"),
    "git_status": git("status", "--porcelain"),
    "launch_description": a.launch_description,
    "calibration_status": "unvalidated unless explicitly documented in source files",
    "files": {},
}
for path in a.files:
    absolute = path if path.is_absolute() else root / path
    data = absolute.read_bytes()
    result["files"][str(path)] = {
        "sha256": hashlib.sha256(data).hexdigest(),
        "content": data.decode("utf-8"),
    }
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(result, indent=2) + "\n")
