"""Subprocess import probe for runtime Python candidates."""

from __future__ import annotations

import json
from pathlib import Path

from dev_os_compat_exec import run_command


def python_import_probe_code(*, require_viewer: bool) -> str:
    return r"""
import importlib
import json
import sys

payload = {
    "executable": sys.executable,
    "version": [sys.version_info.major, sys.version_info.minor, sys.version_info.micro],
}
try:
    mujoco = importlib.import_module("mujoco")
    payload["mujoco_version"] = getattr(mujoco, "__version__", "unknown")
    payload["mujoco_ok"] = True
except Exception as exc:
    payload["mujoco_ok"] = False
    payload["mujoco_error"] = str(exc)
if """ + repr(bool(require_viewer)) + r""":
    try:
        importlib.import_module("mujoco.viewer")
        payload["viewer_ok"] = True
    except Exception as exc:
        payload["viewer_ok"] = False
        payload["viewer_error"] = str(exc)
print(json.dumps(payload, sort_keys=True))
"""


def parse_probe_output(output: str) -> tuple[dict[str, object] | None, int]:
    try:
        return json.loads(output), 0
    except json.JSONDecodeError:
        return None, 2


def probe_python(python_bin: Path, *, require_viewer: bool) -> tuple[int, dict[str, object] | None, str]:
    code = python_import_probe_code(require_viewer=require_viewer)
    code_status, output = run_command([str(python_bin), "-c", code], timeout_s=8.0)
    if code_status != 0:
        return code_status, None, output
    payload, parse_status = parse_probe_output(output)
    return parse_status, payload, output


__all__ = ["parse_probe_output", "probe_python", "python_import_probe_code"]
