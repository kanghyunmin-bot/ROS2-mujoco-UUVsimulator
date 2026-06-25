#!/usr/bin/env python3
"""Smoke-check the GUI entry module under the same ROS Python contract."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.ros_bash import ros_bash_command  # noqa: E402


def main() -> int:
    command = "\n".join(
        [
            "unset UUV_GUI_PILOT_CONTROL_MODE",
            "unset UUV_GUI_DEFAULT_PILOT_CONTROL_MODE",
            "python - <<'PY'",
            "import pathlib",
            "import sys",
            "root = pathlib.Path.cwd()",
            "sys.path.insert(0, str(root))",
            "from gui.config_backend import GUI_PILOT_CONTROL_MODE",
            "if GUI_PILOT_CONTROL_MODE != 'rc_override':",
            "    raise AssertionError(f'GUI default pilot mode is {GUI_PILOT_CONTROL_MODE!r}')",
            "import gui.uuv_control_gui",
            "print('pilot_mode=' + GUI_PILOT_CONTROL_MODE)",
            "print('gui_entry_import=PASS')",
            "PY",
        ]
    )
    completed = subprocess.run(
        ros_bash_command(command, cwd=ROOT),
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=10,
        check=False,
    )
    if completed.returncode != 0:
        raise AssertionError(
            "GUI entry contract failed under ROS Python.\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    print(completed.stdout.strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
