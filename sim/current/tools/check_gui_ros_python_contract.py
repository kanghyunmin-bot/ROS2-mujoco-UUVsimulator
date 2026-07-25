#!/usr/bin/env python3
"""Check the GUI ROS shell uses a Python interpreter that can import rclpy."""

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
            "python - <<'PY'",
            "import sys",
            "import rclpy",
            "print(sys.executable)",
            "print(f'{sys.version_info.major}.{sys.version_info.minor}')",
            "print(rclpy.__file__)",
            "PY",
        ]
    )
    completed = subprocess.run(
        ros_bash_command(command),
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=10,
        check=False,
    )
    if completed.returncode != 0:
        raise AssertionError(
            "GUI ROS Python contract failed: use `python`, not bare `python3`, "
            "inside ros_bash_command subprocesses.\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    assert len(lines) >= 3, f"unexpected ROS Python probe output: {completed.stdout!r}"
    print("gui_ros_python_contract=PASS")
    print(f"python={lines[-3]}")
    print(f"version={lines[-2]}")
    print(f"rclpy={lines[-1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
