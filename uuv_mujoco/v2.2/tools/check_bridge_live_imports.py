#!/usr/bin/env python3
"""Import smoke for bridge modules used by live SITL/GUI paths."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main() -> int:
    import bridge.sitl_arm_mode_queue  # noqa: F401
    import bridge.sitl_arm_mode_runtime  # noqa: F401
    import bridge.sitl_transport  # noqa: F401
    import bridge.ros2_bridge  # noqa: F401
    import bridge.ping360_sim  # noqa: F401

    print("bridge_live_imports=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
