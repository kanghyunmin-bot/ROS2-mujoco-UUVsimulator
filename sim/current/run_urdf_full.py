#!/usr/bin/env python3
"""Compatibility entrypoint for the renamed UUV MuJoCo runner.

Use ``run_uuv_mujoco.py`` for new scripts.  This wrapper keeps older launchers,
debug tooling, and saved command lines working while the runtime naming is
cleaned up.
"""

from __future__ import annotations

from run_uuv_mujoco import main


if __name__ == "__main__":
    main()
