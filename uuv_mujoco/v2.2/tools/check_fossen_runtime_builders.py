#!/usr/bin/env python3
"""Smoke checks for residual hydrodynamics runtime builders."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fossen_runtime_builder_smoke_cases import (  # noqa: E402
    check_fossen_residual_runtime,
    check_residual_hydro_runtime,
)


def main() -> int:
    check_residual_hydro_runtime()
    check_fossen_residual_runtime()
    print("fossen_runtime_builders=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
