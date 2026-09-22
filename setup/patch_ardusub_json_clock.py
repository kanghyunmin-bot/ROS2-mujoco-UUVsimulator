# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Apply the audited ArduSub 4.1.2 SITL JSON microsecond rounding fix."""

from __future__ import annotations

import argparse
from pathlib import Path

ORIGINAL = "time_now_us += deltat * 1.0e6;"
ROUNDED = "time_now_us += uint64_t(llround(deltat * 1.0e6));"


def patch_json_clock(source: str) -> str:
    """Round simulated elapsed time to microseconds [us] before accumulation."""
    if source.count(ROUNDED) == 1 and ORIGINAL not in source:
        return source
    if source.count(ORIGINAL) != 1 or ROUNDED in source:
        raise ValueError(
            "Unrecognized SITL JSON clock implementation; source was not changed"
        )
    return source.replace(ORIGINAL, ROUNDED)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ardupilot_dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "ardupilot_sub_stable",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Write the recognized one-line fix; rebuild SITL afterwards.",
    )
    args = parser.parse_args()
    path = args.ardupilot_dir / "libraries/SITL/SIM_JSON.cpp"
    source = path.read_text()
    patched = patch_json_clock(source)
    if source == patched:
        print(
            "SITL JSON clock: rounded source present (binary must be built from this source)"
        )
        return 0
    if not args.apply:
        print("SITL JSON clock: truncation present; use --apply and rebuild SITL")
        return 1
    path.write_text(patched)
    print(f"Patched {path}; rebuild the SITL binary before running")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
