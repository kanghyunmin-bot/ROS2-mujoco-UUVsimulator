#!/usr/bin/env python3
"""Apply/revert MuJoCo AUV stability candidates and measure response."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

from roll_stability_candidates import default_candidates
from roll_stability_sweep_files import read_original_file_texts
from roll_stability_sweep_loop import print_final_result, run_sweep_candidates
from roll_stability_sweep_paths import default_sweep_paths


PATHS = default_sweep_paths(Path(__file__))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--measure-s", type=float, default=8.0)
    parser.add_argument("--settle-s", type=float, default=4.0)
    parser.add_argument("--max-candidates", type=int, default=0, help="0 means all")
    parser.add_argument("--candidate", action="append", default=[])
    parser.add_argument("--include-sign-checks", action="store_true")
    parser.add_argument(
        "--stimulus",
        choices=("neutral", "heave-pulse", "forward-pulse", "sway-pulse", "roll-pulse", "pitch-pulse", "yaw-pulse"),
        default="neutral",
    )
    parser.add_argument("--axis-command", type=float, default=0.20)
    parser.add_argument("--pulse-s", type=float, default=None)
    parser.add_argument("--hold-mode", choices=("ALT_HOLD", "MANUAL"), default="ALT_HOLD")
    parser.add_argument("--skip-launcher-ready", action="store_true")
    parser.add_argument("--out-dir", type=Path)
    return parser.parse_args()


def select_candidates(args: argparse.Namespace) -> list[Any]:
    candidates = default_candidates(include_sign_checks=args.include_sign_checks)
    if args.candidate:
        wanted = set(args.candidate)
        candidates = [candidate for candidate in candidates if candidate.name in wanted]
    if args.max_candidates > 0:
        candidates = candidates[: args.max_candidates]
    if not candidates:
        raise RuntimeError("No candidates selected")
    return candidates


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir or (PATHS.log_root / f"roll_stability_sweep_{time.strftime('%Y%m%d_%H%M%S')}")
    out_dir.mkdir(parents=True, exist_ok=True)

    original_texts = read_original_file_texts(PATHS)
    candidates = select_candidates(args)

    print(f"[sweep] out={out_dir}", flush=True)
    print(f"[sweep] candidates={', '.join(c.name for c in candidates)}", flush=True)

    results = run_sweep_candidates(
        candidates=candidates,
        paths=PATHS,
        original_texts=original_texts,
        out_dir=out_dir,
        settle_s=args.settle_s,
        measure_s=args.measure_s,
        stimulus=args.stimulus,
        hold_mode=args.hold_mode,
        axis_command=args.axis_command,
        pulse_s_override=args.pulse_s,
        wait_ready=not args.skip_launcher_ready,
    )
    return print_final_result(results)


if __name__ == "__main__":
    raise SystemExit(main())
