#!/usr/bin/env python3
"""Create a Ping360 visual mesh with the cable assembly removed.

The official assembly STL includes a long external cable. For simulation the
sonar body should be mounted on the vehicle, while the cable would create a
large irrelevant visual/collision extent. This script keeps the upper sonar
body components and removes long, slender cable-like components.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from filter_ping360_stl_pipeline import filter_ping360_stl


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Source Ping360 assembly STL")
    parser.add_argument("output", type=Path, help="Filtered output STL")
    parser.add_argument("--metadata", type=Path, default=None, help="Optional JSON metadata output")
    parser.add_argument("--keep-min-z-mm", type=float, default=840.0)
    parser.add_argument("--cable-min-length-mm", type=float, default=500.0)
    parser.add_argument("--cable-max-thickness-mm", type=float, default=8.0)
    parser.add_argument("--quantize", type=float, default=10000.0)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    try:
        metadata = filter_ping360_stl(
            args.input,
            args.output,
            keep_min_z_mm=args.keep_min_z_mm,
            cable_min_length_mm=args.cable_min_length_mm,
            cable_max_thickness_mm=args.cable_max_thickness_mm,
            quantize=args.quantize,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if args.metadata is not None:
        args.metadata.parent.mkdir(parents=True, exist_ok=True)
        args.metadata.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print(
        "kept "
        f"{metadata['kept_triangle_count']}/{metadata['source_triangle_count']} triangles; "
        f"bbox_mm={metadata['output_size_mm']}; "
        f"offset_mm={metadata['source_offset_removed_mm']}"
    )


if __name__ == "__main__":
    main()
