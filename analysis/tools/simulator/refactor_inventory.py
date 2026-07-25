#!/usr/bin/env python3
"""Inventory active-runtime code ownership and complexity.

This tool is intentionally read-only unless `--output` is provided.  It helps
the refactor proceed by facts instead of visual guesses.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from refactor_inventory_analysis import analyze_python_file, iter_python_files
from refactor_inventory_render import render_markdown
from refactor_inventory_types import DEFAULT_EXCLUDES


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--format", choices=("json", "markdown"), default="markdown")
    parser.add_argument("--limit", type=int, default=25)
    parser.add_argument("--include-debug", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    root = args.root.resolve()
    excludes = set(DEFAULT_EXCLUDES)
    if not args.include_debug:
        excludes.add("debug")

    items = [analyze_python_file(path, root) for path in iter_python_files(root, excludes)]
    if args.format == "json":
        payload = json.dumps([asdict(item) for item in items], indent=2, sort_keys=True)
    else:
        payload = render_markdown(items, args.limit)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    else:
        print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
