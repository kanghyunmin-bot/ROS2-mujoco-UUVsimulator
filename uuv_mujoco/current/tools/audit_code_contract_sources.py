#!/usr/bin/env python3
"""Audit code-level contracts against local ArduSub and the active runtime.

This is a source-contract audit, not a dynamic replay validator. It answers:

* what local ArduSub 4.1.2 actually consumes/emits in code,
* whether the active MuJoCo runtime follows that contract,
* which topics/signals must be excluded from tuning targets.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from audit_code_contract_checks import build_checks
from audit_code_contract_common import ACTIVE_RUNTIME_ROOT
from audit_code_contract_report import build_payload, write_json, write_markdown


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    default_out = ACTIVE_RUNTIME_ROOT / "research_workspace/00_current_contract"
    parser.add_argument("--out-dir", type=Path, default=default_out)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checks, metadata = build_checks()
    payload = build_payload(checks, metadata)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.out_dir / "contract_source_audit.json"
    md_path = args.out_dir / "contract_source_audit.md"
    write_json(json_path, payload)
    write_markdown(md_path, checks, metadata)

    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    print(json.dumps(payload["counts"], sort_keys=True))
    return 1 if payload["counts"]["fail"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
