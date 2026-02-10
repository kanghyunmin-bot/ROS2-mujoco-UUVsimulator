#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if command -v tree >/dev/null 2>&1; then
  tree -a -L 3 -I "__pycache__|*.pyc|*.csv"
else
  find . -maxdepth 3 \
    ! -path "*/__pycache__/*" \
    ! -name "*.pyc" \
    ! -name "*.csv" \
    | sort
fi
