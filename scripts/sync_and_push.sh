#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."
cd "$ROOT_DIR"

run_repo() {
  local path="$1"
  local branch="$2"

  echo "== $path ($branch) =="
  cd "$ROOT_DIR/$path"

  git switch "$branch"
  git fetch upstream
  git pull --rebase upstream "$branch"
  git push origin "$branch"
}

echo "[1/3] Sync mujoco (main)"
run_repo mujoco main

echo "[2/3] Sync ArduPilot (master)"
run_repo kmu_hit25_ros2_ws/ardupilot master

echo "[3/3] Update super-repo submodule pointers"
cd "$ROOT_DIR"
git add mujoco kmu_hit25_ros2_ws/ardupilot
if ! git diff --cached --quiet; then
  git commit -m "update submodule pointers after sync"
  git push origin master
else
  echo "No submodule pointer changes."
fi

echo "Done."
