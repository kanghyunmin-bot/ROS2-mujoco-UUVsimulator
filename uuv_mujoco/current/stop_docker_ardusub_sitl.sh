#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
COMPOSE_FILE="${UUV_ARDUSUB_DOCKER_COMPOSE:-${WORKSPACE_DIR}/docker/ardusub/docker-compose.yml}"

if [[ -f "$COMPOSE_FILE" ]]; then
  cd "$WORKSPACE_DIR"
  docker compose -f "$COMPOSE_FILE" down --remove-orphans
fi
