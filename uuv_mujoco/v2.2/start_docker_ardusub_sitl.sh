#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
COMPOSE_FILE="${UUV_ARDUSUB_DOCKER_COMPOSE:-${WORKSPACE_DIR}/docker/ardusub/docker-compose.yml}"

case "${1:-}" in
  -h|--help)
    cat <<'USAGE'
Usage: ./start_docker_ardusub_sitl.sh

Builds and starts only the Ubuntu Docker ArduSub SITL backend, then follows its logs.
Use start_docker_sitl_mujoco_mj311.sh to start Docker SITL plus host MuJoCo together.
USAGE
    exit 0
    ;;
esac

if [[ ! -f "$COMPOSE_FILE" ]]; then
  echo "[docker-sitl] compose file not found: $COMPOSE_FILE" >&2
  exit 1
fi

cd "$WORKSPACE_DIR"
docker compose -f "$COMPOSE_FILE" up --build -d ardusub-sitl
docker compose -f "$COMPOSE_FILE" logs -f ardusub-sitl
