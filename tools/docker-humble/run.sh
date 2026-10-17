#!/usr/bin/env bash
# Opens a shell, or runs the given command, in the ROS2 Humble container.
# --build rebuilds the image first (needed after editing the Dockerfile).
set -euo pipefail
cd "$(dirname "$0")"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
if [[ "${1:-}" == "--build" ]]; then
    shift
    docker compose build
fi
exec docker compose run --rm humble "$@"
