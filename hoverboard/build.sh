#!/usr/bin/env bash
# Builds the hoverboard mainboard firmware with config.patch applied.
# Extra arguments go to "pio run", e.g. "./build.sh -t upload" to flash.
# The submodule is restored to a clean state on exit.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$HERE/hoverboard-firmware-hack-FOC"

if ! git -C "$FW" diff --quiet; then
  echo "Submodule has local changes, commit them to config.patch or discard them first." >&2
  exit 1
fi

git -C "$FW" apply --whitespace=nowarn "$HERE/config.patch"
trap 'git -C "$FW" apply --whitespace=nowarn -R "$HERE/config.patch"' EXIT

(cd "$FW" && pio run "$@")
echo "Firmware: $FW/.pio/build/VARIANT_USART/firmware.bin"
