#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
test "$(findmnt -rn -M /mnt/apex-data -o TARGET)" = /mnt/apex-data
mkdir -p "$ROOT/.cache" "$ROOT/logs"
export PIXI_HOME="$ROOT/.pixi-home" XDG_CACHE_HOME="$ROOT/.cache"
export RATTLER_CACHE_DIR="$ROOT/.cache/rattler" UV_CACHE_DIR="$ROOT/.cache/uv" TMPDIR="$ROOT/.cache"
cd "$ROOT/portable_deps/litchi_hardware"
"$ROOT/pixi" install --locked -e ros2 -e retarget-v4
"$ROOT/pixi" run -e ros2 build-manus-bridge
cd ros2
"$ROOT/pixi" run -e ros2 colcon build --symlink-install --packages-select manus_driver sharpa_driver manus_sharpa_teleop sharpa_wave_description
echo 'Environment built. No hardware initialized. USB rule is staged only; see README.'
