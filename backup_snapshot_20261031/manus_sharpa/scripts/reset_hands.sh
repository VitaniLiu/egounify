#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
test "$(findmnt -rn -M /mnt/apex-data -o TARGET)" = /mnt/apex-data
unset PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
cd "$ROOT/portable_deps/litchi_hardware"
exec "$ROOT/pixi" run -e ros2 python "$ROOT/scripts/reset_hands.py" "$@"
