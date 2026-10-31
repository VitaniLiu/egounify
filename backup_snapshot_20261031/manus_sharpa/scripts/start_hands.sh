#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
test "$(findmnt -rn -M /mnt/apex-data -o TARGET)" = /mnt/apex-data
export ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
export LITCHI_RETARGET_V4_PYTHON="$ROOT/portable_deps/litchi_hardware/.pixi/envs/retarget-v4/bin/python"
export HANDS_ROOT="$ROOT"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
cd "$ROOT/portable_deps/litchi_hardware"
exec "$ROOT/pixi" run -e ros2 python "$ROOT/runtime.py" "$@"
