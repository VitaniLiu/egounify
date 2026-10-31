#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" == --execute ]]; then
  shift
  exec "$ROOT/scripts/start_hands.sh" --execute --replay "$@"
fi
exec /home/descfly/apex_n_sharpa/.venv/bin/python "$ROOT/replay.py" "$@"
