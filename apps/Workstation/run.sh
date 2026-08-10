#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SASL_ROOT="$(cd "$APP_DIR/../.." && pwd)"
COMMON_PYTHON="$SASL_ROOT/common/python"

resolve_python() {
  local candidate resolved
  for candidate in "$@"; do
    [[ -n "$candidate" ]] || continue
    if [[ "$candidate" == */* ]]; then
      if [[ -x "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
      fi
      continue
    fi
    resolved="$(command -v "$candidate" 2>/dev/null || true)"
    if [[ -n "$resolved" && -x "$resolved" ]]; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done
  return 1
}

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(resolve_python "$SASL_ROOT/.venv/bin/python" python3 python)"
fi

export PYTHONPATH="$SASL_ROOT:$COMMON_PYTHON${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$SASL_ROOT/.local/workstation_data/.cache/matplotlib}"
mkdir -p "$MPLCONFIGDIR"

case "${1:-}" in
  --check|--help|-h)
    exec "$PYTHON_BIN" -m Workstation "$@"
    ;;
esac

exec "$PYTHON_BIN" -m Workstation --maximized "$@"
