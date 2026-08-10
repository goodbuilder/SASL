#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
  PYTHON_BIN="$(resolve_python "$REPO_DIR/.venv/bin/python" python3 python)"
fi

export PYTHONPATH="$REPO_DIR${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$REPO_DIR/workstation_data/.cache/matplotlib}"
mkdir -p "$MPLCONFIGDIR"

case "${1:-}" in
  --check|--help|-h)
    exec "$PYTHON_BIN" -m workstation "$@"
    ;;
esac

exec "$PYTHON_BIN" -m workstation --maximized "$@"
