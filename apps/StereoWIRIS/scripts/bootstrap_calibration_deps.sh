#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SASL_ROOT="$(cd "$APP_DIR/../.." && pwd)"

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
  PYTHON_BIN="$(resolve_python "$APP_DIR/.venv/bin/python" "$SASL_ROOT/apps/Simulation/.venv/bin/python" "$SASL_ROOT/.local/legacy/Simulation/.venv/bin/python" python3 python)"
fi
REQ_FILE="$APP_DIR/requirements/calibration.txt"
MARKER="$APP_DIR/.calibration-deps.ok"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "[deps] Python not found or not executable: $PYTHON_BIN" >&2
  echo "[deps] Set PYTHON_BIN=/path/to/python and retry." >&2
  exit 1
fi

needs_install=0
if [[ ! -f "$MARKER" || "$REQ_FILE" -nt "$MARKER" ]]; then
  needs_install=1
else
  if ! "$PYTHON_BIN" - <<'PY' >/dev/null 2>&1
import cv2
import numpy as np
assert int(np.__version__.split('.')[0]) < 2, np.__version__
PY
  then
    needs_install=1
  fi
fi

if [[ "$needs_install" -eq 0 ]]; then
  echo "[deps] Calibration dependencies already available."
  exit 0
fi

echo "[deps] Installing calibration dependencies with $PYTHON_BIN"
"$PYTHON_BIN" -m pip install -r "$REQ_FILE"
"$PYTHON_BIN" - <<'PY'
import cv2
import numpy as np
print(f"[deps] OpenCV {cv2.__version__}, NumPy {np.__version__}")
assert int(np.__version__.split('.')[0]) < 2, np.__version__
PY
date -u +%Y-%m-%dT%H:%M:%SZ > "$MARKER"
echo "[deps] Calibration dependencies ready."
