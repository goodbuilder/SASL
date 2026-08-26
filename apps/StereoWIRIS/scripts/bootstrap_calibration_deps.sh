#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SASL_ROOT="$(cd "$APP_DIR/../.." && pwd)"
CANONICAL_PYTHON="$SASL_ROOT/.venv/bin/python"

PYTHON_BIN="${PYTHON_BIN:-$CANONICAL_PYTHON}"
REQ_FILE="$APP_DIR/requirements/calibration.txt"
MARKER="$APP_DIR/.calibration-deps.ok"

if [[ ! -x "$CANONICAL_PYTHON" ]]; then
  echo "FAIL calibration.python: canonical Python is missing: $CANONICAL_PYTHON" >&2
  echo "  Recovery: cd $SASL_ROOT && python3 -m venv --system-site-packages .venv" >&2
  exit 1
fi
if [[ "$PYTHON_BIN" != /* ]]; then
  PYTHON_BIN="$(realpath -s -m "$PWD/$PYTHON_BIN")"
fi
if [[ "$(realpath -s -m "$PYTHON_BIN")" != "$(realpath -s -m "$CANONICAL_PYTHON")" ]]; then
  echo "FAIL calibration.python: only $CANONICAL_PYTHON is supported" >&2
  echo "  Recovery: unset PYTHON_BIN" >&2
  exit 2
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
