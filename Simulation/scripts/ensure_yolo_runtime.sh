#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"
PREFERRED_PYTHON="${PREFERRED_PYTHON:-}"

while (($#)); do
  case "$1" in
    --venv-dir)
      VENV_DIR="${2:?missing value for --venv-dir}"
      shift 2
      ;;
    --preferred-python)
      PREFERRED_PYTHON="${2:?missing value for --preferred-python}"
      shift 2
      ;;
    *)
      printf 'unknown option: %s\n' "$1" >&2
      exit 2
      ;;
  esac
done

resolve_executable() {
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

python_has_yolo_runtime() {
  local python_bin="$1"
  "$python_bin" -c "import cv2, torch, ultralytics" >/dev/null 2>&1
}

bootstrap_pip() {
  local builder_python="$1"
  local venv_python="$2"
  local get_pip_py
  if "$venv_python" -m pip --version >/dev/null 2>&1; then
    return 0
  fi
  if "$venv_python" -m ensurepip --upgrade >/dev/null 2>&1; then
    return 0
  fi
  get_pip_py="$(mktemp)"
  "$builder_python" - "$get_pip_py" <<'PY'
import sys
import urllib.request

urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py", sys.argv[1])
PY
  "$venv_python" "$get_pip_py" >/dev/null
  rm -f "$get_pip_py"
}

install_yolo_runtime() {
  local venv_python="$1"
  "$venv_python" -m pip install --upgrade pip setuptools wheel >&2
  "$venv_python" -m pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision >&2
  "$venv_python" -m pip install ultralytics >&2
}

if [[ -x "${VENV_DIR}/bin/python" ]] && python_has_yolo_runtime "${VENV_DIR}/bin/python"; then
  printf '%s\n' "${VENV_DIR}/bin/python"
  exit 0
fi

if [[ -n "$PREFERRED_PYTHON" ]]; then
  if preferred_resolved="$(resolve_executable "$PREFERRED_PYTHON")"; then
    if python_has_yolo_runtime "$preferred_resolved"; then
      printf '%s\n' "$preferred_resolved"
      exit 0
    fi
    BUILDER_PYTHON="$preferred_resolved"
  fi
fi

BUILDER_PYTHON="${BUILDER_PYTHON:-}"
if [[ -z "$BUILDER_PYTHON" ]]; then
  BUILDER_PYTHON="$(resolve_executable python3 python)"
fi

"$BUILDER_PYTHON" -m venv "$VENV_DIR"
VENV_PYTHON="${VENV_DIR}/bin/python"
bootstrap_pip "$BUILDER_PYTHON" "$VENV_PYTHON"

if ! python_has_yolo_runtime "$VENV_PYTHON"; then
  install_yolo_runtime "$VENV_PYTHON"
fi

if python_has_yolo_runtime "$VENV_PYTHON"; then
  printf '%s\n' "$VENV_PYTHON"
  exit 0
fi

printf '%s\n' "failed to prepare YOLO Python runtime in ${VENV_DIR}" >&2
exit 1
