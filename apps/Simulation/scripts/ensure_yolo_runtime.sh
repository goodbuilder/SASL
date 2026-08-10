#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"
PREFERRED_PYTHON="${PREFERRED_PYTHON:-}"
YOLO_TORCH_BACKEND="${YOLO_TORCH_BACKEND:-cuda}"
YOLO_TORCH_CUDA_INDEX_URL="${YOLO_TORCH_CUDA_INDEX_URL:-https://download.pytorch.org/whl/cu126}"
YOLO_TORCH_CPU_INDEX_URL="${YOLO_TORCH_CPU_INDEX_URL:-https://download.pytorch.org/whl/cpu}"

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

ensure_system_site_packages() {
  local pyvenv_cfg="${VENV_DIR}/pyvenv.cfg"
  [[ -f "$pyvenv_cfg" ]] || return 0
  if grep -q '^include-system-site-packages = ' "$pyvenv_cfg"; then
    sed -i 's/^include-system-site-packages = .*/include-system-site-packages = true/' "$pyvenv_cfg"
  else
    printf '%s\n' 'include-system-site-packages = true' >>"$pyvenv_cfg"
  fi
}

python_has_yolo_runtime() {
  local python_bin="$1"
  "$python_bin" -c "import cv2, torch, ultralytics" >/dev/null 2>&1
}

python_matches_torch_backend() {
  local python_bin="$1"
  case "$YOLO_TORCH_BACKEND" in
    auto)
      return 0
      ;;
    cuda)
      "$python_bin" -c "import torch; raise SystemExit(0 if torch.version.cuda else 1)" >/dev/null 2>&1
      ;;
    cpu)
      "$python_bin" -c "import torch; raise SystemExit(0 if not torch.version.cuda else 1)" >/dev/null 2>&1
      ;;
    *)
      printf 'unknown YOLO_TORCH_BACKEND: %s\n' "$YOLO_TORCH_BACKEND" >&2
      return 2
      ;;
  esac
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
  local torch_index_url
  "$venv_python" -m pip install --upgrade pip setuptools wheel >&2
  if [[ "$YOLO_TORCH_BACKEND" == "cpu" ]]; then
    torch_index_url="$YOLO_TORCH_CPU_INDEX_URL"
  else
    torch_index_url="$YOLO_TORCH_CUDA_INDEX_URL"
  fi
  "$venv_python" -m pip install --upgrade --force-reinstall --index-url "$torch_index_url" torch torchvision >&2
  "$venv_python" -m pip install --upgrade 'numpy<2' 'opencv-python<4.10' ultralytics >&2
}

if [[ -x "${VENV_DIR}/bin/python" ]]; then
  ensure_system_site_packages
  if python_has_yolo_runtime "${VENV_DIR}/bin/python" \
    && python_matches_torch_backend "${VENV_DIR}/bin/python"; then
    printf '%s\n' "${VENV_DIR}/bin/python"
    exit 0
  fi
  VENV_PYTHON="${VENV_DIR}/bin/python"
fi

if [[ -n "$PREFERRED_PYTHON" ]]; then
  if preferred_resolved="$(resolve_executable "$PREFERRED_PYTHON")"; then
    if python_has_yolo_runtime "$preferred_resolved" \
      && python_matches_torch_backend "$preferred_resolved"; then
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

if [[ -z "${VENV_PYTHON:-}" ]]; then
  "$BUILDER_PYTHON" -m venv --system-site-packages "$VENV_DIR"
  ensure_system_site_packages
  VENV_PYTHON="${VENV_DIR}/bin/python"
fi
bootstrap_pip "$BUILDER_PYTHON" "$VENV_PYTHON"

if ! python_has_yolo_runtime "$VENV_PYTHON" \
  || ! python_matches_torch_backend "$VENV_PYTHON"; then
  install_yolo_runtime "$VENV_PYTHON"
fi

if python_has_yolo_runtime "$VENV_PYTHON" \
  && python_matches_torch_backend "$VENV_PYTHON"; then
  printf '%s\n' "$VENV_PYTHON"
  exit 0
fi

printf '%s\n' "failed to prepare YOLO Python runtime in ${VENV_DIR}" >&2
exit 1
