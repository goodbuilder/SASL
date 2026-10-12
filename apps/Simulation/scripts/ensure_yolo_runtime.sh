#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"
PREFERRED_PYTHON="${PREFERRED_PYTHON:-}"
YOLO_TORCH_BACKEND="${YOLO_TORCH_BACKEND:-cuda}"
YOLO_TORCH_CUDA_INDEX_URL="${YOLO_TORCH_CUDA_INDEX_URL:-https://download.pytorch.org/whl/cu126}"
YOLO_TORCH_CPU_INDEX_URL="${YOLO_TORCH_CPU_INDEX_URL:-https://download.pytorch.org/whl/cpu}"
YOLO_TORCH_VERSION="${YOLO_TORCH_VERSION:-}"
YOLO_TORCHVISION_VERSION="${YOLO_TORCHVISION_VERSION:-}"
YOLO_TORCHAUDIO_VERSION="${YOLO_TORCHAUDIO_VERSION:-}"
YOLO_ULTRALYTICS_VERSION="${YOLO_ULTRALYTICS_VERSION:-}"
YOLO_VENV_ISOLATED="${YOLO_VENV_ISOLATED:-false}"

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
    --isolated)
      YOLO_VENV_ISOLATED=true
      shift
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

configure_system_site_packages() {
  local pyvenv_cfg="${VENV_DIR}/pyvenv.cfg"
  [[ -f "$pyvenv_cfg" ]] || return 0
  local desired=true
  if [[ "$YOLO_VENV_ISOLATED" == true ]]; then desired=false; fi
  if grep -q '^include-system-site-packages = ' "$pyvenv_cfg"; then
    sed -i "s/^include-system-site-packages = .*/include-system-site-packages = ${desired}/" "$pyvenv_cfg"
  else
    printf 'include-system-site-packages = %s\n' "$desired" >>"$pyvenv_cfg"
  fi
}

create_virtualenv() {
  local builder_python="$1"
  local venv_args=()
  local virtualenv_args=(--clear)
  if [[ "$YOLO_VENV_ISOLATED" != true ]]; then
    venv_args+=(--system-site-packages)
    virtualenv_args+=(--system-site-packages)
  fi
  if "$builder_python" -m venv "${venv_args[@]}" "$VENV_DIR"; then
    return 0
  fi
  printf '%s\n' \
    "stdlib venv unavailable; retrying with the installed virtualenv module" >&2
  "$builder_python" -m virtualenv "${virtualenv_args[@]}" "$VENV_DIR"
}

python_has_yolo_runtime() {
  local python_bin="$1"
  "$python_bin" -c "import cv2, torch, ultralytics" >/dev/null 2>&1
}

python_has_campaign_analysis_runtime() {
  local python_bin="$1"
  "$python_bin" -c "import numpy, scipy, zstandard" >/dev/null 2>&1
}

install_campaign_analysis_runtime() {
  local python_bin="$1"
  "$python_bin" -m pip install --requirement \
    "$ROOT_DIR/campaigns/stereo_v1_1/requirements-postprocess.txt" >&2
}

python_matches_torch_distribution() {
  local python_bin="$1"
  "$python_bin" - \
    "$YOLO_TORCH_BACKEND" \
    "$YOLO_TORCH_VERSION" \
    "$YOLO_TORCHVISION_VERSION" \
    "$YOLO_TORCHAUDIO_VERSION" \
    "$YOLO_ULTRALYTICS_VERSION" <<'PY' >/dev/null 2>&1
import importlib
import sys

import torch

backend, torch_version, vision_version, audio_version, ultralytics_version = sys.argv[1:]


def base_version(value: str) -> str:
    return value.split("+", 1)[0]


if torch_version:
    assert base_version(torch.__version__) == torch_version
if backend == "cuda":
    assert torch.version.cuda
elif backend == "cpu":
    assert not torch.version.cuda
if vision_version:
    torchvision = importlib.import_module("torchvision")
    assert base_version(torchvision.__version__) == vision_version
if audio_version:
    torchaudio = importlib.import_module("torchaudio")
    assert base_version(torchaudio.__version__) == audio_version
if ultralytics_version:
    ultralytics = importlib.import_module("ultralytics")
    assert base_version(ultralytics.__version__) == ultralytics_version
PY
}

python_matches_torch_backend() {
  local python_bin="$1"
  case "$YOLO_TORCH_BACKEND" in
    auto)
      return 0
      ;;
    cuda)
      "$python_bin" -c "import torch; assert torch.version.cuda; assert torch.cuda.is_available(); torch.zeros(1, device='cuda:0')" >/dev/null 2>&1
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

report_backend_failure() {
  local python_bin="$1"
  "$python_bin" - <<'PY' >&2 || true
import torch

print(
    "YOLO CUDA runtime is installed but the GPU is unavailable: "
    f"torch={torch.__version__}, cuda_build={torch.version.cuda}, "
    f"device_count={torch.cuda.device_count()}, "
    f"is_available={torch.cuda.is_available()}"
)
try:
    torch.zeros(1, device="cuda:0")
except Exception as exc:
    print(f"CUDA probe failed: {type(exc).__name__}: {exc}")
PY
}

report_backend_success() {
  local python_bin="$1"
  "$python_bin" - "$YOLO_TORCH_BACKEND" <<'PY' >&2
import site
import sys

import torch
import torchvision
import ultralytics

backend = sys.argv[1]
if backend == "cuda":
    torch.zeros(1, device="cuda:0")
    device = torch.cuda.get_device_name(0)
else:
    device = "cpu"
print(
    "YOLO runtime verified: "
    f"python={sys.executable}, torch={torch.__version__}, "
    f"torchvision={torchvision.__version__}, ultralytics={ultralytics.__version__}, "
    f"cuda_build={torch.version.cuda}, device={device}, "
    f"user_site_enabled={site.ENABLE_USER_SITE}"
)
PY
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
  local torch_packages=(torch torchvision)
  "$venv_python" -m pip install --upgrade pip setuptools wheel >&2
  if [[ "$YOLO_TORCH_BACKEND" == "cpu" ]]; then
    torch_index_url="$YOLO_TORCH_CPU_INDEX_URL"
  else
    torch_index_url="$YOLO_TORCH_CUDA_INDEX_URL"
  fi
  if [[ -n "$YOLO_TORCH_VERSION" ]]; then torch_packages=("torch==$YOLO_TORCH_VERSION"); fi
  if [[ -n "$YOLO_TORCHVISION_VERSION" ]]; then
    torch_packages+=("torchvision==$YOLO_TORCHVISION_VERSION")
  elif [[ -z "$YOLO_TORCH_VERSION" ]]; then
    torch_packages+=(torchvision)
  fi
  if [[ -n "$YOLO_TORCHAUDIO_VERSION" ]]; then
    torch_packages+=("torchaudio==$YOLO_TORCHAUDIO_VERSION")
  fi
  "$venv_python" -m pip install --upgrade --force-reinstall --index-url "$torch_index_url" "${torch_packages[@]}" >&2
  local ultralytics_package=ultralytics
  if [[ -n "$YOLO_ULTRALYTICS_VERSION" ]]; then
    ultralytics_package="ultralytics==$YOLO_ULTRALYTICS_VERSION"
  fi
  "$venv_python" -m pip install --upgrade 'numpy<2' 'opencv-python<4.10' "$ultralytics_package" >&2
}

if [[ -x "${VENV_DIR}/bin/python" ]]; then
  configure_system_site_packages
  if python_has_yolo_runtime "${VENV_DIR}/bin/python" \
    && python_matches_torch_distribution "${VENV_DIR}/bin/python"; then
    if ! python_has_campaign_analysis_runtime "${VENV_DIR}/bin/python"; then
      install_campaign_analysis_runtime "${VENV_DIR}/bin/python"
    fi
    if ! python_has_campaign_analysis_runtime "${VENV_DIR}/bin/python"; then
      printf '%s\n' "campaign postprocessing/analysis dependencies are unavailable in ${VENV_DIR}" >&2
      exit 1
    fi
    if python_matches_torch_backend "${VENV_DIR}/bin/python"; then
      report_backend_success "${VENV_DIR}/bin/python"
      printf '%s\n' "${VENV_DIR}/bin/python"
      exit 0
    fi
    report_backend_failure "${VENV_DIR}/bin/python"
    exit 1
  fi
  VENV_PYTHON="${VENV_DIR}/bin/python"
fi

if [[ -n "$PREFERRED_PYTHON" ]]; then
  if preferred_resolved="$(resolve_executable "$PREFERRED_PYTHON")"; then
    if [[ "$YOLO_VENV_ISOLATED" != true ]] \
      && python_has_yolo_runtime "$preferred_resolved" \
      && python_matches_torch_distribution "$preferred_resolved" \
      && python_has_campaign_analysis_runtime "$preferred_resolved" \
      && python_matches_torch_backend "$preferred_resolved"; then
      report_backend_success "$preferred_resolved"
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
  create_virtualenv "$BUILDER_PYTHON"
  configure_system_site_packages
  VENV_PYTHON="${VENV_DIR}/bin/python"
fi
bootstrap_pip "$BUILDER_PYTHON" "$VENV_PYTHON"

if ! python_has_yolo_runtime "$VENV_PYTHON" \
  || ! python_matches_torch_distribution "$VENV_PYTHON"; then
  install_yolo_runtime "$VENV_PYTHON"
fi

if ! python_has_campaign_analysis_runtime "$VENV_PYTHON"; then
  install_campaign_analysis_runtime "$VENV_PYTHON"
fi

if python_has_yolo_runtime "$VENV_PYTHON" \
  && python_matches_torch_distribution "$VENV_PYTHON"; then
  if ! python_has_campaign_analysis_runtime "$VENV_PYTHON"; then
    printf '%s\n' "campaign postprocessing/analysis dependencies are unavailable in ${VENV_DIR}" >&2
    exit 1
  fi
  if python_matches_torch_backend "$VENV_PYTHON"; then
    report_backend_success "$VENV_PYTHON"
    printf '%s\n' "$VENV_PYTHON"
    exit 0
  fi
  report_backend_failure "$VENV_PYTHON"
  exit 1
fi

printf '%s\n' "failed to prepare YOLO Python runtime in ${VENV_DIR}" >&2
exit 1
