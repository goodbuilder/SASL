#!/usr/bin/env bash
set -euo pipefail

# The doctor must measure the same closed runtime used by the operator
# launchers.  In particular, do not let an invoking shell's user packages,
# ROS overlay, or locally installed GStreamer plugins satisfy the check.
export PYTHONNOUSERSITE=1
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONPATH
unset PYTHONHOME
unset LD_LIBRARY_PATH
unset GI_TYPELIB_PATH
unset GST_PLUGIN_PATH
unset GST_PLUGIN_PATH_1_0
unset GST_PLUGIN_SYSTEM_PATH
unset GST_PLUGIN_SYSTEM_PATH_1_0
unset GST_PLUGIN_SCANNER
unset GST_PLUGIN_SCANNER_1_0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SASL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CANONICAL_PYTHON="$SASL_ROOT/.venv/bin/python"

usage() {
  cat <<'USAGE'
Usage: tools/check_operator_runtime.sh --execution|--deployment [--decoder auto|nvidia|software]

Read-only checks for the canonical StereoWIRIS/Workstation runtime.
  --execution   Validate the repository .venv and current app capabilities.
  --deployment  Validate Ubuntu packages and venv recreation readiness.
  --help        Show this dependency-free help.
USAGE
}

case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
  --execution|--deployment)
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

if [[ -n "${PYTHON_BIN:-}" ]]; then
  candidate="$PYTHON_BIN"
  if [[ "$candidate" != /* ]]; then
    if [[ "$candidate" == */* ]]; then
      candidate="$(realpath -s -m "$PWD/$candidate")"
    else
      candidate="$(command -v "$candidate" 2>/dev/null || true)"
    fi
  fi
  expected="$(realpath -s -m "$CANONICAL_PYTHON")"
  actual="$(realpath -s -m "$candidate")"
  if [[ "$actual" != "$expected" ]]; then
    printf '%s\n' \
      "FAIL python.override: PYTHON_BIN points outside the canonical runtime." \
      "  actual: ${actual:-unresolved}" \
      "  required: $expected" \
      "  Recovery: unset PYTHON_BIN" >&2
    exit 2
  fi
fi

export PYTHONPATH="$SASL_ROOT/common/python${PYTHONPATH:+:$PYTHONPATH}"

if [[ "$1" == "--execution" ]]; then
  if [[ ! -x "$CANONICAL_PYTHON" ]]; then
    printf '%s\n' \
      "FAIL python.executable: canonical venv is missing: $CANONICAL_PYTHON" \
      "  Recovery: cd $SASL_ROOT" \
      "  Recovery: python3 -m venv --system-site-packages .venv" \
      "  Recovery: .venv/bin/python -m pip install -r requirements.txt" >&2
    exit 1
  fi
  exec "$CANONICAL_PYTHON" -m sasl_common.operator_runtime "$@"
fi

SYSTEM_PYTHON="/usr/bin/python3"
if [[ ! -x "$SYSTEM_PYTHON" ]]; then
  printf '%s\n' "FAIL python.system: required Ubuntu interpreter is missing: $SYSTEM_PYTHON" >&2
  exit 1
fi
exec "$SYSTEM_PYTHON" -m sasl_common.operator_runtime "$@"
