#!/usr/bin/env bash
set -euo pipefail

# The canonical environment must not be completed by ~/.local packages or
# executables. System-site packages remain available through pyvenv.cfg.
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

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="$(cd "$APP_DIR/.." && pwd)"
SASL_ROOT="$(cd "$APP_DIR/../.." && pwd)"
COMMON_PYTHON="$SASL_ROOT/common/python"
CANONICAL_PYTHON="$SASL_ROOT/.venv/bin/python"

usage() {
  cat <<'USAGE'
Usage: apps/Workstation/run.sh [--check] [--maximized] [app args...]

Options:
  --check  Verify the canonical Python, settings, model, and GStreamer runtime.
  --help   Show this dependency-free help.
USAGE
}

case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
esac

if [[ ! -x "$CANONICAL_PYTHON" ]]; then
  printf '%s\n' \
    "FAIL python.executable: canonical venv is missing: $CANONICAL_PYTHON" \
    "  Recovery: cd $SASL_ROOT" \
    "  Recovery: python3 -m venv --system-site-packages .venv" \
    "  Recovery: .venv/bin/python -m pip install -r requirements.txt" >&2
  exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-$CANONICAL_PYTHON}"
if [[ "$PYTHON_BIN" != /* ]]; then
  if [[ "$PYTHON_BIN" == */* ]]; then
    PYTHON_BIN="$(realpath -s -m "$PWD/$PYTHON_BIN")"
  else
    PYTHON_BIN="$(command -v "$PYTHON_BIN" 2>/dev/null || true)"
  fi
fi
if [[ -z "$PYTHON_BIN" ]]; then
  printf '%s\n' "FAIL python.override: PYTHON_BIN could not be resolved" "  Recovery: unset PYTHON_BIN" >&2
  exit 2
fi
if [[ "$(realpath -s -m "$PYTHON_BIN")" != "$(realpath -s -m "$CANONICAL_PYTHON")" ]]; then
  printf '%s\n' \
    "FAIL python.override: Workstation only supports $CANONICAL_PYTHON" \
    "  actual PYTHON_BIN: ${PYTHON_BIN:-unresolved}" \
    "  Recovery: unset PYTHON_BIN" >&2
  exit 2
fi

export PYTHONPATH="$APPS_DIR:$SASL_ROOT:$COMMON_PYTHON"
WORKSTATION_DATA_ROOT="${SASL_WORKSTATION_DATA_ROOT:-$SASL_ROOT/.local/workstation_data}"
if [[ "$WORKSTATION_DATA_ROOT" != /* ]]; then
  WORKSTATION_DATA_ROOT="$SASL_ROOT/$WORKSTATION_DATA_ROOT"
fi
export SASL_WORKSTATION_DATA_ROOT="$WORKSTATION_DATA_ROOT"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$WORKSTATION_DATA_ROOT/.cache/matplotlib}"
mkdir -p "$MPLCONFIGDIR"

case "${1:-}" in
  --check)
    exec "$PYTHON_BIN" -m Workstation "$@"
    ;;
esac

# Every launch overwrites this file with the current session's stdout/stderr
# (including the traceback PyQt5 prints just before an unhandled-exception
# abort) so the most recent run's log is always available without having to
# remember a `| tee` wrapper.
LOG_FILE="${WORKSTATION_RUN_LOG:-$WORKSTATION_DATA_ROOT/logs/run.log}"
mkdir -p "$(dirname "$LOG_FILE")"
exec > >(tee "$LOG_FILE") 2>&1

exec "$PYTHON_BIN" -m Workstation --maximized "$@"
