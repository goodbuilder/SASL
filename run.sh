#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_RUNNER="$REPO_DIR/StereoWIRIS/run.sh"

case "${1:-}" in
  --check|--help|-h)
    exec bash "$APP_RUNNER" "$1"
    ;;
esac

exec bash "$APP_RUNNER" --ui workstation --maximized "$@"
