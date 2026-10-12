#!/usr/bin/env bash
# Reproducible Workstation developer/test environment, isolated from user-site/ROS.
set -euo pipefail
SASL_ROOT="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
export PYTHONNOUSERSITE=1
export PIP_DISABLE_PIP_VERSION_CHECK=1
unset PYTHONPATH
unset PYTHONHOME
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), "This reference profile requires Python 3.10"'
if [[ ! -x "$SASL_ROOT/.venv/bin/python" ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$SASL_ROOT/.venv"
fi
"$SASL_ROOT/.venv/bin/python" -m pip install 'pip>=24,<27'
"$SASL_ROOT/.venv/bin/python" -m pip install \
    -r "$SASL_ROOT/requirements/dev.txt" \
    -c "$SASL_ROOT/requirements/constraints-workstation-py310.txt"
