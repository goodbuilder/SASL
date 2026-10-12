#!/usr/bin/env bash
# Run software regression with loopback-only networking, including child workers.
set -euo pipefail

SASL_TEST_ROOT="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
SASL_TEST_PYTHON="${SASL_TEST_PYTHON:-$SASL_TEST_ROOT/.venv/bin/python}"
if [[ ! -x "$SASL_TEST_PYTHON" ]]; then
    printf '%s\n' 'Workstation test environment missing. Run tools/setup_workstation_test_env.sh from the repository.' >&2
    exit 2
fi
export SASL_TEST_PYTHON

if [[ "${1:-}" != "--inside-test-network" ]]; then
    exec unshare --user --map-root-user --net "${BASH_SOURCE[0]}" --inside-test-network "$@"
fi
shift
mapfile -t SASL_TEST_LINKS < <(ip -o link show)
if [[ "${#SASL_TEST_LINKS[@]}" -ne 1 || "${SASL_TEST_LINKS[0]}" != *": lo: "* ]]; then
    printf '%s\n' 'Refusing regression: network namespace contains a non-loopback interface.' >&2
    exit 2
fi
ip link set lo up
cd "$SASL_TEST_ROOT"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
unset PYTHONHOME
export PYTHONDONTWRITEBYTECODE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export QT_QPA_PLATFORM=offscreen
export WORKSTATION_FORBID_EXTERNAL_NETWORK=1
SASL_TEST_PATHS="apps/StereoWIRIS/tests/test_coordinate_contract.py apps/StereoWIRIS/tests/test_stereo_geometry_common.py apps/StereoWIRIS/tests/test_triangulation.py apps/Workstation/tests"
# pytest applies testpaths only without explicit file arguments. Options such
# as -q or --collect-only must not accidentally select unrelated app suites.
exec "$SASL_TEST_PYTHON" -B -m pytest -p no:cacheprovider -o "testpaths=$SASL_TEST_PATHS" "$@"
