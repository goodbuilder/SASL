"""The checked-in gate must collect the current Workstation regression suite."""

from pathlib import Path
import shlex

import yaml


def test_quality_workflow_targets_current_workstation_directory():
    root = Path(__file__).resolve().parents[3]
    workflow = yaml.safe_load((root / ".github/workflows/quality.yml").read_text())
    commands = [step["run"] for step in workflow["jobs"]["python-tests"]["steps"] if "run" in step]
    command = next(command for command in commands if "pytest" in command)
    paths = [token for token in shlex.split(command) if token.startswith("apps/")]
    assert paths
    assert all((root / path).exists() for path in paths), paths
    assert "apps/Workstation/tests" in paths


def test_historical_workstation_paths_are_exempt_only_for_reviewed_bytes():
    from tools.check_portable_paths import scan_text

    root = Path(__file__).resolve().parents[3]
    path = "apps/Workstation/Pixhawk/P1/workstation_endpoint.json"
    historical = (root / path).read_text()
    assert scan_text(path, historical) == ()
    home_path = "/".join(("", "home", "operator", "active.bin"))
    assert scan_text(path, historical + f'\n"new_asset": "{home_path}"\n')
    # An adjacent new file must never inherit an entire-directory exemption.
    assert scan_text("apps/Workstation/Pixhawk/P1/new_runtime.json", historical)
    assert scan_text("apps/Workstation/docs/changes/new/current-behavior.md", historical)


def test_current_measurement_docs_match_registry_and_no_deadline_policy():
    from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS

    app = Path(__file__).resolve().parents[1]
    contract = (app / "docs/MEASUREMENT_RUNTIME_CONTRACT.md").read_text()
    schema = (app / "docs/RECORDING_SCHEMA.md").read_text()
    current_readme = (app / "README.md").read_text().split("## Change Log", 1)[0]
    for method in MEASUREMENT_METHOD_IDS:
        assert f"`{method}`" in contract
        assert f"`{method}`" in schema
    assert "no automatic compute deadline" in contract
    assert "one-second timeout" not in current_readme
    assert "Comparison timeouts start" not in current_readme
    assert "A live-Pixhawk Measurement is blocked until" not in current_readme
    assert "Primary timeout is one second" not in schema


def test_historical_hash_checks_actual_file_bytes_including_line_endings(tmp_path):
    from tools.check_portable_paths import repository_violations

    root = Path(__file__).resolve().parents[3]
    path = "apps/Workstation/Pixhawk/P1/workstation_endpoint.json"
    destination = tmp_path / path
    destination.parent.mkdir(parents=True)
    historical = (root / path).read_bytes()
    destination.write_bytes(historical)
    assert repository_violations(tmp_path, paths=[path]) == ()
    destination.write_bytes(historical.replace(b"\n", b"\r\n"))
    assert repository_violations(tmp_path, paths=[path])
