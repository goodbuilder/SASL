from __future__ import annotations

import json
import importlib.util
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import latest_valid_attempt, next_attempt_directory


def _cli_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "run_correction_search_sweep.py"
    name = "s2_r3_cli_resume_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _state(path: Path, *, valid: bool, attempt: int) -> None:
    path.mkdir(parents=True)
    (path / "attempt_state.json").write_text(
        json.dumps({"case_valid": valid, "attempt": attempt}), encoding="utf-8"
    )


def test_attempt_numbering_preserves_previous_attempts(tmp_path: Path) -> None:
    number, path = next_attempt_directory(tmp_path / "case")
    assert number == 1 and path.name == "attempt-0001"
    _state(path, valid=False, attempt=1)
    original = (path / "attempt_state.json").read_bytes()
    number, second = next_attempt_directory(tmp_path / "case")
    assert number == 2 and second.name == "attempt-0002"
    assert (path / "attempt_state.json").read_bytes() == original


def test_latest_valid_attempt_skips_invalid_and_incomplete(tmp_path: Path) -> None:
    case = tmp_path / "case"
    _state(case / "attempt-0001", valid=False, attempt=1)
    (case / "attempt-0002").mkdir()
    assert latest_valid_attempt(case) is None
    _state(case / "attempt-0003", valid=True, attempt=3)
    assert latest_valid_attempt(case).name == "attempt-0003"


def test_attempt_allocator_ignores_unrelated_directories(tmp_path: Path) -> None:
    case = tmp_path / "case"
    (case / "notes").mkdir(parents=True)
    (case / "attempt-invalid").mkdir()
    assert next_attempt_directory(case)[0] == 1


def test_resume_plan_identity_fields_must_match() -> None:
    module = _cli_module()
    stored = {"schema_version": "v", "plan_hash": "h", "commit_sha": "a", "branch": "b"}
    for field in stored:
        regenerated = dict(stored)
        regenerated[field] += "x"
        with pytest.raises(ValueError, match=field):
            module.validate_resume_plan(stored, regenerated)


def test_valid_completed_case_does_not_allocate_new_attempt(tmp_path: Path) -> None:
    case = tmp_path / "case"
    _state(case / "attempt-0001", valid=True, attempt=1)
    before = sorted(path.name for path in case.iterdir())
    assert latest_valid_attempt(case) is not None
    after = sorted(path.name for path in case.iterdir())
    assert after == before
