from __future__ import annotations

import json
import importlib.util
from pathlib import Path
import sys
from decimal import Decimal

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import (
    CameraContract,
    build_sweep_plan,
    next_attempt_directory,
    write_plan,
)


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


def test_attempt_allocator_ignores_unrelated_directories(tmp_path: Path) -> None:
    case = tmp_path / "case"
    (case / "notes").mkdir(parents=True)
    (case / "attempt-invalid").mkdir()
    assert next_attempt_directory(case)[0] == 1


def test_state_only_case_valid_is_not_accepted_without_r2_final_marker(
    tmp_path: Path,
) -> None:
    module = _cli_module()
    case = {"case_id": "case-1", "case_hash": "hash", "payload": {}}
    attempt = tmp_path / "case-1" / "attempt-0001"
    _state(attempt, valid=True, attempt=1)
    inspection = module.inspect_latest_attempt(
        case=case,
        case_dir=attempt.parent,
        provenance={"commit_sha": "a" * 40, "branch": "sim/test"},
        sweep_id="sweep",
        warmup_count=0,
        analysis_count=1,
    )
    assert inspection["classification"] == "INCOMPLETE"


def test_validated_attempt_inspection_does_not_allocate_new_attempt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _cli_module()
    case = {"case_id": "case-1", "case_hash": "hash", "payload": {}}
    attempt = tmp_path / "case-1" / "attempt-0001"
    _state(attempt, valid=True, attempt=1)
    state = json.loads((attempt / "attempt_state.json").read_text())
    state.update(
        {
            "case_id": "case-1",
            "case_hash": "hash",
            "statistics": {},
        }
    )
    (attempt / "attempt_state.json").write_text(json.dumps(state), encoding="utf-8")
    (attempt / "recording").mkdir()
    (attempt / "recording" / "RUN_FINALIZED_VALID").touch()
    monkeypatch.setattr(module, "validate_r2_artifacts", lambda *args, **kwargs: ([], {}))
    monkeypatch.setattr(module, "compute_case_statistics", lambda *args, **kwargs: {})
    monkeypatch.setattr(module, "_attempt_spatial_validation", lambda *args, **kwargs: {})
    before = sorted(path.name for path in attempt.parent.iterdir())
    validated = module.validated_latest_attempt(
        case=case,
        case_dir=attempt.parent,
        provenance={"commit_sha": "a" * 40, "branch": "sim/test"},
        sweep_id="sweep",
        warmup_count=0,
        analysis_count=1,
    )
    assert validated is not None and validated[0] == attempt
    assert sorted(path.name for path in attempt.parent.iterdir()) == before


def test_resume_plan_identity_fields_must_match() -> None:
    module = _cli_module()
    stored = {"schema_version": "v", "commit_sha": "a", "branch": "b"}
    stored["plan_hash"] = module.sha256_bytes(
        module.canonical_json(stored).encode("utf-8")
    )
    for field in ("schema_version", "plan_hash", "commit_sha", "branch"):
        regenerated = dict(stored)
        regenerated[field] += "x"
        with pytest.raises(ValueError, match=field):
            module.validate_resume_plan(stored, regenerated)


def test_resume_validates_r2_artifacts_and_stored_statistics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _cli_module()
    case = {"case_id": "case-1", "case_hash": "hash", "payload": {}}
    attempt = tmp_path / "case-1" / "attempt-0001"
    _state(attempt, valid=True, attempt=1)
    state_path = attempt / "attempt_state.json"
    state = json.loads(state_path.read_text())
    state["case_id"] = "case-1"
    state["case_hash"] = "hash"
    state["statistics"] = {"analysis_sample_count": 2}
    state_path.write_text(json.dumps(state), encoding="utf-8")
    (attempt / "recording").mkdir()
    (attempt / "recording" / "RUN_FINALIZED_VALID").touch()
    monkeypatch.setattr(module, "validate_r2_artifacts", lambda *args, **kwargs: ([{}], {}))
    monkeypatch.setattr(
        module,
        "compute_case_statistics",
        lambda *args, **kwargs: {"analysis_sample_count": 2},
    )
    monkeypatch.setattr(module, "_attempt_spatial_validation", lambda *args, **kwargs: {})
    provenance = {"commit_sha": "a" * 40, "branch": "sim/test"}
    validated = module.validated_latest_attempt(
        case=case,
        case_dir=attempt.parent,
        provenance=provenance,
        sweep_id="sweep",
        warmup_count=1,
        analysis_count=2,
    )
    assert validated is not None and validated[0] == attempt

    monkeypatch.setattr(
        module,
        "validate_r2_artifacts",
        lambda *args, **kwargs: (_ for _ in ()).throw(ValueError("corrupt")),
    )
    assert module.validated_latest_attempt(
        case=case,
        case_dir=attempt.parent,
        provenance=provenance,
        sweep_id="sweep",
        warmup_count=1,
        analysis_count=2,
    ) is None


def test_latest_incomplete_attempt_prevents_fallback_to_older_valid(tmp_path: Path) -> None:
    module = _cli_module()
    case = {"case_id": "case-1", "case_hash": "hash", "payload": {}}
    older = tmp_path / "case-1" / "attempt-0001"
    _state(older, valid=True, attempt=1)
    (older / "recording").mkdir()
    (older / "recording" / "RUN_FINALIZED_VALID").touch()
    latest = tmp_path / "case-1" / "attempt-0007"
    _state(latest, valid=False, attempt=7)
    (latest / "recording").mkdir()
    (latest / "recording" / "RUN_INCOMPLETE").touch()
    inspection = module.inspect_latest_attempt(
        case=case,
        case_dir=latest.parent,
        provenance={"commit_sha": "a" * 40, "branch": "sim/test"},
        sweep_id="sweep",
        warmup_count=1,
        analysis_count=2,
    )
    assert inspection["classification"] == "INCOMPLETE"
    assert inspection["path"] == latest
    assert module.next_attempt_directory(latest.parent)[0] == 8


def test_contradictory_recording_markers_classify_latest_attempt_unknown(
    tmp_path: Path,
) -> None:
    module = _cli_module()
    case = {"case_id": "case-1", "case_hash": "hash", "payload": {}}
    attempt = tmp_path / "case-1" / "attempt-0001"
    _state(attempt, valid=True, attempt=1)
    recording = attempt / "recording"
    recording.mkdir()
    (recording / "RUN_FINALIZED_VALID").touch()
    (recording / "RUN_INCOMPLETE").touch()
    inspection = module.inspect_latest_attempt(
        case=case,
        case_dir=attempt.parent,
        provenance={"commit_sha": "a" * 40, "branch": "sim/test"},
        sweep_id="sweep",
        warmup_count=0,
        analysis_count=1,
    )
    assert inspection["classification"] == "UNKNOWN"
    assert "contradictory" in inspection["reason"]


def test_completed_resume_revalidates_top_level_csv_hash_and_semantics(tmp_path: Path) -> None:
    module = _cli_module()
    plan, cases = build_sweep_plan(
        sweep_id="sweep",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="repo",
        attitude_mode="ofat",
        axis_ranges=(),
        range_values=(Decimal("500"),),
        bearing_values=(Decimal("0"),),
        elevation_values=(Decimal("0"),),
        repetitions=1,
        seed=0,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=2,
        contract=CameraContract(),
    )
    write_plan(tmp_path, plan, cases)
    module.enter_running_state(tmp_path)
    state = {
        "attempt": 1,
        "case_id": cases[0]["case_id"],
        "case_hash": cases[0]["case_hash"],
        "plan_hash": plan["plan_hash"],
        "case_valid": True,
        "failure_reason": "",
        "statistics": {},
    }
    row = module._case_row(cases[0], 1, state)
    summary = module.finalize_sweep(
        output_dir=tmp_path,
        plan=plan,
        cases=cases,
        rows=[row],
        counters={"launch_count": 1},
    )
    completed = [(tmp_path / "cases" / cases[0]["case_id"] / "attempt-0001", state)]
    module.validate_final_sweep_artifacts(
        output_dir=tmp_path,
        plan=plan,
        cases=cases,
        completed=completed,
        summary=summary,
    )
    with (tmp_path / "sweep_cases.csv").open("a", encoding="utf-8") as handle:
        handle.write("tampered\n")
    with pytest.raises(ValueError, match="hash mismatch"):
        module.validate_final_sweep_artifacts(
            output_dir=tmp_path,
            plan=plan,
            cases=cases,
            completed=completed,
            summary=summary,
        )
