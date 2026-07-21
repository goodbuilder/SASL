from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_qualification import (
    IDENTITY_MATRIX,
    MATRIX_DIRECTION,
    PROFILE_CATALOG_ROLE,
    SELECTION_RULE,
    QualificationSpec,
    authorize_unseal,
    prepare_plans,
    qualification_status,
    validate_access_ledger,
    validate_frozen_catalog,
    validate_sealed_commitment,
)
from stereo_ground_sim.correction_sweep import ATTITUDE_AXES, CameraContract
from stereo_ground_sim.research_recording import atomic_write_json, sha256_file


SHA = "d" * 40
BRANCH = "sim/s2-r4a-sealed-test"


def _prepared(tmp_path: Path) -> Path:
    root = tmp_path / "qualification"
    prepare_plans(root, QualificationSpec("sealed-test", "goodbuilder/SASL", BRANCH, SHA))
    return root


def test_sealed_commitment_is_exact_deterministic_plan_only_contract(tmp_path: Path) -> None:
    root = _prepared(tmp_path)
    commitment = json.loads((root / "sealed_evaluation_commitment.json").read_text(encoding="utf-8"))
    validated = validate_sealed_commitment(
        commitment,
        expected_branch=BRANCH,
        expected_commit_sha=SHA,
        expected_qualification_id="sealed-test",
    )
    assert validated["case_count"] == 225
    assert validated["created_before_profile_freeze"] is True
    assert validated["expected_output_frame"] == "baseline_center_frd"
    assert validated["target_physics_contract"] == {
        "trajectory_mode": "static_gazebo_reference_point",
        "target_physics_mode": "static_model",
        "target_static_for_research": True,
    }
    assert (root / "SEALED_EVALUATION_NOT_EXECUTED").is_file()
    assert not (root / "SEALED_EVALUATION_OPENED").exists()
    assert not (root / "SEALED_EVALUATION_COMPLETED").exists()
    assert not (root / "sealed_plan" / "cases").exists()
    assert not list((root / "sealed_plan").rglob("pair_records.jsonl"))


@pytest.mark.parametrize("mutation", ("hash", "extra", "missing", "metric"))
def test_sealed_commitment_tampering_and_metric_fields_are_rejected(tmp_path: Path, mutation: str) -> None:
    root = _prepared(tmp_path)
    commitment = json.loads((root / "sealed_evaluation_commitment.json").read_text(encoding="utf-8"))
    if mutation == "hash":
        commitment["sealed_plan_hash"] = "0" * 64
    elif mutation == "extra":
        commitment["unexpected"] = True
    elif mutation == "missing":
        del commitment["case_id_ordered_hash"]
    else:
        commitment["sealed_position_error_median"] = 0.0
    with pytest.raises(ValueError, match="schema mismatch|hash mismatch"):
        validate_sealed_commitment(commitment)


def test_access_ledger_blocks_sealed_search_freeze_runtime_and_open_stages() -> None:
    base = {
        "canonical_path": "/external/sealed/pair_records.jsonl",
        "artifact_hash": "0" * 64,
        "operation": "read",
        "utc": "2026-07-22T00:00:00Z",
        "process_pid": 1,
    }
    for stage in ("fit", "validation_score", "unseal", "sealed_evaluation"):
        entry = {**base, "sequence": 1, "stage": stage, "role": "sealed_evaluation"}
        with pytest.raises(ValueError, match="sealed Evaluation"):
            validate_access_ledger([entry])
    freeze_runtime = {
        **base,
        "sequence": 1,
        "stage": "freeze",
        "role": "sealed_evaluation",
        "operation": "read_pair_records",
    }
    with pytest.raises(ValueError, match="sealed runtime"):
        validate_access_ledger([freeze_runtime])


def test_unseal_requires_exact_frozen_hashes_then_performs_one_way_marker_transition(tmp_path: Path) -> None:
    root = _prepared(tmp_path)
    os.replace(root / "QUALIFICATION_PLANS_PREPARED", root / "QUALIFICATION_PROFILE_FROZEN")
    profile = {
        "source_attitude_deg": {axis: 0.0 for axis in ATTITUDE_AXES},
        "fitted_left_matrix_row_major": [list(row) for row in IDENTITY_MATRIX],
        "fitted_right_matrix_row_major": [list(row) for row in IDENTITY_MATRIX],
        "left_determinant": 1.0,
        "right_determinant": 1.0,
        "left_orthonormality_error": 0.0,
        "right_orthonormality_error": 0.0,
        "left_support_case_count": 18,
        "right_support_case_count": 18,
        "left_singular_values": [1.0, 1.0, 1.0],
        "right_singular_values": [1.0, 1.0, 1.0],
        "calibration_residual_diagnostics": {},
        "validation_metric_summary": {},
    }
    catalog = {
        "schema_version": "s2_sealed_qualification_v1",
        "qualification_id": "sealed-test",
        "branch": BRANCH,
        "commit_sha": SHA,
        "selected_method_family": "dual_camera_wahba_so3",
        "selection_rule": SELECTION_RULE,
        "calibration_plan_hash": "1" * 64,
        "validation_plan_hash": "2" * 64,
        "sealed_plan_commitment_hash": "3" * 64,
        "candidate_search_code_sha256": "4" * 64,
        "camera_contract": CameraContract().payload(),
        "observation_mode": "ideal_reference_point",
        "output_frame": "baseline_center_frd",
        "output_axes": "forward_right_down",
        "position_unit": "m",
        "matrix_direction_convention": MATRIX_DIRECTION,
        "quaternion_order": "xyzw",
        "profile_catalog_role": PROFILE_CATALOG_ROLE,
        "runtime_selection_source": "not_implemented",
        "deployment_ready": False,
        "profile_count": 25,
        "profiles": [{"profile_key": f"profile-{index:02d}", **profile} for index in range(25)],
    }
    assert validate_frozen_catalog(catalog, expected_branch=BRANCH, expected_commit_sha=SHA) == catalog
    extra = {**catalog, "sealed_position_error_median": 0.0}
    with pytest.raises(ValueError, match="schema mismatch"):
        validate_frozen_catalog(extra)
    missing = dict(catalog)
    del missing["deployment_ready"]
    with pytest.raises(ValueError, match="schema mismatch"):
        validate_frozen_catalog(missing)
    reflected = copy.deepcopy(catalog)
    reflected["profiles"][0]["fitted_left_matrix_row_major"][0][0] = -1.0
    with pytest.raises(ValueError, match="proper orthonormal rotation"):
        validate_frozen_catalog(reflected)
    atomic_write_json(root / "frozen_correction_catalog.json", catalog)
    atomic_write_json(
        root / "qualification_state.json",
        {
            "schema_version": "s2_sealed_qualification_v1",
            "qualification_id": "sealed-test",
            "state": "PROFILE_FROZEN",
            "branch": BRANCH,
            "commit_sha": SHA,
        },
    )
    commitment_path = root / "sealed_evaluation_commitment.json"
    catalog_path = root / "frozen_correction_catalog.json"
    with pytest.raises(ValueError, match="hash mismatch"):
        authorize_unseal(
            root,
            frozen_catalog_sha256="0" * 64,
            sealed_commitment_sha256=sha256_file(commitment_path),
            expected_branch=BRANCH,
            expected_commit_sha=SHA,
        )
    result = authorize_unseal(
        root,
        frozen_catalog_sha256=sha256_file(catalog_path),
        sealed_commitment_sha256=sha256_file(commitment_path),
        expected_branch=BRANCH,
        expected_commit_sha=SHA,
    )
    assert result == {"state": "SEALED_OPENED", "selected_family": "dual_camera_wahba_so3"}
    assert (root / "SEALED_EVALUATION_OPENED").is_file()
    assert not (root / "SEALED_EVALUATION_NOT_EXECUTED").exists()
    assert qualification_status(root)["state"] == "SEALED_OPENED"
    ledger = validate_access_ledger(root / "data_access_ledger.jsonl", require_r4a_sealed_closed=False)
    assert ledger[-1]["stage"] == "unseal"
