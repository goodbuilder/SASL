from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

import stereo_ground_sim.correction_qualification as qualification
from stereo_ground_sim.correction_qualification import (
    CANDIDATE_FAMILIES,
    CalibrationRepresentative,
    CandidateProfile,
    IDENTITY_MATRIX,
    QualificationSpec,
    ValidationCase,
    fit_select_freeze,
    prepare_plans,
    select_candidate_family,
)
from stereo_ground_sim.correction_sweep import ATTITUDE_AXES, CameraContract


SHA = "c" * 40
BRANCH = "sim/s2-r4a-freeze-test"


def _family_result(family: str, median: float, worst: float, zero: float, *, passed: bool = True) -> dict:
    return {
        "method_family": family,
        "gate_pass": passed,
        "profile_pass_count": 25 if passed else 0,
        "zero_profile_p99": zero,
        "nonzero_profile_macro_median": median,
        "nonzero_profile_macro_p95": median,
        "nonzero_profile_worst_p99": worst,
        "corrected_raw_macro_median_ratio": 0.001,
        "invalid_prediction_count": 0,
        "invalid_rate": 0.0,
        "profiles": {},
    }


def test_global_selection_uses_validation_lexicographic_rule_and_excludes_failures() -> None:
    results = {
        "families": {
            "raw_nominal": _family_result("raw_nominal", 1.0, 1.0, 0.0, passed=False),
            "shared_wahba_so3": _family_result("shared_wahba_so3", 2.0e-6, 5.0e-6, 1.0e-8),
            "dual_camera_wahba_so3": _family_result("dual_camera_wahba_so3", 1.0e-6, 9.0e-6, 1.0e-8),
        }
    }
    assert select_candidate_family(results) == "dual_camera_wahba_so3"
    results["families"]["dual_camera_wahba_so3"]["gate_pass"] = False
    assert select_candidate_family(results) == "shared_wahba_so3"


def test_selection_ties_reach_complexity_only_within_fixed_tolerance() -> None:
    shared = _family_result("shared_wahba_so3", 1.0e-6, 2.0e-6, 1.0e-8)
    dual = _family_result(
        "dual_camera_wahba_so3",
        1.0e-6 + 5.0e-13,
        2.0e-6 - 5.0e-13,
        1.0e-8 - 5.0e-13,
    )
    assert select_candidate_family({"families": {"shared": shared, "dual": dual}}) == "shared_wahba_so3"
    dual["nonzero_profile_macro_median"] = 1.0e-6 + 2.0e-12
    assert select_candidate_family({"families": {"shared": shared, "dual": dual}}) == "shared_wahba_so3"
    with pytest.raises(ValueError, match="frozen"):
        select_candidate_family({"families": {"shared": shared}}, tolerance=1.0e-9)


def test_worst_p99_then_zero_profile_are_secondary_validation_tiebreakers() -> None:
    shared = _family_result("shared_wahba_so3", 1.0e-6, 4.0e-6, 1.0e-9)
    dual = _family_result("dual_camera_wahba_so3", 1.0e-6, 2.0e-6, 9.0e-9)
    assert select_candidate_family({"families": {"shared": shared, "dual": dual}}) == "dual_camera_wahba_so3"
    dual["nonzero_profile_worst_p99"] = shared["nonzero_profile_worst_p99"]
    assert select_candidate_family({"families": {"shared": shared, "dual": dual}}) == "shared_wahba_so3"


def test_selection_requires_at_least_one_globally_passing_family() -> None:
    failed = _family_result("dual_camera_wahba_so3", 0.0, 0.0, 0.0, passed=False)
    with pytest.raises(ValueError, match="no fitted candidate"):
        select_candidate_family({"families": {"dual": failed}})


def _freeze_fixture(monkeypatch: pytest.MonkeyPatch, root: Path) -> dict:
    spec = QualificationSpec("freeze-test", "goodbuilder/SASL", BRANCH, SHA)
    prepared = prepare_plans(root, spec)
    bundle = qualification.build_qualification_bundle(spec)
    profile_payloads = {}
    for case in bundle["roles"]["calibration"]["cases"]:
        profile_payloads.setdefault(qualification.profile_key(case["payload"]), case["payload"])
    contract = CameraContract()
    representatives = []
    validation_cases = []
    candidates = {family: {} for family in CANDIDATE_FAMILIES}
    for key, payload in profile_payloads.items():
        source = tuple(float(payload["attitude_source_deg"][axis]["value"]) for axis in ATTITUDE_AXES)
        for index in range(18):
            representatives.append(
                CalibrationRepresentative(key, f"cal-{key}-{index}", "calibration", (1.0, 2.0), (3.0, 4.0), (300.0, 0.0, 0.0), contract, source)
            )
        for index in range(9):
            validation_cases.append(
                ValidationCase(key, f"val-{key}-{index}", "validation", tuple({} for _ in range(30)), contract, source)
            )
        for family in CANDIDATE_FAMILIES:
            candidates[family][key] = CandidateProfile(family, key, IDENTITY_MATRIX, IDENTITY_MATRIX, None, None)

    evaluation = {"schema_version": qualification.S2_QUALIFICATION_SCHEMA_VERSION, "validation_gates": qualification.validation_gate_contract(), "families": {}}
    for family in CANDIDATE_FAMILIES:
        item = _family_result(family, 0.0, 0.0, 0.0, passed=family == "dual_camera_wahba_so3")
        item["profiles"] = {
            key: {
                "profile_key": key,
                "valid": True,
                "source_attitude_deg": [
                    float(payload["attitude_source_deg"][axis]["value"])
                    for axis in ATTITUDE_AXES
                ],
            }
            for key, payload in profile_payloads.items()
        }
        evaluation["families"][family] = item

    def load_role_dataset(**kwargs):
        return representatives if kwargs["role"] == "calibration" else validation_cases

    monkeypatch.setattr(qualification, "_load_role_dataset", load_role_dataset)
    monkeypatch.setattr(qualification, "fit_candidate_families", lambda items: candidates)
    monkeypatch.setattr(qualification, "evaluate_candidate_families", lambda fitted, cases: evaluation)
    calibration_root = root.parent / "calibration-runtime"
    validation_root = root.parent / "validation-runtime"
    calibration_root.mkdir()
    validation_root.mkdir()
    (calibration_root / "sweep_summary.json").write_text("{}\n", encoding="utf-8")
    (validation_root / "sweep_summary.json").write_text("{}\n", encoding="utf-8")
    result = fit_select_freeze(
        qualification_root=root,
        calibration_sweep_root=calibration_root,
        validation_sweep_root=validation_root,
        expected_branch=BRANCH,
        expected_commit_sha=SHA,
    )
    return {"prepared": prepared, "result": result}


def test_freeze_writes_exact_catalog_markers_and_rejects_reselection(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "qualification"
    outcome = _freeze_fixture(monkeypatch, root)
    assert outcome["result"]["state"] == "PROFILE_FROZEN"
    catalog = json.loads((root / "frozen_correction_catalog.json").read_text(encoding="utf-8"))
    assert catalog["selected_method_family"] == "dual_camera_wahba_so3"
    assert catalog["profile_count"] == 25
    assert len(catalog["profiles"]) == 25
    assert catalog["output_frame"] == "baseline_center_frd"
    assert catalog["matrix_direction_convention"].startswith("R_Cnominal_Cactual")
    assert catalog["profile_catalog_role"] == "simulation_method_qualification_only"
    assert catalog["runtime_selection_source"] == "not_implemented"
    assert catalog["deployment_ready"] is False
    assert (root / "QUALIFICATION_PROFILE_FROZEN").is_file()
    assert (root / "SEALED_EVALUATION_NOT_EXECUTED").is_file()
    assert not (root / "SEALED_EVALUATION_OPENED").exists()
    with pytest.raises(ValueError, match="not eligible"):
        fit_select_freeze(
            qualification_root=root,
            calibration_sweep_root=tmp_path / "calibration-runtime",
            validation_sweep_root=tmp_path / "validation-runtime",
            expected_branch=BRANCH,
            expected_commit_sha=SHA,
        )
