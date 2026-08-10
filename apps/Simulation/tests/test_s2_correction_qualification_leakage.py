from __future__ import annotations

from dataclasses import fields
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_qualification import (
    CalibrationRepresentative,
    CandidatePrediction,
    QualificationSpec,
    ValidationCase,
    build_calibration_representative,
    build_qualification_bundle,
    fit_candidate_families,
)
from stereo_ground_sim.correction_sweep import CameraContract


def _case() -> dict:
    spec = QualificationSpec("leakage-test", "goodbuilder/SASL", "sim/test", "b" * 40)
    return build_qualification_bundle(spec)["roles"]["calibration"]["cases"][0]


def _records(case_id: str, *, corrected_offset: float) -> list[dict]:
    return [
        {
            "record_sequence": index + 1,
            "sample_eligible": True,
            "case_id": case_id,
            "output_frame": "baseline_center_frd",
            "output_axes": "forward_right_down",
            "position_unit": "m",
            "left_u_px": 630.0 + index * 0.001,
            "left_v_px": 350.0 + index * 0.001,
            "right_u_px": 650.0 + index * 0.001,
            "right_v_px": 350.0 + index * 0.001,
            "gt_forward_m": 300.0,
            "gt_right_m": -5.0,
            "gt_down_m": 2.0,
            "corrected_forward_m": corrected_offset + index,
            "corrected_right_m": corrected_offset - index,
            "corrected_down_m": corrected_offset * 2.0,
            "corrected_position_error_m": abs(corrected_offset),
            "correction_source": "injected_attitude_oracle",
        }
        for index in range(40)
    ]


def test_calibration_and_validation_roles_are_type_checked_before_fit_or_score() -> None:
    contract = CameraContract()
    with pytest.raises(ValueError, match="role must be calibration"):
        CalibrationRepresentative("profile", "case", "validation", (1.0, 2.0), (3.0, 4.0), (5.0, 6.0, 7.0), contract)
    with pytest.raises(ValueError, match="role must be validation"):
        ValidationCase("profile", "case", "sealed_evaluation", tuple({} for _ in range(30)), contract)


def test_solver_feature_contract_excludes_oracle_corrected_and_injected_rotation_fields() -> None:
    representative_fields = {item.name for item in fields(CalibrationRepresentative)}
    prediction_fields = {item.name for item in fields(CandidatePrediction)}
    forbidden = {
        "attitude_corrected_position",
        "corrected_forward_m",
        "corrected_right_m",
        "corrected_down_m",
        "effective_left_optical_roll_error_deg",
        "effective_right_optical_yaw_error_deg",
        "injected_attitude_matrix",
    }
    assert representative_fields.isdisjoint(forbidden)
    assert prediction_fields.isdisjoint(forbidden)
    assert {"left_pixel_px", "right_pixel_px", "gt_frd_m"} <= representative_fields


def test_oracle_corrected_value_changes_do_not_change_calibration_representative() -> None:
    case = _case()
    first = build_calibration_representative(_records(case["case_id"], corrected_offset=0.0), case)
    second = build_calibration_representative(_records(case["case_id"], corrected_offset=1.0e9), case)
    assert first == second


def test_candidate_fit_rejects_partial_or_non_calibration_input_structure() -> None:
    case = _case()
    representative = build_calibration_representative(_records(case["case_id"], corrected_offset=0.0), case)
    with pytest.raises(ValueError, match="exactly 25 attitude profiles"):
        fit_candidate_families([representative])


def test_analysis_record_rejects_frame_aliasing_and_string_truthiness() -> None:
    case = _case()
    string_truth = _records(case["case_id"], corrected_offset=0.0)
    string_truth[10]["sample_eligible"] = "true"
    with pytest.raises(ValueError, match="insufficient eligible samples"):
        build_calibration_representative(string_truth, case)

    wrong_frame = _records(case["case_id"], corrected_offset=0.0)
    wrong_frame[10]["output_frame"] = "camera_optical_rdf"
    with pytest.raises(ValueError, match="output frame mismatch"):
        build_calibration_representative(wrong_frame, case)
