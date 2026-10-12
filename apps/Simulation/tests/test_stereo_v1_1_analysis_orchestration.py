from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np

from campaigns.stereo_v1_1.analyze_campaign import (
    _evaluation_record,
    _sensitivity_scenarios,
    _write_csv_immutable,
)
from campaigns.stereo_v1_1 import analyze_campaign
from campaigns.stereo_v1_1.calibration_solver import R_WC_NOMINAL
from campaigns.stereo_v1_1.gt_alignment import GroundTruthSeries
from campaigns.stereo_v1_1.summarize_campaign import _analysis_handoff


SIM_ROOT = Path(__file__).resolve().parents[1]


def test_sensitivity_matrix_covers_protocol_levels_without_full_factorial():
    spec = json.loads((SIM_ROOT / "campaigns/stereo_v1_1/protocol/03_experiment_spec.json")
                      .read_text(encoding="utf-8-sig"))
    scenarios = _sensitivity_scenarios(spec)
    ids = [row["scenario_id"] for row in scenarios]
    assert len(ids) == 25
    assert len(set(ids)) == len(ids)
    by_factor = {}
    for row in scenarios:
        by_factor.setdefault(row["factor"], []).append(row)
    assert set(by_factor) == {
        "camera_rtk_sigma_xyz_m", "camera_imu_roll_pitch_sigma_deg",
        "drone_rtk_sigma_xyz_m", "initial_yaw_bias_deg",
        "ideal_projection_pixel_sigma_px", "drone_rtk_timestamp_offset_s",
        "right_camera_timestamp_offset_s", "drone_rtk_shared_enu_bias",
    }
    assert {row["factor_value"] for row in by_factor["camera_imu_roll_pitch_sigma_deg"]} == {0.05}
    assert len(by_factor["initial_yaw_bias_deg"]) == 8
    assert all(row["input_branch"] == "synthetic_ground_truth_projection"
               for row in by_factor["ideal_projection_pixel_sigma_px"])


def test_summary_handoff_is_absent_until_a_frozen_analysis_exists(tmp_path):
    assert _analysis_handoff(tmp_path)["offline_analysis_status"] == "NOT_RUN"


def test_analyzer_cli_accepts_a_separate_read_only_source_root(tmp_path, monkeypatch):
    output_root = tmp_path / "analysis-output"
    input_root = tmp_path / "existing-campaign"
    captured = {}
    monkeypatch.setattr(sys, "argv", [
        "analyze_campaign.py", "--output-root", str(output_root),
        "--input-root", str(input_root), "--mc-trials", "7",
    ])
    monkeypatch.setattr(
        analyze_campaign, "analyze",
        lambda output, trials, input_root=None: captured.update(
            output=output, trials=trials, input_root=input_root,
        ) or {},
    )

    assert analyze_campaign.main() == 0
    assert captured == {"output": output_root, "trials": 7, "input_root": input_root}


def test_summary_handoff_verifies_the_freeze_hash_and_reports_analysis(tmp_path):
    version = tmp_path / "analysis_versions" / "analysis_test"
    version.mkdir(parents=True)
    freeze = version / "analysis_freeze.json"
    freeze.write_text("{}\n", encoding="utf-8")
    summary = version / "analysis_summary.json"
    summary.write_text(json.dumps({
        "analysis_id": "analysis_test",
        "selected_path_id": "P4",
        "calibration_result_count": 12,
        "evaluation_result_count": 18,
        "v0_position_errors_evaluated_after_freeze": True,
    }), encoding="utf-8")
    pointer = {
        "analysis_id": "analysis_test",
        "analysis_summary": "analysis_versions/analysis_test/analysis_summary.json",
        "analysis_freeze_sha256": hashlib.sha256(freeze.read_bytes()).hexdigest(),
    }
    (tmp_path / "analysis_latest.json").write_text(json.dumps(pointer), encoding="utf-8")
    result = _analysis_handoff(tmp_path)
    assert result["offline_analysis_status"] == "ANALYSIS_COMPLETE"
    assert result["selected_path_id"] == "P4"
    assert result["v0_position_metrics_available_after_freeze"] is True


def test_oracle_evaluation_does_not_require_estimated_euler_angles():
    result = {
        "method": "ORACLE_true_camera_pose",
        "attitude_variant": "ORACLE",
        "convergence_status": "REFERENCE_ONLY_NOT_A_CALIBRATION_ALGORITHM",
        "fixed_camera_centers_world_m": {
            "left": [-12.5, 0.0, 0.1], "right": [12.5, 0.0, 0.1],
        },
        "estimated_R_WC": {
            "left": R_WC_NOMINAL.tolist(), "right": R_WC_NOMINAL.tolist(),
        },
        "estimated_phi_theta_psi_rad": None,
        "rig_group": "G0",
    }
    true_camera = {
        side: {"center": np.asarray(result["fixed_camera_centers_world_m"][side]),
               "rotation": R_WC_NOMINAL}
        for side in ("left", "right")
    }
    spec = json.loads((SIM_ROOT / "campaigns/stereo_v1_1/protocol/03_experiment_spec.json")
                      .read_text(encoding="utf-8-sig"))
    payload = _evaluation_record(
        result, "G0-V0-R01", "all_data",
        {"nominal_science_pair_count": 0,
         "both_detected_science_pair_count": 0},
        [], true_camera, GroundTruthSeries([]), spec,
    )
    assert payload["attitude_errors"] is None
    assert payload["evaluation_status"] == "NO_VALID_POSITION_ESTIMATES"


def test_csv_reports_are_idempotent_across_text_mode_newline_normalization(tmp_path):
    path = tmp_path / "metrics.csv"
    rows = [{"method": "A_rtk_independent", "rmse_m": 0.5},
            {"method": "C_no_drone_rtk_joint", "rmse_m": None}]
    first_hash = _write_csv_immutable(path, ["method", "rmse_m"], rows)
    first_bytes = path.read_bytes()
    second_hash = _write_csv_immutable(path, ["method", "rmse_m"], rows)
    assert first_hash == second_hash
    assert path.read_bytes() == first_bytes
    assert b"\r" not in first_bytes

