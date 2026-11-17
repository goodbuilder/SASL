from __future__ import annotations

import csv
import hashlib
import json
import zipfile
from pathlib import Path
from xml.etree import ElementTree

import pytest

from campaigns.stereo_v1_1 import campaign_report_v3 as report


def _write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def _refresh_analysis_checksums(analysis_root: Path):
    entries = []
    for path in sorted(analysis_root.rglob("*")):
        if path.is_file() and path.name != "analysis_checksums.sha256":
            relative = path.relative_to(analysis_root).as_posix()
            entries.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {relative}\n")
    (analysis_root / "analysis_checksums.sha256").write_text("".join(entries), encoding="utf-8")


def _cell_value(cell, namespace):
    inline = cell.find("x:is", namespace)
    if inline is not None:
        return "".join(node.text or "" for node in inline.findall(".//x:t", namespace))
    value = cell.find("x:v", namespace)
    return value.text if value is not None else None


def _make_campaign(root: Path):
    analysis_id = "analysis_test_v3"
    analysis_root = root / "analysis_versions" / analysis_id
    rows = []
    planned = []
    run_specs = [
        {"run_id": f"G0-P{path}-R{repetition:02d}", "rig_group": "G0", "path_id": f"P{path}",
         "repetition": repetition, "role": "calibration", "data_split": "development_calibration",
         "use_for_calibration": True, "use_for_final_evaluation": False}
        for path in range(5) for repetition in range(1, 6)
    ] + [
        {"run_id": f"G1-P{path}-R{repetition:02d}", "rig_group": "G1", "path_id": f"P{path}",
         "repetition": repetition, "role": "calibration", "data_split": "development_calibration",
         "use_for_calibration": True, "use_for_final_evaluation": False}
        for path in (0, 4) for repetition in range(1, 4)
    ] + [
        {"run_id": f"{group}-V0-R{repetition:02d}", "rig_group": group, "path_id": "V0",
         "repetition": repetition, "role": "final_evaluation", "data_split": "evaluation",
         "use_for_calibration": False, "use_for_final_evaluation": True}
        for group, repetitions in (("G0", range(1, 6)), ("G1", range(1, 4)))
        for repetition in repetitions
    ]
    for index, run_spec in enumerate(run_specs, 1):
        run_id = run_spec["run_id"]
        attempt_id = "a01"
        recording = f"원본_실행별/{run_id}/{attempt_id}"
        attempt_dir = root / recording
        slot = {
            "sequence": index, **run_spec,
            "status": "VALID", "attempt_count": 1, "recording_path": recording,
            "attempts": [{"run_id": run_id, "attempt_id": attempt_id, "status": "VALID",
                          "recording_path": recording}],
        }
        rows.append(slot)
        planned.append({
            "sequence": index, **run_spec,
            "parameters": {"Path": {"north_start_m": 600.0, "north_end_m": 400.0, "glide_angle_deg": 20.0}},
        })
        _write_json(attempt_dir / "qc_report.json", {
            "run_id": run_id, "attempt_id": attempt_id, "status": "VALID",
            "infrastructure_errors": [], "protocol_errors": [], "warnings": [],
            "evidence": {
                "v12_science_capture": {"status": "CHECKED", "start_simulation_ns": 0, "end_simulation_ns": 20,
                                        "cameras": {"left": {"unique_frame_count": 2, "observed_window_hz": 50.0, "max_gap_ns": 20},
                                                    "right": {"unique_frame_count": 2, "observed_window_hz": 50.0, "max_gap_ns": 20}}},
                "science_setpoint_cadence_recomputed": {"required_hz": 50, "expected_full_reference_slots": 1,
                    "covered_full_reference_slots": 1, "missing_full_slot_count": 0, "max_simulation_gap_s": 0.02,
                    "science_window_duration_s": 0.02, "sufficient_unique_simulation_cadence": True},
            },
        })
        _write_json(attempt_dir / "gt_alignment_summary.json", {
            "maximum_bracketing_gap_s": 0.04,
            "image_count_by_alignment_status": {"EXACT": 4, "GAP_GE_40MS": 0},
            "science_image_count_by_alignment_status": {"EXACT": 4, "GAP_GE_40MS": 0},
            "tracking": None, "tracking_metrics_revealed": False,
        })
        _write_json(attempt_dir / "image_index_summary.json", {
            "left": {"frame_count": 2}, "right": {"frame_count": 2}, "pair_count": 2,
        })
        _write_json(attempt_dir / "controller_result.json", {"controller_status": "science_completed"})
        for side in ("left", "right"):
            _write_jsonl(attempt_dir / f"observations_{side}.jsonl", [
                {"detection_status": "DETECTED"}, {"detection_status": "NOT_DETECTED"},
            ])
        _write_jsonl(attempt_dir / "ground_truth.jsonl", [
            {"simulation_epoch": 1, "source_timestamp_ns": stamp, "position_world_enu_m": [0, 400, 100],
             "orientation_world_from_reference_xyzw": [0, 0, 0, 1]}
            for stamp in (0, 10, 20)
        ])
        _write_jsonl(attempt_dir / "flight_state.jsonl", [
            {"phase": "science", "simulation_epoch": 1, "simulation_timestamp_ns": stamp,
             "gt_timestamp_ns": stamp,
             "gt_reference_position_world_enu_m": [0, 400, 100],
             "reference_position_world_enu_m": [0, 400, 100]}
            for stamp in (0, 10, 20)
        ] + [{
            "phase": "science", "simulation_epoch": 1, "simulation_timestamp_ns": 30,
            "gt_timestamp_ns": 40_000_031,
            "gt_reference_position_world_enu_m": [0, 400, 100],
            "reference_position_world_enu_m": [0, 400, 100],
        }
        ])
        (attempt_dir / "checksums.sha256").write_text("fixture\n", encoding="utf-8")

    _write_json(root / "run_status.json", {"planned_run_count": 39, "valid_run_count": 39, "run_slots": rows})
    _write_json(root / "workbook_campaign_plan.json", {
        "campaign": {"campaign_id": "test_v12", "protocol_id": "stereo_calibration_campaign_v1_2"},
        "planned_runs": planned,
    })
    analysis_root.mkdir(parents=True, exist_ok=True)
    protocol_spec_path = root / "frozen_protocol" / "v1_2_specification.json"
    protocol_spec = {
        "schema_version": "stereo_calibration_campaign_v1_2", "document_version": "1.2",
        "reprocessing": {"yaw_bias_cases_deg": [{"id": f"Y{index}"} for index in range(7)]},
        "split_policy": {"limitation": "39 runs do not establish field performance."},
    }
    _write_json(protocol_spec_path, protocol_spec)
    freeze = {"status": "FROZEN_BEFORE_V0_POSITION_EVALUATION", "v0_position_error_read_before_freeze": False,
              "mc_trial_count": 100, "selected_path_id": "P2"}
    _write_json(analysis_root / "analysis_freeze.json", freeze)
    predictions = []
    for path_id in ("P0", "P1", "P2", "P3", "P4"):
        for repetition in range(1, 6):
            for method in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint"):
                predictions.append({"path_id": path_id, "repetition": repetition, "method": method,
                    "attitude_variant": "RPY_IMU", "predicted_position_uncertainty_mean_sigma_m": 0.01 + repetition / 1000,
                    "predicted_position_uncertainty_p95_sigma_m": 0.02})
    selection = {"selected_path_id": "P2", "candidate_summary": [
        {"path_id": path_id, "predicted_position_uncertainty_mean_sigma_m": score,
         "reference_duration_mean_s": 25 + index}
        for index, (path_id, score) in enumerate(
            (("P0", 0.12), ("P1", 0.11), ("P2", 0.10), ("P3", 0.13), ("P4", 0.14))
        )
    ], "target_geometry": {
        "frame": "world_enu:east,north,up_m", "distances_m": [600, 550, 500, 450, 400]},
        "per_calibration_prediction": predictions}
    path_selection_path = analysis_root / "path_selection.json"
    _write_json(path_selection_path, selection)
    freeze["path_selection_sha256"] = hashlib.sha256(path_selection_path.read_bytes()).hexdigest()
    _write_json(analysis_root / "analysis_freeze.json", freeze)
    freeze_sha = hashlib.sha256((analysis_root / "analysis_freeze.json").read_bytes()).hexdigest()
    _write_json(analysis_root / "analysis_summary.json", {
        "analysis_id": analysis_id, "analysis_freeze_sha256": freeze_sha,
        "mc_trial_count": 100, "selected_path_id": "P2", "source_valid_run_count": 39,
        "v0_position_errors_evaluated_after_freeze": True, "v0_recalibration_performed": False,
        "main_comparison_summary": [{
            "data_scope": scope, "method": method, "attitude_variant": variant,
            "evaluation_case_count": 18100, "source_calibration_run_count": 31,
            "source_evaluation_run_count": 8, "offline_mc_trial_count": 100,
            "metric_available_case_count": 18100, "position_3d_rmse_mean_m": 0.1,
            "position_3d_p95_mean_m": 0.2, "detection_rate_mean": 0.9,
            "valid_measurement_rate_mean": 0.8,
        } for scope in ("all_data", "matched_200")
          for method in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint")
          for variant in ("RPY_IMU", "YAW_ONLY")],
        "calibration_convergence_rates": {},
    })
    _write_json(analysis_root / "analysis_run_manifest.json", {
        "analysis_id": analysis_id,
        "source_campaign_manifest_sha256": {
            "frozen_protocol/v1_2_specification.json": hashlib.sha256(protocol_spec_path.read_bytes()).hexdigest(),
            "run_status.json": hashlib.sha256((root / "run_status.json").read_bytes()).hexdigest(),
            "workbook_campaign_plan.json": hashlib.sha256((root / "workbook_campaign_plan.json").read_bytes()).hexdigest(),
        },
        "source_attempt_checksums_sha256": {
            f"{slot['run_id']}/{slot['attempts'][0]['attempt_id']}": hashlib.sha256(
                (root / slot["recording_path"] / "checksums.sha256").read_bytes()
            ).hexdigest()
            for slot in rows
        },
    })
    metric_fields = ["rig_group", "calibration_run_id", "evaluation_run_id", "trial_index", "yaw_case", "data_scope", "method",
                     "attitude_variant", "evaluation_status", "detection_rate", "valid_measurement_rate",
                     "lateral_east_rmse_m", "forward_north_rmse_m", "vertical_up_rmse_m", "position_3d_rmse_m", "position_3d_p95_m"]
    with (analysis_root / "evaluation_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=metric_fields)
        writer.writeheader()
        def comparison_rows():
            for planned_row in (row for row in planned if row["use_for_calibration"]):
                group = planned_row["rig_group"]
                yaw_cases = tuple(f"Y{index}" for index in range(7)) if group == "G0" else ("G1_PRIMARY_ZERO",)
                for method in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint"):
                    for variant in ("RPY_IMU", "YAW_ONLY"):
                        for scope in ("all_data", "matched_200"):
                            for yaw_case in yaw_cases:
                                for trial in range(100):
                                    yield {
                                        "rig_group": group,
                                        "calibration_run_id": planned_row["run_id"],
                                        "evaluation_run_id": f"{group}-V0-R{planned_row['repetition']:02d}",
                                        "trial_index": trial, "yaw_case": yaw_case, "data_scope": scope,
                                        "method": method, "attitude_variant": variant,
                                        "evaluation_status": "EVALUATED", "detection_rate": 0.9,
                                        "valid_measurement_rate": 0.8, "lateral_east_rmse_m": 0.03,
                                        "forward_north_rmse_m": 0.04, "vertical_up_rmse_m": 0.05,
                                        "position_3d_rmse_m": 0.1 + (int(yaw_case[1:]) / 1000 if yaw_case.startswith("Y") else 0) + trial / 10000,
                                        "position_3d_p95_m": 0.2 + (int(yaw_case[1:]) / 1000 if yaw_case.startswith("Y") else 0) + trial / 10000,
                                    }
        writer.writerows(comparison_rows())
    calibration_fields = ["rig_group", "calibration_run_id", "method", "attitude_variant", "convergence_status"]
    with (analysis_root / "calibration_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=calibration_fields)
        writer.writeheader()
        writer.writerows(
            {"rig_group": group, "calibration_run_id": f"{group}-P0-R01",
             "method": method, "attitude_variant": variant,
             "convergence_status": "CONVERGED" if group == "G0" else "RANK_DEFICIENT"}
            for group in ("G0", "G1")
            for method in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint")
            for variant in ("RPY_IMU", "YAW_ONLY")
        )
    variant_fields = ["family", "condition_kind", "condition_id", "data_scope", "rig_group", "path_id",
                      "physical_repetition", "method", "rmse_delta_rpy_minus_yaw_m",
                      "p95_delta_rpy_minus_yaw_m", "comparison_status"]
    with (analysis_root / "attitude_variant_comparisons.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=variant_fields)
        writer.writeheader()
        writer.writerow({"family": "primary", "condition_kind": "yaw_case", "condition_id": "Y0",
                         "data_scope": "all_data", "rig_group": "G0", "path_id": "P2",
                         "physical_repetition": 1, "method": "A_rtk_independent",
                         "rmse_delta_rpy_minus_yaw_m": "-0.01", "p95_delta_rpy_minus_yaw_m": "-0.02",
                         "comparison_status": "COMPARABLE_CASE_METRIC"})
    checksum_lines = []
    for path in sorted(analysis_root.rglob("*")):
        if path.is_file() and path.name != "analysis_checksums.sha256":
            relative = path.relative_to(analysis_root).as_posix()
            checksum_lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {relative}\n")
    (analysis_root / "analysis_checksums.sha256").write_text("".join(checksum_lines), encoding="utf-8")
    _write_json(root / "analysis_latest.json", {
        "analysis_id": analysis_id, "analysis_summary": f"analysis_versions/{analysis_id}/analysis_summary.json",
        "analysis_freeze_sha256": freeze_sha,
    })
    audit = root / "source_evidence" / "audit.json"
    _write_json(audit, {"status": "PASS", "analysis_id": analysis_id, "scope": "coverage/schema/provenance only"})
    return audit


def test_v3_report_is_built_from_frozen_current_campaign(tmp_path):
    pytest.importorskip("xlsxwriter")
    audit = _make_campaign(tmp_path)

    manifest = report.build_report(tmp_path, audit)

    output = tmp_path / "reports" / "stereo_v1_2_summary_report_ready_v3.xlsx"
    assert output.is_file()
    assert manifest["run_count"] == 39
    assert manifest["analysis_id"] == "analysis_test_v3"
    assert manifest["worksheet_names"] == list(report.SHEET_NAMES)
    assert manifest["dashboard_chart_count"] == 22
    assert manifest["chart_count"] == 23
    with zipfile.ZipFile(output) as archive:
        workbook_xml = archive.read("xl/workbook.xml").decode("utf-8")
        assert all(name in workbook_xml for name in report.SHEET_NAMES)
        chart_files = [name for name in archive.namelist() if name.startswith("xl/charts/chart") and name.endswith(".xml")]
        assert len(chart_files) == 23
        mc_xml = ElementTree.fromstring(archive.read("xl/worksheets/sheet9.xml"))
        namespace = {"x": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        cells = {cell.attrib["r"]: cell for cell in mc_xml.findall(".//x:c", namespace)}
        assert cells["J2"].find("x:v", namespace).text == "0.9"
        assert cells["J2"].get("t") != "s"
        comparison_xml = ElementTree.fromstring(archive.read("xl/worksheets/sheet5.xml"))
        comparison_cells = {
            cell.attrib["r"]: cell
            for cell in comparison_xml.findall(".//x:c", namespace)
        }
        assert _cell_value(comparison_cells["A3"], namespace) == "rig_group"
        assert [_cell_value(comparison_cells[f"A{row}"], namespace) for row in range(4, 28)] == ["G0"] * 12 + ["G1"] * 12
        assert _cell_value(comparison_cells["E4"], namespace) == "17500"
        assert _cell_value(comparison_cells["E16"], namespace) == "600"
        variant_xml = ElementTree.fromstring(archive.read("xl/worksheets/sheet15.xml"))
        variant_cells = {cell.attrib["r"]: cell for cell in variant_xml.findall(".//x:c", namespace)}
        assert variant_cells["I2"].find("x:v", namespace).text == "-0.01"
        assert variant_cells["I2"].get("t") != "s"
        run_xml = ElementTree.fromstring(archive.read("xl/worksheets/sheet17.xml"))
        run_cells = {cell.attrib["r"]: cell for cell in run_xml.findall(".//x:c", namespace)}
        assert run_cells["G48"].find("x:v", namespace).text == "700"
        assert abs(float(run_cells["B5"].find("x:v", namespace).text) - 0.10795) < 1e-12
        run_chart_xml = archive.read("xl/charts/chart23.xml").decode("utf-8")
        assert "'16_실행별V0'!$A$5:$A$9" in run_chart_xml
        tracking_xml = ElementTree.fromstring(archive.read("xl/worksheets/sheet18.xml"))
        tracking_cells = {cell.attrib["r"]: cell for cell in tracking_xml.findall(".//x:c", namespace)}
        assert tracking_cells["F9"].find("x:v", namespace).text == "3"
        assert tracking_cells["H9"].find("x:v", namespace).text == "3"
        assert float(tracking_cells["E9"].find("x:v", namespace).text) == 0
        assert float(tracking_cells["G9"].find("x:v", namespace).text) == 0

    input_hashes = manifest["inputs_sha256"]
    assert "원본_실행별/G0-V0-R01/a01/ground_truth.jsonl" in input_hashes
    assert "원본_실행별/G0-V0-R01/a01/flight_state.jsonl" in input_hashes
    assert manifest["comparison_case_counts_by_group"] == {"G0": 17500, "G1": 600}
    assert manifest["legacy_pooled_analysis_summary_reaggregated"] is True
    assert any(Path(name).name == "analysis_checksums.sha256" for name in manifest["inputs_sha256"])


def test_calibration_convergence_report_rows_keep_rig_groups_separate():
    rows = report._calibration_convergence_by_group([
        {"rig_group": "G0", "calibration_run_id": "G0-P0-R01", "method": "A_rtk_independent",
         "attitude_variant": "RPY_IMU", "convergence_status": "CONVERGED"},
        {"rig_group": "G1", "calibration_run_id": "G1-P0-R01", "method": "A_rtk_independent",
         "attitude_variant": "RPY_IMU", "convergence_status": "RANK_DEFICIENT"},
    ])

    assert [row["rig_group"] for row in rows] == ["G0", "G1"]
    assert [row["nonconverged_fit_count"] for row in rows] == [0, 1]


def test_paired_variant_chart_rows_average_only_comparable_primary_y0_cases():
    base = {
        "family": "primary", "condition_kind": "yaw_case", "condition_id": "Y0",
        "data_scope": "all_data", "comparison_status": "COMPARABLE_CASE_METRIC",
        "rig_group": "G0", "path_id": "P2", "physical_repetition": "1",
        "method": "A_rtk_independent",
    }
    rows = [
        {**base, "rmse_delta_rpy_minus_yaw_m": "-0.01"},
        {**base, "rmse_delta_rpy_minus_yaw_m": "-0.03"},
        {**base, "condition_id": "Y1", "rmse_delta_rpy_minus_yaw_m": "-0.9"},
        {**base, "comparison_status": "DIFFERENT_VALID_COUNTS", "rmse_delta_rpy_minus_yaw_m": "-0.8"},
    ]

    chart_rows = report._paired_variant_run_rows(rows, "rmse_delta_rpy_minus_yaw_m")

    assert chart_rows == [["G0-P2-R01 / A_rtk_independent · n=2", -0.02]]


def test_report_uses_the_first_valid_attempt_after_a_technical_retry():
    slot = {
        "run_id": "G0-P2-R03",
        "attempts": [
            {"run_id": "G0-P2-R03", "attempt_id": "a01", "status": "INFRASTRUCTURE_FAILED",
             "recording_path": "runs/G0-P2-R03/a01"},
            {"run_id": "G0-P2-R03", "attempt_id": "a02", "status": "VALID_WITH_WARNINGS",
             "recording_path": "runs/G0-P2-R03/a02"},
            {"run_id": "G0-P2-R03", "attempt_id": "a03", "status": "VALID",
             "recording_path": "runs/G0-P2-R03/a03"},
        ],
    }

    assert report._first_valid_attempt(slot)["attempt_id"] == "a02"


def test_report_rejects_path_selection_changed_after_freeze(tmp_path):
    audit = _make_campaign(tmp_path)
    latest = json.loads((tmp_path / "analysis_latest.json").read_text())
    analysis_root = tmp_path / "analysis_versions" / latest["analysis_id"]
    selection_path = analysis_root / "path_selection.json"
    selection = json.loads(selection_path.read_text())
    selection["selected_path_id"] = "P1"
    _write_json(selection_path, selection)
    _refresh_analysis_checksums(analysis_root)

    with pytest.raises(ValueError, match="path selection hash"):
        report.build_report(tmp_path, audit)


def test_v3_report_rejects_unfrozen_or_non_enu_v0_source(tmp_path):
    audit = _make_campaign(tmp_path)
    latest = json.loads((tmp_path / "analysis_latest.json").read_text())
    analysis_root = tmp_path / "analysis_versions" / latest["analysis_id"]
    freeze = json.loads((analysis_root / "analysis_freeze.json").read_text())
    freeze["v0_position_error_read_before_freeze"] = True
    _write_json(analysis_root / "analysis_freeze.json", freeze)
    latest["analysis_freeze_sha256"] = hashlib.sha256((analysis_root / "analysis_freeze.json").read_bytes()).hexdigest()
    _write_json(tmp_path / "analysis_latest.json", latest)
    summary = json.loads((analysis_root / "analysis_summary.json").read_text())
    summary["analysis_freeze_sha256"] = latest["analysis_freeze_sha256"]
    _write_json(analysis_root / "analysis_summary.json", summary)
    _refresh_analysis_checksums(analysis_root)

    with pytest.raises(ValueError, match="freeze contract"):
        report.build_report(tmp_path, audit)


def test_first_valid_attempt_status_must_match_run_slot_status():
    slot = {
        "run_id": "G0-P2-R03", "status": "VALID",
        "attempts": [{"run_id": "G0-P2-R03", "attempt_id": "a02",
                      "status": "VALID_WITH_WARNINGS", "recording_path": "runs/G0-P2-R03/a02"}],
    }
    with pytest.raises(ValueError, match="status differs"):
        report._first_valid_attempt(slot)


@pytest.mark.parametrize("tamper", ["source_run_status", "analysis_metric"])
def test_report_rejects_campaign_or_analysis_data_changed_after_audit(tmp_path, tamper):
    audit = _make_campaign(tmp_path)
    if tamper == "source_run_status":
        status_path = tmp_path / "run_status.json"
        payload = json.loads(status_path.read_text())
        payload["run_slots"][0]["recording_path"] = "changed/path"
        _write_json(status_path, payload)
        expected = "campaign source manifest hash mismatch"
    else:
        metrics_path = tmp_path / "analysis_versions/analysis_test_v3/evaluation_metrics.csv"
        with metrics_path.open("a", encoding="utf-8") as handle:
            handle.write("tampered\n")
        expected = "analysis checksum mismatch"
    with pytest.raises(ValueError, match=expected):
        report.build_report(tmp_path, audit)
