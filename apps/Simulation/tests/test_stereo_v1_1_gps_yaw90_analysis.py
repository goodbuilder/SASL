"""Independent branch parity, real solver/evaluator and saved-workbook checks."""
from __future__ import annotations

import copy
import importlib.util
import json
import math
import sys
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from campaigns.stereo_v1_1 import analyze_campaign as analysis
from campaigns.stereo_v1_1.analysis_workbooks import export_analysis_workbooks
from campaigns.stereo_v1_1.calibration_solver import solve_calibration
from campaigns.stereo_v1_1.gps_yaw90 import GPS_YAW90_CASE, campaign_yaw90, select_yaw90
from campaigns.stereo_v1_1.gt_alignment import GroundTruthSeries
from test_stereo_v1_1_calibration_solver import INTRINSICS, _fixture

SIM_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("north,east,left,right,reverse", [
    (0, 25, 0, 0, False), (0, 25, 180, 180, False),
    (25, 0, 270, 270, False), (2, -25, 359, 1, True),
    (0, 25, 0, 180, False), (0, 25, None, 0, False),
    (0, 25, None, None, False), (0, 25, None, None, True),
    (1, 1, float("nan"), 45, False),
])
def test_yaw90_matches_workstation_branch_selection(north, east, left, right, reverse):
    path = SIM_ROOT.parent / "Workstation/workstation_core/baseline_yaw.py"
    spec = importlib.util.spec_from_file_location("workstation_yaw90_parity", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    kwargs = dict(north_m=north, east_m=east, p1_yaw_deg=left,
                  p2_yaw_deg=right, reverse=reverse)
    expected = module.select_baseline_yaw(**kwargs).to_mapping()
    actual = select_yaw90(**kwargs)
    for key, value in actual.items():
        assert value == expected[key], key


def test_noisy_enu_baseline_heading_sign_and_truth_independence():
    sensor, _, _, _ = _fixture()
    sensor["cameras"]["right"]["optical_center_rtk_position_world_enu_m"] = [12.5, 0.1, 0.4]
    before = copy.deepcopy(sensor)
    yaw, evidence = campaign_yaw90(sensor)
    # East-going P1->P2 baseline tilted north: north-facing perpendicular
    # must point slightly west, independently of the vertical baseline.
    assert yaw["left"] == pytest.approx(-math.atan2(0.1, 25))
    assert yaw["left"] == yaw["right"]
    assert evidence["baseline_ned_m"] == pytest.approx([0.1, 25, -0.3])
    assert sensor == before
    sensor["ground_truth"] = {"yaw": 123}
    assert campaign_yaw90(sensor) == (yaw, evidence)
    sensor["cameras"]["right"]["optical_center_rtk_position_world_enu_m"] = [-12.5, 0, 10]
    yaw, evidence = campaign_yaw90(sensor)
    assert not evidence["ready"]
    assert yaw == {"left": None, "right": None}


def _read_sheet(path, number):
    ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read(f"xl/worksheets/sheet{number}.xml"))
    rows = []
    for row in root.findall("m:sheetData/m:row", ns):
        cells = {}
        for cell in row.findall("m:c", ns):
            text = cell.find("m:is/m:t", ns)
            number = cell.find("m:v", ns)
            cells[cell.attrib["r"].rstrip("0123456789")] = (
                text.text if text is not None else number.text if number is not None else None)
            assert cell.find("m:f", ns) is None
        rows.append(cells)
    return [{rows[0][key]: value for key, value in row.items()} for row in rows[1:]]


def test_both_sources_refit_all_methods_then_evaluate_and_reopen_two_workbooks(tmp_path):
    sensor, original_trial, pairs, rtk = _fixture()
    spec = json.loads((SIM_ROOT / "campaigns/stereo_v1_1/protocol/03_experiment_spec.json")
                      .read_text(encoding="utf-8-sig"))
    truth_rows = []
    for index, (pair, observation) in enumerate(zip(pairs, rtk)):
        pair.update(simulation_epoch=0, timestamp_ns=index * 20_000_000)
        truth_rows.append(dict(simulation_epoch=0, source_timestamp_ns=pair["timestamp_ns"],
            position_world_enu_m=observation["position_world_enu_m"],
            orientation_world_from_reference_xyzw=[0, 0, 0, 1]))
    truth = GroundTruthSeries(truth_rows)
    true_camera = {side: {"center": np.asarray(sensor["cameras"][side]["optical_center_rtk_position_world_enu_m"]),
                         "rotation": analysis.rotation_world_from_optical(0, 0, 0)}
                   for side in ("left", "right")}
    summary = dict(nominal_science_pair_count=len(pairs), both_detected_science_pair_count=len(pairs))
    payloads, initial_rotations = [], []
    for yaw_case in ("Y1", GPS_YAW90_CASE):
        trial = analysis._make_trial_config("G0", yaw_case, {"left": 0.3, "right": -0.25},
            spec, 0, "G0-P0-R01", "G0-V0-R01", "all_data", camera_sensor=sensor)
        solver_trial = {key: trial[key] for key in original_trial}
        results = [solve_calibration(method=method, attitude_variant=variant,
            sensor_observations=sensor, trial_config=solver_trial, image_pairs=pairs,
            intrinsics=INTRINSICS, drone_rtk_observations=None if method.startswith("C_") else rtk)
            for method in analysis.METHODS for variant in analysis.VARIANTS]
        assert len(results) == 6
        assert {row["method"] for row in results} == set(analysis.METHODS)
        centers, rotations = analysis._initial_pose(sensor, trial)
        initial_rotations.append(rotations["left"])
        m0 = dict(method="M0_sensor_only", attitude_variant="INITIAL", camera_dof=0,
            fixed_camera_centers_world_m=centers,
            estimated_R_WC={key: value.tolist() for key, value in rotations.items()},
            convergence_status="INITIAL_SENSOR_VALUES", rig_group="G0")
        evaluated = []
        for row in [*results, m0]:
            row["rig_group"] = "G0"
            evaluated.append(analysis._evaluation_record(row, "G0-V0-R01", "all_data",
                summary, pairs, true_camera, truth, spec))
        relative = f"derived/G0-P0-R01/a01/trial_000/{yaw_case}/all_data"
        directory = tmp_path / relative
        directory.mkdir(parents=True)
        (directory / "trial_config.json").write_text(json.dumps(trial))
        payloads.append(dict(calibration_run_id="G0-P0-R01", evaluation_run_id="G0-V0-R01",
            trial_index=0, yaw_case=yaw_case, data_scope="all_data", results=evaluated,
            calibration_results=results, case_directory=relative))
    assert not np.allclose(initial_rotations[0], initial_rotations[1])
    with pytest.raises(RuntimeError, match="frozen"):
        export_analysis_workbooks(tmp_path, payloads)
    (tmp_path / "analysis_freeze.json").write_text("{}\n")
    artifacts = export_analysis_workbooks(tmp_path, payloads)
    assert set(artifacts) == {"existing_yaw", "gps_yaw90"}
    for family, artifact in artifacts.items():
        path = tmp_path / artifact["path"]
        evaluation = _read_sheet(path, 1)
        calibration = _read_sheet(path, 2)
        assert len(evaluation) == 7 and len(calibration) == 6
        assert {row["yaw_source"] for row in evaluation} == {family}
        expected_case = GPS_YAW90_CASE if family == "gps_yaw90" else "Y1"
        assert {row["yaw_case"] for row in evaluation} == {expected_case}
        assert {row["attitude_variant"] for row in calibration} == set(analysis.VARIANTS)
        assert all(float(row["metrics.valid_position_estimate_count"]) == len(pairs) for row in evaluation)
        assert any(row["method"] == "M0_sensor_only" for row in evaluation)
    assert export_analysis_workbooks(tmp_path, payloads) == artifacts
    payloads[1]["results"][0]["evaluation_status"] = "changed"
    with pytest.raises(FileExistsError, match="different content"):
        export_analysis_workbooks(tmp_path, payloads)


def test_failed_batch_is_preserved_and_formula_like_reason_is_plain_text(tmp_path):
    (tmp_path / "analysis_freeze.json").write_text("{}")
    directory = tmp_path / "case"
    directory.mkdir()
    (directory / "trial_config.json").write_text("{}")
    payload = dict(calibration_run_id="G0-P0-R01", evaluation_run_id="G0-V0-R01",
        trial_index=0, yaw_case=GPS_YAW90_CASE, data_scope="matched_200",
        case_directory="case", evaluation_status="MATCHED_200_UNMET", results=[],
        calibration_results=[dict(method="C_no_drone_rtk_joint", attitude_variant="RPY_IMU",
                                 failure_reason="=1+2", convergence_status="RANK_DEFICIENT")])
    files = export_analysis_workbooks(tmp_path, [payload])
    path = tmp_path / files["gps_yaw90"]["path"]
    assert _read_sheet(path, 1)[0]["status"] == "MATCHED_200_UNMET"
    assert _read_sheet(path, 2)[0]["failure_reason"] == "=1+2"


def test_campaign_schedules_both_sources_and_keeps_original_path_selection(monkeypatch, tmp_path):
    slots = [dict(run_id=f"{group}-{path}-R01", rig_group=group, path_id=path,
                  repetition=1, attempts=[dict(attempt_id="a01")])
             for group in ("G0", "G1") for path in ("P0", "P4", "V0")]
    calls = []
    monkeypatch.setattr(analysis, "generate_camera_sensors", lambda *a, **kw: [])
    monkeypatch.setattr(analysis, "_yaw_cases", lambda group, spec: [("Y0", {})])
    def calibrate(root, slot, trial, yaw_case, bias, scope, *args):
        calls.append((slot["run_id"], trial, yaw_case, scope))
        results = [{"method": "fixture", "yaw_case": yaw_case}]
        directory = analysis._analysis_case_dir(args[3], slot, trial, yaw_case, scope)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "calibration_results.json").write_text(json.dumps({"results": results}))
        return {}, results
    monkeypatch.setattr(analysis, "_calibrate_scope", calibrate)
    manifest, selected, _, _ = analysis._calibrate_all(tmp_path, slots, {}, 2, tmp_path)
    assert not any("V0" in run for run, _, _, _ in calls)
    for slot in slots:
        if slot["path_id"] == "V0":
            continue
        for scope in (analysis.ALL_DATA, analysis.MATCHED_DATA):
            assert (slot["run_id"], 0, GPS_YAW90_CASE, scope) in calls
        assert (slot["run_id"], 1, GPS_YAW90_CASE, analysis.ALL_DATA) in calls
    assert manifest["calibration_result_count"] == len(calls)
    assert all(key[1] == "Y0" for key in selected)
    assert all(row["yaw_case"] != GPS_YAW90_CASE for rows in selected.values() for row in rows)
