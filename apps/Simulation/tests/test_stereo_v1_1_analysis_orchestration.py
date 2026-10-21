from __future__ import annotations

import hashlib
import json
import pytest
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


@pytest.mark.parametrize("workers", [None, 2])
def test_analyzer_cli_accepts_a_separate_read_only_source_root(tmp_path, monkeypatch, workers):
    output_root = tmp_path / "analysis-output"
    input_root = tmp_path / "existing-campaign"
    captured = {}
    monkeypatch.setattr(sys, "argv", [
        "analyze_campaign.py", "--output-root", str(output_root),
        "--input-root", str(input_root), "--mc-trials", "7",
    ])
    if workers is not None:
        sys.argv.extend(["--workers", str(workers)])
    monkeypatch.setattr(
        analyze_campaign, "analyze",
        lambda output, trials, input_root=None, workers=1: captured.update(
            output=output, trials=trials, input_root=input_root, workers=workers,
        ) or {},
    )

    assert analyze_campaign.main() == 0
    assert captured == {"output": output_root, "trials": 7, "input_root": input_root, "workers": workers or 1}


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



def _source_qc_fixture(root, *, workbook):
    from campaigns.stereo_v1_1.workbook_campaign import _plan_digest
    sources = [{"run_id": f"slot-{i}", "protocol_version": "1.2",
                "parameters": {"Capture": {"min_gt_rate_hz": 50}},
                "status": "PLANNED", "attempt_count": 0, "recording_path": None}
               for i in range(39)]
    slots = [{**(row if workbook else {"run_id": row["run_id"]}),
              "status": "VALID", "attempt_count": 1,
              "recording_path": f"raw/{row['run_id']}/a01",
              "attempts": [{"attempt_id": "a01", "recording_path": f"raw/{row['run_id']}/a01"}]}
             for row in sources]
    status = {"planned_run_count": 39, "valid_run_count": 39, "run_slots": slots,
              "schema_version": "stereo_v1_1_run_status_v1", "workbook_sha256": None,
              "campaign_plan_sha256": "legacy plans also have a digest"}
    if workbook:
        folder = root / "frozen_protocol"
        folder.mkdir()
        (folder / "input_parameters.xlsx").write_bytes(b"frozen workbook test fixture")
        digest = hashlib.sha256((folder / "input_parameters.xlsx").read_bytes()).hexdigest()
        plan = {"planned_runs": sources, "workbook_sha256": digest}
        (root / "workbook_campaign_plan.json").write_text(json.dumps(plan))
        status.update(workbook_sha256=digest, campaign_plan_sha256=_plan_digest(plan))
    (root / "run_status.json").write_text(json.dumps(status))
    return sources


@pytest.mark.parametrize("counted", [True, False])
def test_source_qc_uses_verified_frozen_workbook_conditions(tmp_path, monkeypatch, counted):
    sources = _source_qc_fixture(tmp_path, workbook=True)
    seen = []
    def validate(path, run_id, attempt_id, **kwargs):
        seen.append(kwargs["planned_source"])
        return {"status": "VALID", "counted_in_39": False, "counted_in_campaign": counted}
    monkeypatch.setattr(analyze_campaign, "validate_attempt", validate)
    if not counted:
        with pytest.raises(ValueError, match="fresh source QC differs"):
            analyze_campaign._status_slots(tmp_path)
        return
    _, slots = analyze_campaign._status_slots(tmp_path)
    assert len(slots) == 39
    assert seen == sources
    assert all(row["status"] == "PLANNED" for row in seen)


def test_source_qc_preserves_legacy_invocation(tmp_path, monkeypatch):
    _source_qc_fixture(tmp_path, workbook=False)
    seen = []
    def validate(path, run_id, attempt_id):
        seen.append(run_id)
        return {"status": "VALID", "counted_in_39": True}
    monkeypatch.setattr(analyze_campaign, "validate_attempt", validate)
    analyze_campaign._status_slots(tmp_path)
    assert len(seen) == 39


def test_source_qc_rejects_inconsistent_workbook_before_qc(tmp_path, monkeypatch):
    import pytest
    _source_qc_fixture(tmp_path, workbook=True)
    def unexpected(*args, **kwargs):
        raise AssertionError("QC must not run with unverified workbook inputs")
    monkeypatch.setattr(analyze_campaign, "validate_attempt", unexpected)
    plan_path = tmp_path / "workbook_campaign_plan.json"
    workbook_path = tmp_path / "frozen_protocol/input_parameters.xlsx"
    status_path = tmp_path / "run_status.json"
    original = {p: p.read_bytes() for p in (plan_path, workbook_path, status_path)}
    for target in (plan_path, workbook_path):
        target.write_bytes(b"{}")
        with pytest.raises(ValueError):
            analyze_campaign._status_slots(tmp_path)
        target.write_bytes(original[target])
        target.unlink()
        with pytest.raises(ValueError):
            analyze_campaign._status_slots(tmp_path)
        target.write_bytes(original[target])
    status = json.loads(original[status_path])
    status["run_slots"][0]["parameters"]["Capture"]["min_gt_rate_hz"] = 1
    status_path.write_text(json.dumps(status))
    with pytest.raises(ValueError, match="differs from frozen workbook"):
        analyze_campaign._status_slots(tmp_path)
    status = json.loads(original[status_path])
    status["run_slots"][0]["run_id"] = status["run_slots"][1]["run_id"]
    status_path.write_text(json.dumps(status))
    with pytest.raises(ValueError, match="source slots differ"):
        analyze_campaign._status_slots(tmp_path)


def test_every_mc_trial_covers_full_main_matrix_and_preserves_gps(tmp_path, monkeypatch):
    spec = json.loads((SIM_ROOT / 'campaigns/stereo_v1_1/protocol/03_experiment_spec.json').read_text())
    slots = [{'run_id': f'{group}-{path}-R{rep:02}', 'rig_group': group,
              'path_id': path, 'repetition': rep, 'attempts': [{'attempt_id': 'a01'}]}
             for group, paths, repetitions in (
                 ('G0', ('P0', 'P1', 'P2', 'P3', 'P4', 'V0'), 5),
                 ('G1', ('P0', 'P4', 'V0'), 3))
             for rep in range(1, repetitions + 1) for path in paths]
    calls, generated_trials = [], []
    def sensors(root, trial, *, output_root):
        generated_trials.append(trial)
        return []
    def calibrate(root, slot, trial, yaw_case, bias, scope, *args):
        calls.append((trial, slot['run_id'], yaw_case, scope))
        cache, analysis_root = args[2], args[3]
        assert not cache or set(cache) == {(slot['run_id'], trial)}
        cache[(slot['run_id'], trial)] = 'current input'
        results = [{'method': method, 'attitude_variant': variant}
                   for method in analyze_campaign.METHODS for variant in analyze_campaign.VARIANTS]
        directory = analyze_campaign._analysis_case_dir(analysis_root, slot, trial, yaw_case, scope)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'calibration_results.json').write_text(json.dumps({'results': results}))
        return {}, results
    monkeypatch.setattr(analyze_campaign, 'generate_camera_sensors', sensors)
    monkeypatch.setattr(analyze_campaign, '_calibrate_scope', calibrate)
    manifest, selection, _, _ = analyze_campaign._calibrate_all(tmp_path, slots, spec, 2, tmp_path / 'analysis')
    expected_per_trial = set()
    for slot in slots:
        if slot['path_id'] == 'V0':
            continue
        cases = ([f'Y{i}' for i in range(7)] if slot['rig_group'] == 'G0'
                 else ['G1_PRIMARY_ZERO']) + [analyze_campaign.GPS_YAW90_CASE]
        expected_per_trial.update((slot['run_id'], case, scope)
                                  for case in cases for scope in ('all_data', 'matched_200'))
    assert len(expected_per_trial) == 424
    assert generated_trials == [0, 1]
    assert len(calls) == 848
    for trial in (0, 1):
        assert {(run, case, scope) for t, run, case, scope in calls if t == trial} == expected_per_trial
    assert manifest['calibration_result_count'] == 848 * 6
    assert all('results' not in row and row['calibration_results_sha256'] for row in manifest['calibration_batches'])
    assert len(selection) == 25
    assert all(case == 'Y0' and scope == 'all_data' for _, case, scope in selection)


def test_analysis_protocol_requires_frozen_v12_and_records_actual_hash(tmp_path):
    import pytest
    _source_qc_fixture(tmp_path, workbook=True)
    status = json.loads((tmp_path / 'run_status.json').read_text())
    frozen = tmp_path / 'frozen_protocol/v1_2_specification.json'
    with pytest.raises(ValueError, match='requires frozen_protocol'):
        analyze_campaign._load_analysis_protocol(tmp_path, status)
    for schema, version in [('stereo_calibration_campaign_v1_1', '1.2'),
                            ('stereo_calibration_campaign_v1_2', '1.1')]:
        frozen.write_text(json.dumps({'schema_version': schema, 'document_version': version}))
        with pytest.raises(ValueError, match='must declare v1.2'):
            analyze_campaign._load_analysis_protocol(tmp_path, status)
    payload = {'schema_version': 'stereo_calibration_campaign_v1_2',
               'document_version': '1.2', 'source_marker': 'frozen campaign'}
    frozen.write_text(json.dumps(payload))
    protocol, provenance = analyze_campaign._load_analysis_protocol(tmp_path, status)
    assert protocol == payload
    assert provenance['source'] == 'frozen_protocol/v1_2_specification.json'
    assert provenance['sha256'] == hashlib.sha256(frozen.read_bytes()).hexdigest()


def test_analysis_protocol_legacy_uses_bundled_contract(tmp_path):
    _source_qc_fixture(tmp_path, workbook=False)
    status = json.loads((tmp_path / 'run_status.json').read_text())
    protocol, provenance = analyze_campaign._load_analysis_protocol(tmp_path, status)
    bundled = SIM_ROOT / 'campaigns/stereo_v1_1/protocol/03_experiment_spec.json'
    assert protocol == json.loads(bundled.read_text())
    assert provenance['source'] == 'bundled_protocol/03_experiment_spec.json'
    assert provenance['sha256'] == hashlib.sha256(bundled.read_bytes()).hexdigest()


def test_sensitivity_executes_every_source_repetition_and_mc_trial(tmp_path, monkeypatch):
    spec = json.loads((SIM_ROOT / 'campaigns/stereo_v1_1/protocol/03_experiment_spec.json').read_text())
    scenario = analyze_campaign._sensitivity_scenarios(spec)[0]
    monkeypatch.setattr(analyze_campaign, '_sensitivity_scenarios', lambda spec: [scenario])
    slots = [{'run_id': f'G0-{path}-R{rep:02d}', 'rig_group': 'G0',
              'path_id': path, 'repetition': rep, 'attempts': [{'attempt_id': 'a01'}]}
             for rep in (1, 2) for path in ('P0', 'P3', 'V0')]
    calls = []
    def sensor(root, run_id, trial, scenario_id, **kwargs):
        calls.append((run_id, trial))
        return {'shared_sensor_key': f'{run_id[-3:]}:{trial}'}, {}
    monkeypatch.setattr(analyze_campaign, 'make_sensitivity_observation', sensor)
    monkeypatch.setattr(analyze_campaign, '_sensitivity_pairs', lambda *args: ([], {}, None))
    def fake_rtk_inputs(*args):
        input_dir = args[-1]
        input_dir.mkdir(parents=True, exist_ok=True)
        (input_dir / 'drone_rtk_observations.jsonl').write_text('')
        (input_dir / 'drone_rtk_generation_provenance.json').write_text('{}')
        return [], [], {}
    monkeypatch.setattr(analyze_campaign, '_sensitivity_rtk_inputs', fake_rtk_inputs)
    output = tmp_path / 'analysis'
    output.mkdir()
    manifest = analyze_campaign._run_sensitivity_calibrations(
        tmp_path, slots, spec, {'selected_path_id': 'P3'}, output, mc_trials=2)
    assert set(calls) == {(f'G0-{path}-R{rep:02d}', trial)
                          for path in ('P0', 'P3') for rep in (1, 2) for trial in (0, 1)}
    assert len(calls) == 8  # one sensor realization per scenario/run/trial
    assert len(manifest['cases']) == 16
    assert {case['data_scope'] for case in manifest['cases']} == {'all_data', 'matched_200'}
    assert manifest['mc_trials_per_sensitivity_case'] == 2
    for case in manifest['cases']:
        repetition = case['calibration_run_id'][-3:]
        assert case['evaluation_run_id'] == f'G0-V0-{repetition}'
        folder = output / case['case_path']
        config = json.loads((folder / 'trial_config.json').read_text())
        assert config['mc_seed'] == config['trial_index'] == case['trial_index']
        assert (folder / 'calibration_checksums.json').is_file()
        results = json.loads((folder / 'calibration_results.json').read_text())['results']
        assert {(r['method'], r['attitude_variant']) for r in results} == {
            (m, v) for m in analyze_campaign.METHODS for v in analyze_campaign.VARIANTS}

    # Persist evaluator/CSV handoff on synthetic missing-input outcomes only.
    (output / 'analysis_freeze.json').write_text('{}')
    monkeypatch.setattr(analyze_campaign, '_attempt_dir', lambda *args: tmp_path)
    monkeypatch.setattr(analyze_campaign, '_camera_truth', lambda *args: {})
    monkeypatch.setattr(analyze_campaign, '_read_jsonl', lambda *args: [])
    monkeypatch.setattr(analyze_campaign, 'GroundTruthSeries', lambda *args: None)
    def evaluate(row, *args):
        return {'method': row['method'], 'attitude_variant': row['attitude_variant'],
                'calibration_status': row['convergence_status'], 'mc_seed': row['mc_seed'],
                'evaluation_status': 'CALIBRATION_FAILED', 'metrics': {}}
    monkeypatch.setattr(analyze_campaign, '_evaluation_record', evaluate)
    summary = analyze_campaign._evaluate_sensitivity_cases(tmp_path, slots, spec, output, manifest)
    assert summary['case_count'] == 16
    assert len(summary['summary_rows']) == 24  # two scopes x two paths x six method/variants
    for row in summary['summary_rows']:
        assert row['path_id'] in ('P0', 'P3') and row['path_count'] == 1
        assert row['physical_repetition_count'] == 2
        assert row['mc_trial_count'] == 2 and row['evaluation_case_count'] == 4
    import csv
    with (output / 'sensitivity_evaluation_metrics.csv').open() as handle:
        csv_rows = list(csv.DictReader(handle))
    assert len(csv_rows) == 96
    assert {row['data_scope'] for row in csv_rows} == {'all_data', 'matched_200'}
    assert {row['trial_index'] for row in csv_rows} == {'0', '1'}
    assert {row['physical_repetition'] for row in csv_rows} == {'1', '2'}


def test_analysis_summary_separates_mc_cases_from_physical_runs_and_fit_failures():
    payloads = []
    for run, trial, status, rmse in [(1, 0, 'CONVERGED', 1.),
                                      (1, 1, 'OPTIMIZER_FAILED', 3.),
                                      (2, 0, 'NOT_RUN_INSUFFICIENT_INPUT', None)]:
        row = {'method': 'A_rtk_independent', 'attitude_variant': 'RPY_IMU',
               'data_scope': 'all_data', 'calibration_run_id': f'G0-P0-R{run:02d}',
               'evaluation_run_id': f'G0-V0-R{run:02d}', 'mc_seed': trial,
               'metrics': None if rmse is None else {'position_3d_rmse_m': rmse}}
        payloads.append({'results': [row], 'calibration_results': [
            {'method': row['method'], 'attitude_variant': row['attitude_variant'],
             'convergence_status': status}]})
    summary = analyze_campaign._summarize_evaluations(payloads)
    row = summary['main_comparison_summary'][0]
    assert row['evaluation_case_count'] == 3
    assert row['source_calibration_run_count'] == row['source_evaluation_run_count'] == 2
    assert row['offline_mc_trial_count'] == row['metric_available_case_count'] == 2
    assert row['position_3d_rmse_mean_m'] == 2.
    rates = summary['calibration_convergence_rates']['A_rtk_independent__RPY_IMU']
    assert rates['attempted_fit_count'] == 2 and rates['not_run_or_other_status_count'] == 1
    assert rates['nonconvergence_fraction_among_attempted_fits'] == .5
    assert rates['failure_or_not_run_fraction_all_cases'] == 2/3


def test_evaluation_preserves_valid_estimates_when_another_pair_is_invalid(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(analyze_campaign, 'triangulate_stereo_pairs',
                        lambda *args, **kwargs: np.array([[0., 10., 1.], [np.nan, np.nan, np.nan]]))
    truth = SimpleNamespace(align=lambda *args: SimpleNamespace(
        status='EXACT', position_world_enu_m=[0., 10., 0.]))
    pairs = [{'simulation_epoch': 0, 'timestamp_ns': t,
              'left_uv_px': [640., 360.], 'right_uv_px': [640., 360.]}
             for t in (0, 50_000_000)]
    metrics = analyze_campaign._evaluate_pose(
        pairs, {'nominal_science_pair_count': 2, 'both_detected_science_pair_count': 2},
        {'left': [-1., 0., 0.], 'right': [1., 0., 0.]},
        {'left': R_WC_NOMINAL, 'right': R_WC_NOMINAL},
        {'fx_px': 500., 'fy_px': 500., 'cx_px': 640., 'cy_px': 360.}, truth)
    assert metrics['valid_position_estimate_count'] == metrics['reconstruction_failure_count'] == 1
    assert metrics['valid_measurement_rate'] == .5 and metrics['detection_rate'] == 1.
    assert metrics['position_3d_rmse_m'] == 1. and metrics['position_3d_p95_m'] == 1.


def test_evaluation_inputs_reuse_only_current_static_attempt(tmp_path, monkeypatch):
    calls = {'pairs': 0, 'gt': 0}
    def image_pairs(root, slot):
        calls['pairs'] += 1
        return [{'pair_id': f'p{i}', 'timestamp_ns': i, 'simulation_epoch': 0}
                for i in range(250)], {'nominal_science_pair_count': 250}
    def gt(rows):
        calls['gt'] += 1
        return object()
    monkeypatch.setattr(analyze_campaign, '_image_pairs', image_pairs)
    monkeypatch.setattr(analyze_campaign, 'GroundTruthSeries', gt)
    monkeypatch.setattr(analyze_campaign, '_read_jsonl', lambda p: [])
    monkeypatch.setattr(analyze_campaign, '_read_json', lambda p: {'path': str(p)})
    monkeypatch.setattr(analyze_campaign, '_camera_truth', lambda p: {})
    monkeypatch.setattr(analyze_campaign, '_select_time_stratified', lambda rows, n: rows[:n])
    slot = {'run_id': 'G0-V0-R01', 'attempts': [{'attempt_id': 'a01', 'recording_path': 'raw/a01'}]}
    cache = {}
    first = None
    for trial in range(3):
        for scope in ('all_data', 'matched_200'):
            result = analyze_campaign._v0_evaluation_inputs(tmp_path, slot, trial, scope, cache)
            first = result['gt_series'] if first is None else first
            assert result['gt_series'] is first
            assert f'trial_{trial:03d}' in result['sensor']['path']
            assert len(result['pairs']) == (200 if scope == 'matched_200' else 250)
    assert calls == {'pairs': 1, 'gt': 1}
    assert 'sensor' not in cache and len(cache) == 6
    slot['attempts'] = [{'attempt_id': 'a02', 'recording_path': 'raw/a02'}]
    assert analyze_campaign._v0_evaluation_inputs(tmp_path, slot, 0, 'all_data', cache)['gt_series'] is not first
    assert calls == {'pairs': 2, 'gt': 2}
    assert len(cache) == 5


def test_disk_case_iterator_repeatable_handoff_and_tamper_detection(tmp_path):
    import pytest
    from campaigns.stereo_v1_1.analysis_workbooks import export_analysis_workbooks
    (tmp_path / 'analysis_freeze.json').write_text('{}')
    freeze_hash = hashlib.sha256(b'{}').hexdigest()
    batches = []
    for yaw in ('Y0', 'GPS_YAW90'):
        case = tmp_path / 'derived' / yaw
        case.mkdir(parents=True)
        identity = {'trial_index': 0, 'yaw_case': yaw, 'data_scope': 'all_data'}
        calibration = {'source_run_id': 'G0-P0-R01', **identity,
                       'results': [{'method': 'A_rtk_independent', 'attitude_variant': 'RPY_IMU',
                                    'convergence_status': 'CONVERGED', 'camera_dof': 6}]}
        evaluation = {'calibration_run_id': 'G0-P0-R01', 'evaluation_run_id': 'G0-V0-R01',
                      **identity, 'freeze_sha256': freeze_hash, 'results': [{
                          'method': 'A_rtk_independent', 'attitude_variant': 'RPY_IMU',
                          'calibration_run_id': 'G0-P0-R01', 'evaluation_run_id': 'G0-V0-R01',
                          'mc_seed': 0, 'data_scope': 'all_data', 'evaluation_status': 'EVALUATED',
                          'metrics': {'position_3d_rmse_m': 0.25, 'detection_rate': 0.5}}]}
        for name, payload in [('calibration_results.json', calibration), ('evaluation_results.json', evaluation),
                              ('trial_config.json', identity)]:
            (case / name).write_text(json.dumps(payload))
        batches.append({'run_id': 'G0-P0-R01', **identity, 'case_directory': case.relative_to(tmp_path).as_posix(),
                        **{name.replace('.json', '_sha256'): hashlib.sha256((case/name).read_bytes()).hexdigest()
                           for name in ('calibration_results.json', 'evaluation_results.json')}})
    calls = []
    def factory():
        calls.append(1)
        return analyze_campaign._iter_evaluation_payloads(tmp_path, batches)
    materialized = list(factory())
    assert materialized == list(factory())
    assert analyze_campaign._summarize_evaluations(iter(materialized)) == analyze_campaign._summarize_evaluations(factory())
    original = export_analysis_workbooks(tmp_path, materialized)
    calls.clear()
    assert export_analysis_workbooks(tmp_path, factory) == original
    assert len(calls) == 2
    for kind in ('calibration', 'evaluation'):
        rows = list(analyze_campaign._iter_metric_rows(materialized, kind))
        fields = list(rows[0])
        path = tmp_path / f'{kind}.csv'
        digest = analyze_campaign._write_csv_immutable(path, fields, rows)
        assert analyze_campaign._write_csv_immutable(path, fields, analyze_campaign._iter_metric_rows(factory(), kind)) == digest
    evaluation_path = tmp_path / batches[0]['case_directory'] / 'evaluation_results.json'
    evaluation_path.write_text('{}')
    with pytest.raises(ValueError, match='evaluation batch hash'):
        list(factory())
    assert not list(tmp_path.glob('.csv-*'))


def test_workbook_factory_must_be_repeatable_and_stable(tmp_path):
    import pytest
    from campaigns.stereo_v1_1.analysis_workbooks import export_analysis_workbooks
    (tmp_path / 'analysis_freeze.json').write_text('{}')
    (tmp_path / 'trial_config.json').write_text('{}')
    payload = {'yaw_case': 'Y0', 'case_directory': '.', 'results': [], 'calibration_results': []}
    with pytest.raises(ValueError, match='repeatable'):
        export_analysis_workbooks(tmp_path, iter([payload]))
    count = 0
    def changing():
        nonlocal count
        count += 1
        return iter([{**payload, 'trial_index': count}])
    with pytest.raises(ValueError, match='changed between passes'):
        export_analysis_workbooks(tmp_path, changing)
    assert not list(tmp_path.glob('*.xlsx'))


def test_streamed_mean_matches_statistics_without_retaining_samples():
    import statistics
    values = [1e16, 1., -1e16, 0.1, 0.2]
    mean = analyze_campaign._Mean()
    for value in values: mean.add(value)
    assert mean.value() == statistics.mean(values)
    assert mean.count == len(values)
    assert set(vars(mean)) == {'total', 'count'}


@pytest.mark.parametrize("workers, fail_calibration, drift", [(1, False, False), (2, False, False), (2, True, False), (2, False, True)])
def test_analyze_handoff_streams_fixture_cases_after_freeze(tmp_path, monkeypatch, workers, fail_calibration, drift):
    """Exercise real export/coordinator/freeze without fits or real V0 data."""
    spec = json.loads((SIM_ROOT/'campaigns/stereo_v1_1/protocol/03_experiment_spec.json').read_text())
    slot = {'run_id':'G0-P0-R01','rig_group':'G0','path_id':'P0','repetition':1,
            'attempts':[{'attempt_id':'a01','recording_path':'raw/a01'}]}
    (tmp_path/'environment_report.json').write_text('{"status":"PREFLIGHT_PASS"}')
    monkeypatch.setattr(analyze_campaign,'_status_slots',lambda root: ({'valid_run_count':39},[slot]))
    monkeypatch.setattr(analyze_campaign,'_load_analysis_protocol',lambda root,status:(spec,{'sha256':'fixture'}))
    monkeypatch.setattr(analyze_campaign,'_analysis_hashes',lambda: {})
    monkeypatch.setattr(analyze_campaign,'_analysis_dependencies',lambda: {})
    monkeypatch.setattr(analyze_campaign,'_analysis_version_id',lambda *args:'analysis_fixture')
    seen_workers = []
    def calibrate(root, slots, spec, trials, output, *, workers=1):
        seen_workers.append(workers)
        batches=[]
        for yaw in ('Y0','GPS_YAW90'):
            for scope in ('all_data','matched_200'):
                directory=analyze_campaign._analysis_case_dir(output,slot,0,yaw,scope)
                directory.mkdir(parents=True)
                rows=[{'method':method,'attitude_variant':variant,
                       'calibration_run_id':slot['run_id'],'evaluation_run_id':'G0-V0-R01',
                       'mc_seed':0,'data_scope':scope,
                       'convergence_status':'CONVERGED','camera_dof':6,'latent_dof':0}
                      for method in ('A_rtk_independent','B_rtk_joint','C_no_drone_rtk_joint')
                      for variant in ('RPY_IMU','YAW_ONLY')]
                bundle={'source_run_id':slot['run_id'],'trial_index':0,'yaw_case':yaw,'data_scope':scope,'results':rows}
                (directory/'calibration_results.json').write_text(json.dumps(bundle))
                (directory/'trial_config.json').write_text('{}')
                (directory/'calibration_checksums.json').write_text('{}')
                batches.append({'run_id':slot['run_id'],'trial_index':0,'yaw_case':yaw,'data_scope':scope,
                    'case_directory':directory.relative_to(output).as_posix(),
                    'calibration_results_sha256':analyze_campaign._sha256(directory/'calibration_results.json')})
        if fail_calibration:
            raise RuntimeError("worker failed after partial calibration")
        if drift:
            monkeypatch.setattr(analyze_campaign,'_analysis_hashes',lambda: {'changed.py':'new-version'})
        return {'calibration_batches':batches,'calibration_result_count':24},{},{},{}
    monkeypatch.setattr(analyze_campaign,'_calibrate_all',calibrate)
    monkeypatch.setattr(analyze_campaign,'_select_calibration_path',lambda *args:{'selected_path_id':'P0','status':'SELECTED'})
    def sensitivity(root,slots,spec,selection,output,*args,**kwargs):
        manifest={'cases':[]}
        (output/'sensitivity_calibration_manifest.json').write_text(json.dumps(manifest))
        (output/'sensitivity_calibration_checksums.json').write_text('{}')
        return manifest
    monkeypatch.setattr(analyze_campaign,'_run_sensitivity_calibrations',sensitivity)
    evaluation_calls=[]
    def evaluate(root,slots,batch,cache,spec,output,generated):
        freeze=output/'analysis_freeze.json'
        assert freeze.is_file()
        assert len(json.loads((output/'calibration_freeze_manifest.json').read_text())) == 4
        evaluation_calls.append(batch['yaw_case'])
        payload={'calibration_run_id':batch['run_id'],'evaluation_run_id':'G0-V0-R01',
                 **{k:batch[k] for k in ('trial_index','yaw_case','data_scope')},
                 'freeze_sha256':analyze_campaign._sha256(freeze),'results':[{
                    'method':method,'attitude_variant':variant,
                    'calibration_status':'CONVERGED',
                    'calibration_run_id':batch['run_id'],'evaluation_run_id':'G0-V0-R01',
                    'mc_seed':0,'data_scope':batch['data_scope'],'evaluation_status':'EVALUATED',
                    'metrics':{'position_3d_rmse_m':0.5,'valid_position_estimate_count':200}}
                    for method in ('A_rtk_independent','B_rtk_joint','C_no_drone_rtk_joint')
                    for variant in ('RPY_IMU','YAW_ONLY')]}
        (output/batch['case_directory']/'evaluation_results.json').write_text(json.dumps(payload))
        return payload
    monkeypatch.setattr(analyze_campaign,'_evaluate_calibration_batch',evaluate)
    monkeypatch.setattr(analyze_campaign,'_evaluate_sensitivity_cases',lambda *args: {'case_count':0,'trial_count_per_case':100})
    if fail_calibration:
        with pytest.raises(RuntimeError, match="worker failed"):
            analyze_campaign.analyze(tmp_path, 100, workers=workers)
        assert seen_workers == [workers]
        assert evaluation_calls == []
        version = tmp_path / 'analysis_versions/analysis_fixture'
        assert not (version / 'analysis_freeze.json').exists()
        assert list(version.rglob('calibration_results.json'))
        # Public facade must release the coordinator lease after failure.
        with analyze_campaign.analysis_output_lock(tmp_path):
            pass
        return
    if drift:
        with pytest.raises(RuntimeError, match="identity changed before V0 evaluation"):
            analyze_campaign.analyze(tmp_path,100,workers=workers)
        assert evaluation_calls == []
        version=tmp_path/'analysis_versions/analysis_fixture'
        assert (version/'analysis_freeze.json').is_file()
        assert not (version/'evaluation_admission.json').exists()
        assert not (tmp_path/'analysis_latest.json').exists()
        return
    result=analyze_campaign.analyze(tmp_path,100,workers=workers)
    assert seen_workers == [workers]
    assert evaluation_calls == ['Y0','Y0','GPS_YAW90','GPS_YAW90']
    assert result['calibration_batch_count'] == 4
    assert result['evaluation_result_count'] == 12  # original-yaw summary remains separate
    assert result['yaw_source_summaries']['gps_yaw90']['evaluation_result_count'] == 12
    version=tmp_path/'analysis_versions/analysis_fixture'
    index=json.loads((version/'calibration_run_index.json').read_text())
    assert index['schema_version'].endswith('_v2')
    assert all('results' not in row and 'evaluation_results_sha256' in row for row in index['batches'])
    pointer=json.loads((tmp_path/'analysis_latest.json').read_text())
    assert pointer['analysis_freeze_sha256'] == analyze_campaign._sha256(version/'analysis_freeze.json')
    for name in ('calibration_metrics.csv','evaluation_metrics.csv','gps_yaw90_calibration_metrics.csv',
                 'gps_yaw90_evaluation_metrics.csv','existing_yaw_results.xlsx','gps_yaw90_results.xlsx'):
        assert (version/name).is_file()


def _timestamp_pairs(stamps):
    return [{'pair_id':f'p{i:04d}', 'timestamp_ns':stamp, 'simulation_epoch':0}
            for i,stamp in enumerate(stamps)]


def test_matched200_keeps_feasible_gapped_comparisons_and_reports_gaps():
    for stamps in (list(range(200)), list(range(199))+[10000]):
        pairs = _timestamp_pairs(stamps)
        selected, evidence = analyze_campaign._matched_pair_selection(pairs, {})
        assert len(selected) == len({row['pair_id'] for row in selected}) == 200
        assert evidence['status'] == 'SELECTED'
        assert len(evidence['selected_minus_target_ns']) == 200
        assert evidence['max_source_gap_ns'] == max(b-a for a,b in zip(stamps,stamps[1:]))
        json.dumps(evidence,allow_nan=False)
    selected, evidence = analyze_campaign._matched_pair_selection(_timestamp_pairs(range(199)), {})
    assert selected == [] and evidence['status'] == 'INSUFFICIENT_UNIQUE_PAIRS'
    assert evidence['available_unique_pair_count'] == 199


def test_time_selection_is_global_monotone_unique_minimum_against_bruteforce():
    import itertools
    for stamps in ([0,1,2,9,10,30], [0,0,1,20,21,22], [1,2,3,4,5,6], [7,7,7,7,7,7]):
        pairs = _timestamp_pairs(stamps)
        count=3; span=stamps[-1]-stamps[0]
        def cost(rows):
            return sum(abs(2*count*(row['timestamp_ns']-stamps[0])-(2*k+1)*span)
                       for k,row in enumerate(rows))
        selected = analyze_campaign._select_time_stratified(pairs,count)
        assert cost(selected) == min(cost(candidate) for candidate in itertools.combinations(pairs,count))
        assert len({row['pair_id'] for row in selected}) == count
        assert selected == analyze_campaign._select_time_stratified(list(reversed(pairs)),count)
        assert [r['timestamp_ns'] for r in selected] == sorted(r['timestamp_ns'] for r in selected)


def test_time_selection_rejects_malformed_input_even_below_requested_count():
    import copy
    import pytest
    base=_timestamp_pairs([0,1])
    for mutation in ('duplicate_pair','mixed_epochs','nan','inf','float','missing_epoch','reused_frame'):
        pairs=copy.deepcopy(base)
        if mutation=='duplicate_pair': pairs[1]['pair_id']=pairs[0]['pair_id']
        elif mutation=='mixed_epochs': pairs[1]['simulation_epoch']=1
        elif mutation=='missing_epoch': pairs[1].pop('simulation_epoch')
        elif mutation=='reused_frame':
            for row in pairs: row['left_frame_id']='same-source-frame'
        else: pairs[1]['timestamp_ns']={'nan':float('nan'),'inf':float('inf'),'float':1.5}[mutation]
        with pytest.raises(ValueError): analyze_campaign._select_time_stratified(pairs,200)


def test_calibration_and_evaluation_selection_share_cached_feasible_200(tmp_path,monkeypatch):
    pairs=_timestamp_pairs(list(range(239))+[10000])
    data={'pairs':pairs,'rtk':[{'source_id':row['pair_id']} for row in pairs]}
    actual=analyze_campaign._select_time_stratified
    calls=[]
    def counted(rows,n):
        calls.append(id(rows)); return actual(rows,n)
    monkeypatch.setattr(analyze_campaign,'_select_time_stratified',counted)
    for _yaw in range(7):
        chosen,rtk,evidence=analyze_campaign._calibration_pair_subset(data,'matched_200')
        assert len(chosen)==len(rtk)==200 and evidence['status']=='SELECTED'
        assert [r['pair_id'] for r in chosen]==[r['source_id'] for r in rtk]
    assert len(calls)==1
    monkeypatch.setattr(analyze_campaign,'_image_pairs',lambda *args:(pairs,{'nominal_science_pair_count':len(pairs)}))
    monkeypatch.setattr(analyze_campaign,'_read_jsonl',lambda *args:[])
    monkeypatch.setattr(analyze_campaign,'GroundTruthSeries',lambda *args:None)
    monkeypatch.setattr(analyze_campaign,'_camera_truth',lambda *args:{})
    monkeypatch.setattr(analyze_campaign,'_read_json',lambda *args:{})
    slot={'run_id':'G0-V0-R01','attempts':[{'attempt_id':'a01','recording_path':'fixture/a01'}]}
    cache={}
    for trial in range(3):
        evaluated=analyze_campaign._v0_evaluation_inputs(tmp_path,slot,trial,'matched_200',cache)
        assert evaluated['matched_200_met'] is True
        assert evaluated['pairs']==chosen
        assert evaluated['matched_200_selection']['max_source_gap_ns'] > 9000
    assert len(calls)==2
