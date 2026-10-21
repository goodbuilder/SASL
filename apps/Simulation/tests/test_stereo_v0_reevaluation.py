import hashlib
import json
from pathlib import Path

import pytest

from campaigns.stereo_v1_1 import analyze_campaign
from campaigns.stereo_v1_1 import reevaluate_v0


def _json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _batch(run_id, trial, yaw="Y0", scope="all_data"):
    return {
        "run_id": run_id,
        "trial_index": trial,
        "yaw_case": yaw,
        "data_scope": scope,
        "case_directory": f"derived/{run_id}/a01/trial_{trial:03d}/{yaw}/{scope}",
        "calibration_results_sha256": "a" * 64,
    }


def test_batch_selection_requires_every_trial_yaw_and_sample_scope():
    slots = [{"run_id": "G0-P0-R01", "rig_group": "G0", "repetition": 1,
              "path_id": "P0"},
             {"run_id": "G0-V0-R01", "rig_group": "G0", "repetition": 1,
              "path_id": "V0"},
             {"run_id": "G0-P0-R02", "rig_group": "G0", "repetition": 2,
              "path_id": "P0"}]
    rows = [_batch("G0-P0-R01", trial, yaw, scope)
            for trial in range(100) for yaw in ("Y0", "GPS_YAW90")
            for scope in ("all_data", "matched_200")]
    selected_slots, selected = reevaluate_v0._select_calibration_batches(
        {"batches": rows}, slots, "G0", 1, 100,
    )
    assert [slot["run_id"] for slot in selected_slots] == ["G0-P0-R01"]
    assert len(selected) == 400
    assert {row["trial_index"] for row in selected} == set(range(100))

    with pytest.raises(ValueError, match="incomplete frozen MC batch grid"):
        reevaluate_v0._select_calibration_batches(
            {"batches": rows[:-1]}, slots, "G0", 1, 100,
        )
    with pytest.raises(ValueError, match="duplicate frozen calibration batch"):
        reevaluate_v0._select_calibration_batches(
            {"batches": [*rows, rows[0]]}, slots, "G0", 1, 100,
        )


def test_copy_calibration_case_is_verified_and_parent_remains_unchanged(tmp_path):
    parent_root = tmp_path / "parent_analysis"
    output_root = tmp_path / "new_reevaluation"
    batch = _batch("G0-P0-R01", 0)
    source_case = parent_root / batch["case_directory"]
    calibration_payload = {"source_run_id": batch["run_id"], "trial_index": 0,
                           "yaw_case": "Y0", "data_scope": "all_data", "results": []}
    sensor_payload = {"shared_sensor_key": "G0:R01:MC000", "monte_carlo_trial_index": 0}
    _json(source_case / "calibration_results.json", calibration_payload)
    _json(source_case / "sensor_observations.json", sensor_payload)
    source_checksums = {
        "calibration_results.json": _sha(source_case / "calibration_results.json"),
        "sensor_observations.json": _sha(source_case / "sensor_observations.json"),
    }
    _json(source_case / "calibration_checksums.json", source_checksums)
    batch["calibration_results_sha256"] = source_checksums["calibration_results.json"]
    parent = {
        "parent_analysis_root": parent_root,
        "calibration_freeze_manifest": {
            f"{batch['case_directory']}/calibration_checksums.json":
                _sha(source_case / "calibration_checksums.json"),
        },
    }
    source_result_bytes = (source_case / "calibration_results.json").read_bytes()
    reused = reevaluate_v0._copy_calibration_case(parent, batch, output_root, {})
    target_case = output_root / batch["case_directory"]
    assert _sha(target_case / "calibration_results.json") == reused["copied_calibration_results_sha256"]
    assert (target_case / "calibration_results.json").stat().st_ino != (
        source_case / "calibration_results.json").stat().st_ino
    assert (source_case / "calibration_results.json").read_bytes() == source_result_bytes
    assert analyze_campaign._read_json(target_case / "calibration_checksums.json") == {
        "calibration_results.json": _sha(target_case / "calibration_results.json"),
        "sensor_observations.json": _sha(target_case / "sensor_observations.json"),
    }

    (source_case / "sensor_observations.json").write_text("{}\n")
    with pytest.raises(ValueError, match="sensor observation is not covered"):
        reevaluate_v0._copy_calibration_case(parent, batch, output_root, {})


def test_evaluation_trial_config_is_copied_from_the_frozen_parent(tmp_path):
    parent_root = tmp_path / "parent_analysis"
    output_root = tmp_path / "new_reevaluation"
    output_root.mkdir(parents=True)
    _json(output_root / "analysis_freeze.json", {"status": "frozen"})
    batch = _batch("G0-P0-R01", 0)
    source_case = parent_root / batch["case_directory"]
    _json(source_case / "trial_config.json", {
        "gps_yaw90": {"ready": True}, "initial_yaw_rad": {"left": 0.0, "right": 0.0},
    })
    _json(source_case / "calibration_checksums.json", {
        "trial_config.json": _sha(source_case / "trial_config.json"),
    })
    parent = {
        "parent_analysis_root": parent_root,
        "calibration_freeze_manifest": {
            f"{batch['case_directory']}/calibration_checksums.json":
                _sha(source_case / "calibration_checksums.json"),
        },
    }

    count, copied_bytes = reevaluate_v0._copy_evaluation_trial_configs(
        parent, output_root, [batch],
    )

    target = output_root / batch["case_directory"] / "trial_config.json"
    assert count == 1
    assert copied_bytes == target.stat().st_size
    assert target.read_bytes() == (source_case / "trial_config.json").read_bytes()
    row = reevaluate_v0._read_jsonl(output_root / "evaluation_trial_config_index.jsonl")[0]
    assert row["copied_trial_config_sha256"] == _sha(target)
    assert reevaluate_v0._read_json(output_root / "evaluation_input_reuse_summary.json")[
        "calibration_recomputed"
    ] is False

    parent["calibration_freeze_manifest"][
        f"{batch['case_directory']}/calibration_checksums.json"
    ] = "0" * 64
    with pytest.raises(ValueError, match="source calibration checksum manifest differs"):
        reevaluate_v0._copy_evaluation_trial_configs(parent, output_root, [batch])


def test_evaluation_batch_runner_invokes_only_post_freeze_evaluator(tmp_path, monkeypatch):
    output_root = tmp_path / "reevaluation"
    output_root.mkdir()
    batch = _batch("G0-P0-R01", 0)
    slots = [{"run_id": batch["run_id"], "path_id": "P0", "rig_group": "G0",
              "repetition": 1, "attempts": [{"attempt_id": "a01"}]},
             {"run_id": "DEV-V0-NEW", "path_id": "V0", "rig_group": "G0",
              "repetition": 1, "attempts": [{"attempt_id": "a01"}]}]
    calls = []

    def evaluate(*args):
        calls.append(args[2]["run_id"])
        case = analyze_campaign._case_path(args[5], args[2])
        payload = {"calibration_run_id": args[2]["run_id"],
                   "evaluation_run_id": "DEV-V0-NEW", "trial_index": 0,
                   "yaw_case": "Y0", "data_scope": "all_data",
                   "freeze_sha256": "freeze", "results": [{"method": "A_rtk_independent"}]}
        _json(case / "evaluation_results.json", payload)
        return payload

    monkeypatch.setattr(analyze_campaign, "_evaluate_calibration_batch", evaluate)
    monkeypatch.setattr(analyze_campaign, "_calibrate_all",
                        lambda *args, **kwargs: pytest.fail("calibration MC must not rerun"))
    monkeypatch.setattr(analyze_campaign, "_run_sensitivity_calibrations",
                        lambda *args, **kwargs: pytest.fail("sensitivity calibration must not rerun"))
    count, elapsed = reevaluate_v0._evaluate_batches(
        output_root, slots, [batch], {}, {"camera": output_root / "generated_inputs" / "camera"},
    )
    assert calls == ["G0-P0-R01"]
    assert count == 1
    assert elapsed >= 0
    assert batch["evaluation_results_sha256"] == _sha(
        output_root / batch["case_directory"] / "evaluation_results.json"
    )


def test_yolo_outputs_must_match_pilot_parameters_and_pinned_model(tmp_path):
    attempt = tmp_path / "attempt"
    output = attempt / "yolo-8.4.127"
    output.mkdir(parents=True)
    parameters = {"Capture": {"yolo_confidence": 0.1, "yolo_nms_iou": 0.7,
                               "yolo_imgsz_px": 736, "yolo_target_hz": 20.0}}
    _json(attempt / "pilot_manifest.json", {"run_id": "DEV-V0", "attempt_id": "a01"})
    (attempt / "image_index.jsonl").write_text("{}\n")
    (attempt / "pair_index.jsonl").write_text("{}\n")
    for name in ("observations_left.jsonl", "observations_right.jsonl", "paired_observations.jsonl"):
        (output / name).write_text("{}\n")
    _json(output / "observation_summary.json", {
        "status": "COMPLETE", "run_id": "DEV-V0", "attempt_id": "a01",
        "model_sha256": "model", "ultralytics_version": "8.4.127",
        "confidence": 0.1, "nms_iou": 0.7, "inference_imgsz_px": 736,
        "requested_target_hz": 20.0,
        "image_index_sha256": _sha(attempt / "image_index.jsonl"),
        "pair_index_sha256": _sha(attempt / "pair_index.jsonl"),
    })
    detector = {"sha256": "model", "ultralytics_version": "8.4.127"}
    assert reevaluate_v0._verify_yolo_outputs(attempt, parameters, detector)["status"] == "COMPLETE"
    detector["sha256"] = "different"
    with pytest.raises(ValueError, match="YOLO processing identity differs"):
        reevaluate_v0._verify_yolo_outputs(attempt, parameters, detector)
