#!/usr/bin/env python3
"""Fail-closed structural QC for one v1.1 immutable attempt directory.

This validator distinguishes raw-data infrastructure failures from recorded
flight/detector performance. It does not open V0 positioning errors or turn a
weak result into a reason for favorable reruns.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


CAMPAIGN_ROOT = Path(__file__).resolve().parent
SPEC = CAMPAIGN_ROOT / "protocol" / "03_experiment_spec.json"
MANIFEST = CAMPAIGN_ROOT / "protocol" / "04_run_manifest.json"
MODEL = CAMPAIGN_ROOT / "model_manifest.json"
OBSERVATION_REQUIRED = set(json.loads(SPEC.read_text(encoding="utf-8-sig"))["required_observation_fields"])


def _read_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read {path.name}: {exc}") from exc


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rows(path: Path):
    with path.open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            if not line.endswith("\n"):
                raise ValueError(f"{path.name}:{number} has no terminal newline")
            yield json.loads(line)


def _yolo_rate_errors(yolo_summary: dict, capture: dict, image_summary: dict) -> list[str]:
    errors = []
    requested = float(capture["yolo_target_hz"])
    actual_rates = yolo_summary.get("actual_source_simulation_rate_hz", {})
    for side in ("left", "right"):
        source_rate = image_summary.get(side, {}).get("observed_source_hz")
        observed_rate = actual_rates.get(side)
        try:
            expected_rate = min(requested, float(source_rate)) if source_rate is not None else 0.0
            observed_value = float(observed_rate) if observed_rate is not None else math.nan
        except (TypeError, ValueError):
            expected_rate, observed_value = 0.0, math.nan
        if (expected_rate <= 0 or not math.isfinite(observed_value)
                or observed_value + 1e-9 < expected_rate * 0.9):
            errors.append(f"{side}_yolo_actual_rate_below_workbook_target")
    return errors


def _workbook_run_manifest_errors(actual: dict, planned_source: dict) -> list[str]:
    """Compare run metadata with the workbook's normalized source row.

    Vehicle identity is declared in the workbook under ``parameters.Vehicle``;
    the compiled plan intentionally does not duplicate those values at its top
    level.
    """
    vehicle = planned_source["parameters"]["Vehicle"]
    expected = {
        "sequence": planned_source["sequence"],
        "run_id": planned_source["run_id"],
        "repetition": planned_source["repetition"],
        "role": planned_source["role"],
        "seed_requested": planned_source.get("seed_requested"),
        "workbook_sha256": planned_source["workbook_sha256"],
        "use_for_calibration": planned_source["use_for_calibration"],
        "use_for_final_evaluation": planned_source["use_for_final_evaluation"],
        "vehicle_model_id": vehicle.get("model_id", "x500_2m"),
        "vehicle_reference_link": vehicle.get("reference_link", "vehicle_reference_link"),
    }
    return [f"workbook_run_manifest_{key}_mismatch"
            for key, value in expected.items() if actual.get(key) != value]


def validate(attempt_dir: Path, run_id: str, attempt_id: str,
             planned_source: dict | None = None) -> dict:
    attempt_dir = attempt_dir.resolve()
    spec = _read_json(SPEC)
    workbook_source = planned_source is not None and "parameters" in planned_source
    planned = ({} if workbook_source else
               {row["run_id"]: row for row in _read_json(MANIFEST)["runs"]})
    model = _read_json(MODEL)
    development = run_id.startswith("DEVELOPMENT-")
    source = planned_source if workbook_source else planned.get(run_id)
    infrastructure_errors: list[str] = []
    protocol_errors: list[str] = []
    warnings: list[str] = []
    evidence = {}

    if workbook_source and planned_source.get("artifact_contract"):
        attempt_contract = [row for row in planned_source["artifact_contract"]
                            if row.get("validator_id") == "qc_attempt"]
        required = {row["path_pattern"] for row in attempt_contract if row.get("required")}
        checksum_required = {row["path_pattern"] for row in attempt_contract
                             if row.get("required") and row.get("checksum_required")}
    else:
        required = set(spec["required_run_files"])
        required.remove("qc_report.json")  # This command creates the report last.
        if workbook_source:
            required.update(("workbook_parameters.json", "effective_scenario.env"))
        checksum_required = required - {"checksums.sha256"}
    observation_required = (
        {row["field_id"] for row in planned_source.get("observation_contract", [])
         if row.get("required")}
        if workbook_source and planned_source.get("observation_contract")
        else OBSERVATION_REQUIRED
    )
    missing = sorted(name for name in required if not (attempt_dir / name).exists())
    if missing:
        infrastructure_errors.append("missing_required_run_files")
    evidence["missing_required_run_files"] = missing

    if workbook_source:
        manifest_path = attempt_dir / "run_manifest.json"
        parameters_path = attempt_dir / "workbook_parameters.json"
        effective_path = attempt_dir / "effective_config.json"
        if manifest_path.is_file():
            actual = _read_json(manifest_path)
            protocol_errors.extend(_workbook_run_manifest_errors(actual, planned_source))
            if actual.get("attempt_id") != attempt_id:
                protocol_errors.append("workbook_run_manifest_attempt_id_mismatch")
        if parameters_path.is_file() and _read_json(parameters_path) != planned_source["parameters"]:
            protocol_errors.append("workbook_parameter_row_mismatch")
        if effective_path.is_file():
            effective = _read_json(effective_path)
            if effective.get("workbook_sha256") != planned_source.get("workbook_sha256"):
                protocol_errors.append("effective_config_workbook_hash_mismatch")
            expected_model = planned_source["parameters"]["Vehicle"].get("model_id", "x500_2m")
            if effective.get("target_model_name") != expected_model:
                protocol_errors.append("effective_config_vehicle_model_differs_from_workbook")
            expected_reference = planned_source["parameters"]["Vehicle"].get(
                "reference_link", "vehicle_reference_link"
            )
            if effective.get("vehicle_reference_link") != expected_reference:
                protocol_errors.append("effective_config_vehicle_reference_differs_from_workbook")
            if (parameters_path.is_file()
                    and effective.get("workbook_parameters_sha256") != _sha256(parameters_path)):
                protocol_errors.append("effective_config_parameter_hash_mismatch")
            scenario_path = attempt_dir / "effective_scenario.env"
            if (scenario_path.is_file()
                    and effective.get("scenario_environment_sha256") != _sha256(scenario_path)):
                protocol_errors.append("effective_config_scenario_hash_mismatch")
            seed_policy = planned_source.get("seed_policy", "none")
            if effective.get("seed_policy") != seed_policy:
                protocol_errors.append("workbook_effective_config_seed_policy_mismatch")
            if seed_policy == "manifest_required":
                if effective.get("seed_requested") != planned_source.get("seed_requested"):
                    protocol_errors.append("workbook_requested_seed_differs_from_manifest")
                seed_effective = effective.get("seed_effective") or {}
                if not seed_effective.get("gazebo_cli", False):
                    protocol_errors.append("workbook_gazebo_seed_not_forwarded")
                if not seed_effective.get("independent_repetition_verified", False):
                    warnings.append("stochastic_repetition_independence_unverified")
            elif effective.get("seed_requested") is not None:
                protocol_errors.append("workbook_run_unexpected_seed")
    elif source is None:
        if not development:
            protocol_errors.append("run_id_not_in_39_slot_manifest")
    else:
        run_manifest_path = attempt_dir / "run_manifest.json"
        if run_manifest_path.is_file():
            actual = _read_json(run_manifest_path)
            for name in ("run_id", "rig_group", "path_id", "repetition", "seed_requested"):
                if actual.get(name) != source[name]:
                    protocol_errors.append(f"run_manifest_{name}_mismatch")
            if actual.get("attempt_id") != attempt_id:
                protocol_errors.append("run_manifest_attempt_id_mismatch")
        effective_path = attempt_dir / "effective_config.json"
        if effective_path.is_file():
            effective = _read_json(effective_path)
            if effective.get("seed_requested") != source["seed_requested"]:
                protocol_errors.append("requested_seed_differs_from_manifest")
            seed_effective = effective.get("seed_effective", {})
            if not seed_effective.get("gazebo_cli", False):
                protocol_errors.append("gazebo_seed_not_forwarded")
            if not seed_effective.get("independent_repetition_verified", False):
                warnings.append("stochastic_repetition_independence_unverified")

    for name, expected_status in (("raw_capture_summary.json", "CAPTURE_COMPLETE"),
                                  ("state_capture_summary.json", "STATE_CAPTURE_COMPLETE")):
        path = attempt_dir / name
        if not path.is_file():
            infrastructure_errors.append(f"missing_{name}")
            continue
        summary = _read_json(path)
        evidence[name] = {"status": summary.get("status"), "failure_reason": summary.get("failure_reason")}
        if summary.get("status") != expected_status:
            infrastructure_errors.append(f"{name}_not_finalized_valid")
        if name == "state_capture_summary.json":
            rate = summary.get("gt_observed_hz")
            evidence["gt_observed_hz"] = rate
            evidence["gt_gap_ge_40ms_count"] = summary.get("gt_gap_ge_40ms_count")
            min_rate = (planned_source["parameters"]["Capture"]["min_gt_rate_hz"]
                        if workbook_source else 49.5)
            if rate is None or not math.isfinite(float(rate)) or float(rate) + 1e-9 < min_rate:
                infrastructure_errors.append("gt_rate_below_50hz")
            if int(summary.get("gt_gap_ge_40ms_count", 0)) > 0:
                warnings.append("gt_bracketing_gaps_ge_40ms")
        if name == "raw_capture_summary.json":
            for side in ("left", "right"):
                stream = summary.get("streams", {}).get(side, {})
                if stream.get("topic_counts", {}).get(f"/{side}_camera/image", 0) <= 0:
                    infrastructure_errors.append(f"missing_{side}_original_images")
                for artifact in stream.get("files", ()):
                    path = attempt_dir / artifact["path_from_attempt_root"]
                    if not path.is_file() or _sha256(path) != artifact["sha256"]:
                        infrastructure_errors.append(f"{side}_bag_hash_mismatch")

    image_summary_path = attempt_dir / "image_index_summary.json"
    if not image_summary_path.is_file():
        infrastructure_errors.append("missing_image_index_summary")
    else:
        image_summary = _read_json(image_summary_path)
        evidence["image_index"] = {
            "left_frames": image_summary.get("left", {}).get("frame_count"),
            "right_frames": image_summary.get("right", {}).get("frame_count"),
            "pair_count": image_summary.get("pair_count"),
            "maximum_abs_pair_delta_ns": image_summary.get("maximum_abs_pair_delta_ns"),
        }
        if image_summary.get("status") != "INDEX_ONLY":
            infrastructure_errors.append("image_index_status_invalid")
        pair_tolerance_ns = (round(planned_source["parameters"]["Capture"]["pair_tolerance_s"] * 1e9)
                             if workbook_source else 1_000_000)
        if image_summary.get("pair_tolerance_ns") != pair_tolerance_ns:
            protocol_errors.append("image_index_pair_tolerance_mismatch")
        if (image_summary.get("maximum_abs_pair_delta_ns") is not None
                and image_summary["maximum_abs_pair_delta_ns"] > pair_tolerance_ns):
            protocol_errors.append("nominal_pair_skew_exceeds_1ms")
        for side in ("left", "right"):
            if image_summary.get(side, {}).get("duplicate_source_stamp_count", 0):
                warnings.append(f"{side}_duplicate_source_stamp")

    gt_alignment_path = attempt_dir / "gt_alignment_summary.json"
    if not gt_alignment_path.is_file():
        infrastructure_errors.append("missing_gt_alignment_summary")
    else:
        alignment = _read_json(gt_alignment_path)
        evidence["gt_alignment"] = {
            "all_images": alignment.get("image_count_by_alignment_status"),
            "science_images": alignment.get("science_image_count_by_alignment_status"),
        }
        science_bad = sum(
            int(alignment.get("science_image_count_by_alignment_status", {}).get(name, 0))
            for name in ("EXTRAPOLATION", "UNAVAILABLE", "GAP_GE_40MS")
        )
        if science_bad:
            infrastructure_errors.append("science_images_lack_valid_gt")
        all_bad = sum(
            int(alignment.get("image_count_by_alignment_status", {}).get(name, 0))
            for name in ("EXTRAPOLATION", "UNAVAILABLE", "GAP_GE_40MS")
        )
        if all_bad:
            warnings.append("some_images_lack_valid_gt")
        tracking = alignment.get("tracking")
        close_tracking = (planned_source.get("use_for_final_evaluation", False) if workbook_source
                          else run_id in planned and planned[run_id]["path_id"] == "V0")
        if close_tracking and alignment.get("tracking_metrics_revealed"):
            protocol_errors.append("v0_tracking_metrics_opened_before_freeze")
        warning_limit = (planned_source["parameters"]["Motion"]["tracking_warning_p95_m"]
                         if workbook_source else 1.0)
        if tracking and tracking.get("position_3d_p95_m") is not None and tracking["position_3d_p95_m"] > warning_limit:
            warnings.append("tracking_position_p95_above_configured_limit")

    camera_truth_path = attempt_dir / "camera_truth.json"
    if camera_truth_path.is_file():
        camera_truth = _read_json(camera_truth_path).get("camera_payloads", {})
        if set(camera_truth) != {"left", "right"}:
            infrastructure_errors.append("camera_truth_missing_eye")
        for side in ("left", "right"):
            payload = camera_truth.get(side, {})
            position_error = payload.get("desired_to_observed_optical_delta_norm_m")
            rotation_error = payload.get("desired_to_observed_optical_orientation_delta_deg")
            if position_error is None or rotation_error is None or position_error > 1e-6 or rotation_error > 1e-5:
                protocol_errors.append(f"{side}_physical_camera_pose_mismatch")

    observation_summary_path = attempt_dir / "observation_summary.json"
    if not observation_summary_path.is_file():
        infrastructure_errors.append("missing_observation_summary")
    else:
        observation_summary = _read_json(observation_summary_path)
        evidence["observation_summary_status"] = observation_summary.get("status")
        if observation_summary.get("status") != "COMPLETE":
            infrastructure_errors.append("yolo_observations_not_fully_processed")
        if observation_summary.get("ultralytics_version") != model["ultralytics_version"]:
            protocol_errors.append("yolo_runtime_version_mismatch")
        if observation_summary.get("model_sha256") != model["sha256"]:
            protocol_errors.append("yolo_model_hash_mismatch")

    observations = {}
    for side in ("left", "right"):
        path = attempt_dir / f"observations_{side}.jsonl"
        if not path.is_file():
            continue
        observed = list(_rows(path))
        observations[side] = observed
        if len({row.get("frame_id") for row in observed}) != len(observed):
            infrastructure_errors.append(f"duplicate_{side}_frame_id")
        if image_summary_path.is_file() and len(observed) != image_summary[side]["frame_count"]:
            infrastructure_errors.append(f"{side}_observation_frame_count_mismatch")
        for row in observed:
            if not observation_required.issubset(row):
                infrastructure_errors.append(f"{side}_observation_fields_missing")
                break
            if row["run_id"] != run_id or row["attempt_id"] != attempt_id or row["camera_id"] != side:
                infrastructure_errors.append(f"{side}_observation_identity_mismatch")
                break
            if row["yolo_weights_sha256"] != model["sha256"]:
                protocol_errors.append(f"{side}_yolo_weight_mismatch")
                break
            status = row["detection_status"]
            bbox = row["bbox_xyxy_px"]
            center = row["bbox_center_uv_px"]
            if status == "DETECTED":
                if not isinstance(bbox, list) or len(bbox) != 4 or not isinstance(center, list) or len(center) != 2:
                    infrastructure_errors.append(f"{side}_detected_bbox_missing")
                    break
            elif status == "NOT_DETECTED":
                if bbox is not None or center is not None:
                    infrastructure_errors.append(f"{side}_missing_detection_encoded_as_point")
                    break
            elif status == "INFERENCE_ERROR":
                infrastructure_errors.append(f"{side}_yolo_inference_error")
                break
            elif status == "SKIPPED_RATE_LIMIT":
                if bbox is not None or center is not None or row.get("inference_end_timestamp_ns") is not None:
                    infrastructure_errors.append(f"{side}_rate_limited_frame_has_detection_data")
                    break
            else:
                infrastructure_errors.append(f"{side}_unknown_detection_status")
                break
    evidence["observation_counts"] = {side: len(rows) for side, rows in observations.items()}
    paired_path = attempt_dir / "paired_observations.jsonl"
    if paired_path.is_file():
        pairs = list(_rows(paired_path))
        evidence["paired_observation_count"] = len(pairs)
        if image_summary_path.is_file() and len(pairs) != image_summary["pair_count"]:
            infrastructure_errors.append("paired_observation_count_mismatch")
        if len({pair.get("left_frame_id") for pair in pairs}) != len(pairs) or len({pair.get("right_frame_id") for pair in pairs}) != len(pairs):
            infrastructure_errors.append("source_frame_reused_in_pair")
        pair_tolerance_ns = (round(planned_source["parameters"]["Capture"]["pair_tolerance_s"] * 1e9)
                             if workbook_source else 1_000_000)
        if any(abs(int(pair["pair_delta_ns"])) > pair_tolerance_ns for pair in pairs):
            protocol_errors.append("paired_observation_skew_exceeds_1ms")

    controller_path = attempt_dir / "controller_result.json"
    controller_status = None
    if not controller_path.is_file():
        infrastructure_errors.append("missing_controller_result")
    else:
        controller = _read_json(controller_path)
        controller_status = controller.get("controller_status")
        evidence["controller_status"] = controller_status
        evidence["controller_failure_reason"] = controller.get("failure_reason")
        if controller.get("controller_status") != "science_completed":
            reason = controller.get("failure_reason", "")
            if any(term in reason for term in ("telemetry became stale", "simulation clock reset", "local frame reset")):
                infrastructure_errors.append("controller_runtime_boundary_failure")
            else:
                warnings.append("science_not_completed_performance_outcome")

    if workbook_source:
        coordinate_path = attempt_dir / "coordinate_frame_check.json"
        if not coordinate_path.is_file():
            if controller_status == "science_completed":
                infrastructure_errors.append("missing_coordinate_frame_check")
            else:
                warnings.append("coordinate_frame_check_not_reached")
        else:
            coordinate = _read_json(coordinate_path)
            evidence["coordinate_frame_check"] = {
                "status": coordinate.get("status"),
                "local_start_position_error_m": coordinate.get("local_start_position_error_m"),
                "world_start_position_error_m": coordinate.get("world_start_position_error_m"),
                "verify_enu_ned_axes_requested": coordinate.get("verify_enu_ned_axes_requested"),
                "vehicle_spawn_yaw_check": coordinate.get("vehicle_spawn_yaw_check"),
            }
            frame_gate = planned_source["parameters"]["FrameGate"]
            entry = planned_source["parameters"]["Entry"]
            vehicle = planned_source["parameters"]["Vehicle"]
            if coordinate.get("status") != "PASS":
                protocol_errors.append("coordinate_frame_check_not_passed")
            if coordinate.get("run_id") != run_id or coordinate.get("attempt_id") != attempt_id:
                protocol_errors.append("coordinate_frame_check_identity_mismatch")
            if coordinate.get("verify_enu_ned_axes_requested") is not True:
                protocol_errors.append("enu_ned_axis_verification_not_requested")
            if coordinate.get("origin_observations_required") != frame_gate["origin_min_samples"]:
                protocol_errors.append("coordinate_frame_origin_sample_requirement_mismatch")
            if (coordinate.get("same_simulation_epoch_required") is not True
                    or coordinate.get("stable_reset_counters_required") is not True):
                protocol_errors.append("coordinate_frame_origin_stability_requirements_mismatch")
            for key, limit in (("local_start_position_error_m", entry["science_start_position_error_m"]),
                               ("world_start_position_error_m", entry["science_start_position_error_m"])):
                observed = coordinate.get(key)
                if observed is None or not math.isfinite(float(observed)) or float(observed) > limit:
                    protocol_errors.append(f"coordinate_frame_{key}_exceeds_workbook")
            spawn_check = coordinate.get("vehicle_spawn_yaw_check", {})
            if vehicle["verify_spawn_yaw"]:
                error_deg = spawn_check.get("absolute_error_deg")
                tolerance_deg = spawn_check.get("tolerance_deg")
                if (spawn_check.get("status") != "PASS" or spawn_check.get("requested") is not True
                        or spawn_check.get("simulator_spawn_yaw_deg") != vehicle["spawn_yaw_deg"]
                        or error_deg is None or tolerance_deg is None
                        or not math.isfinite(float(error_deg)) or not math.isfinite(float(tolerance_deg))
                        or float(error_deg) > float(tolerance_deg)):
                    protocol_errors.append("vehicle_spawn_yaw_check_differs_from_workbook")
            elif spawn_check.get("status") != "NOT_REQUESTED":
                protocol_errors.append("vehicle_spawn_yaw_check_request_mismatch")

    if workbook_source and observation_summary_path.is_file():
        yolo_summary = _read_json(observation_summary_path)
        capture = planned_source["parameters"]["Capture"]
        for name, expected in (("confidence", capture["yolo_confidence"]),
                               ("nms_iou", capture["yolo_nms_iou"]),
                               ("inference_imgsz_px", capture["yolo_imgsz_px"]),
                               ("requested_target_hz", capture["yolo_target_hz"])):
            observed = yolo_summary.get(name)
            if observed is None or not math.isclose(float(observed), float(expected), rel_tol=0, abs_tol=1e-12):
                protocol_errors.append(f"yolo_{name}_differs_from_workbook")
        if yolo_summary.get("parameter_source") != "manifest_xlsx":
            protocol_errors.append("yolo_parameter_source_not_workbook")
        if image_summary_path.is_file():
            protocol_errors.extend(_yolo_rate_errors(yolo_summary, capture, image_summary))
    if workbook_source and gt_alignment_path.is_file():
        expected_gap = planned_source["parameters"]["Capture"]["max_gt_bracketing_gap_s"]
        actual_gap = _read_json(gt_alignment_path).get("maximum_bracketing_gap_s")
        if actual_gap is None or not math.isclose(float(actual_gap), float(expected_gap), rel_tol=0, abs_tol=1e-12):
            protocol_errors.append("gt_alignment_gap_differs_from_workbook")

    checksums_path = attempt_dir / "checksums.sha256"
    if checksums_path.is_file():
        listed = set()
        for line in checksums_path.read_text(encoding="utf-8").splitlines():
            try:
                expected, relative = line.split("  ", 1)
            except ValueError:
                infrastructure_errors.append("malformed_checksums_file")
                break
            path = (attempt_dir / relative).resolve()
            if not path.is_relative_to(attempt_dir):
                infrastructure_errors.append("checksums_path_escapes_attempt")
                break
            listed.add(relative)
            if not path.is_file() or _sha256(path) != expected:
                infrastructure_errors.append("checksums_file_mismatch")
                break
        for name in required:
            if name == "checksums.sha256":
                continue  # A checksum list cannot contain its own stable digest.
            path = attempt_dir / name
            if name in checksum_required and path.is_file() and name not in listed:
                infrastructure_errors.append("required_file_not_covered_by_checksums")
                break
        if (workbook_source and coordinate_path.is_file()
                and str(coordinate_path.relative_to(attempt_dir)) not in listed):
            infrastructure_errors.append("coordinate_frame_check_not_covered_by_checksums")
        raw_dir = attempt_dir / "raw_images_or_lossless_bag"
        if raw_dir.is_dir() and any(
            str(path.relative_to(attempt_dir)) not in listed
            for path in raw_dir.rglob("*") if path.is_file()
        ):
            infrastructure_errors.append("raw_bag_file_not_covered_by_checksums")

    if development:
        status = "DEVELOPMENT"
    elif protocol_errors:
        status = "PROTOCOL_DEVIATION"
    elif infrastructure_errors:
        status = "INFRASTRUCTURE_FAILED"
    elif warnings:
        status = "VALID_WITH_WARNINGS"
    else:
        status = "VALID"
    return {
        "schema_version": "stereo_v1_1_attempt_qc_v1",
        "run_id": run_id,
        "attempt_id": attempt_id,
        "status": status,
        "counted_in_39": (status in ("VALID", "VALID_WITH_WARNINGS") and not development
                          and not workbook_source),
        "counted_in_campaign": status in ("VALID", "VALID_WITH_WARNINGS") and not development,
        "infrastructure_errors": sorted(set(infrastructure_errors)),
        "protocol_errors": sorted(set(protocol_errors)),
        "warnings": sorted(set(warnings)),
        "evidence": evidence,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--planned-source-json", type=Path)
    parser.add_argument("--report-path", type=Path)
    args = parser.parse_args()
    source = _read_json(args.planned_source_json) if args.planned_source_json else None
    report = validate(args.attempt_dir, args.run_id, args.attempt_id, planned_source=source)
    path = args.report_path or args.attempt_dir / "qc_report.json"
    with path.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"status": report["status"], "counted_in_39": report["counted_in_39"],
                      "counted_in_campaign": report["counted_in_campaign"],
                      "infrastructure_errors": report["infrastructure_errors"],
                      "protocol_errors": report["protocol_errors"],
                      "warnings": report["warnings"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
