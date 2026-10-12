#!/usr/bin/env python3
"""Export compact, read-only v1.1 run metrics for a human summary workbook.

No V0 tracking or positioning metric is read before the offline method freeze.
Every metric names its sample scope; raw stream and GT files stay per attempt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _analysis_handoff(root: Path) -> dict:
    pointer_path = root / "analysis_latest.json"
    if not pointer_path.is_file():
        return {
            "offline_analysis_status": "NOT_RUN",
            "analysis_id": None,
            "analysis_summary_path": None,
            "selected_path_id": None,
            "calibration_result_count": None,
            "evaluation_result_count": None,
            "v0_position_metrics_available_after_freeze": False,
        }
    pointer = _read(pointer_path)
    summary_path = (root / pointer["analysis_summary"]).resolve()
    if not summary_path.is_relative_to(root.resolve()) or not summary_path.is_file():
        raise ValueError("analysis_latest.json points outside the campaign root or to a missing summary")
    analysis_root = summary_path.parent
    freeze_path = analysis_root / "analysis_freeze.json"
    if not freeze_path.is_file():
        raise ValueError("latest analysis summary has no freeze record")
    observed_freeze_hash = hashlib.sha256(freeze_path.read_bytes()).hexdigest()
    if observed_freeze_hash != pointer["analysis_freeze_sha256"]:
        raise ValueError("latest analysis freeze hash does not match its pointer")
    summary = _read(summary_path)
    if summary.get("analysis_id") != pointer.get("analysis_id"):
        raise ValueError("latest analysis ID differs between pointer and summary")
    return {
        "offline_analysis_status": "ANALYSIS_COMPLETE",
        "analysis_id": pointer["analysis_id"],
        "analysis_summary_path": str(summary_path.relative_to(root.resolve())),
        "selected_path_id": summary.get("selected_path_id"),
        "calibration_result_count": summary.get("calibration_result_count"),
        "evaluation_result_count": summary.get("evaluation_result_count"),
        "v0_position_metrics_available_after_freeze": bool(
            summary.get("v0_position_errors_evaluated_after_freeze")
        ),
    }


def summarize(root: Path, *, allow_partial: bool = False) -> dict:
    root = root.resolve()
    status = _read(root / "run_status.json")
    if status["planned_run_count"] != 39 or len(status["run_slots"]) != 39:
        raise ValueError("campaign status is not the 39-slot manifest")
    if not allow_partial and status["valid_run_count"] != 39:
        raise ValueError("39 valid source runs are required for a final summary")
    runs = []
    for slot in status["run_slots"]:
        row = {
            "run_id": slot["run_id"], "rig_group": slot["rig_group"],
            "path_id": slot["path_id"], "repetition": slot["repetition"],
            "seed_requested": slot["seed_requested"],
            "status": slot["status"], "attempt_count": slot["attempt_count"],
            "recording_path": slot["recording_path"],
            "failure_reason": slot["failure_reason"],
            "controller_status": None, "controller_failure_reason": None,
            "seed_independence_verified": False,
            "science_reference_duration_s": None, "science_reference_length_3d_m": None,
            "left_source_frames_full_capture": None, "right_source_frames_full_capture": None,
            "left_observed_source_hz": None, "right_observed_source_hz": None,
            "left_unmatched_frames": None, "right_unmatched_frames": None,
            "maximum_abs_pair_delta_ns": None,
            "left_detected_full_capture": None, "right_detected_full_capture": None,
            "left_detection_rate_full_capture": None, "right_detection_rate_full_capture": None,
            "nominal_pairs_full_capture": None, "both_detected_pairs_full_capture": None,
            "both_detected_pair_rate_full_capture": None,
            "nominal_pairs_science": None, "both_detected_pairs_science": None,
            "both_detected_pair_rate_science": None,
            "science_image_count": None, "science_gt_aligned_count": None,
            "gt_observed_hz": None, "gt_gap_ge_40ms_count": None,
            "tracking_3d_rmse_m_calibration_only": None,
            "tracking_3d_p95_m_calibration_only": None,
            "qc_warnings": None,
        }
        if slot["status"] in ("VALID", "VALID_WITH_WARNINGS"):
            attempt = slot["attempts"][-1]
            directory = root / attempt["recording_path"]
            qc = _read(directory / "qc_report.json")
            if qc["status"] != slot["status"] or not qc["counted_in_39"]:
                raise ValueError(f"status/QC disagreement: {slot['run_id']}")
            reference = _read(directory / "flight_reference.json")
            controller = _read(directory / "controller_result.json")
            observations = _read(directory / "observation_summary.json")
            image_index = _read(directory / "image_index_summary.json")
            state_capture = _read(directory / "state_capture_summary.json")
            alignment = _read(directory / "gt_alignment_summary.json")
            counts = observations["counts"]
            left = counts["left"]
            right = counts["right"]
            pair_count = observations["nominal_pair_count"]
            both = observations["both_detected_pair_count"]
            aligned = alignment["science_image_count_by_alignment_status"]
            events = [json.loads(line) for line in (directory / "events.jsonl").read_text(encoding="utf-8").splitlines()]
            starts = [event for event in events if event["event"] == "science_start"]
            ends = [event for event in events if event["event"] == "science_end"]
            science_completed = controller["controller_status"] == "science_completed"
            if science_completed and (len(starts) != 1 or len(ends) != 1
                                      or starts[0]["simulation_epoch"] != ends[0]["simulation_epoch"]):
                raise ValueError(f"science phase boundaries missing or cross epoch: {slot['run_id']}")
            if not science_completed and (starts or ends):
                raise ValueError(f"partial science boundary requires manual review: {slot['run_id']}")
            science_pairs = science_both = 0
            if science_completed:
                with (directory / "paired_observations.jsonl").open(encoding="utf-8") as handle:
                    for line in handle:
                        pair = json.loads(line)
                        if (pair["simulation_epoch"] == starts[0]["simulation_epoch"]
                                and starts[0]["simulation_timestamp_ns"] <= pair["pair_midpoint_timestamp_ns"]
                                <= ends[0]["simulation_timestamp_ns"]):
                            science_pairs += 1
                            science_both += bool(pair["valid_yolo_pair"])
            row.update({
                "controller_status": controller["controller_status"],
                "controller_failure_reason": controller.get("failure_reason"),
                "science_reference_duration_s": reference["science_duration_s"],
                "science_reference_length_3d_m": reference["length_3d_m"],
                "left_source_frames_full_capture": left["source_frames"],
                "right_source_frames_full_capture": right["source_frames"],
                "left_observed_source_hz": image_index["left"]["observed_source_hz"],
                "right_observed_source_hz": image_index["right"]["observed_source_hz"],
                "left_unmatched_frames": image_index["left"]["unmatched_frame_count"],
                "right_unmatched_frames": image_index["right"]["unmatched_frame_count"],
                "maximum_abs_pair_delta_ns": image_index["maximum_abs_pair_delta_ns"],
                "left_detected_full_capture": left["detected"],
                "right_detected_full_capture": right["detected"],
                "left_detection_rate_full_capture": left["detected"] / left["source_frames"],
                "right_detection_rate_full_capture": right["detected"] / right["source_frames"],
                "nominal_pairs_full_capture": pair_count,
                "both_detected_pairs_full_capture": both,
                "both_detected_pair_rate_full_capture": both / pair_count if pair_count else None,
                "nominal_pairs_science": science_pairs,
                "both_detected_pairs_science": science_both,
                "both_detected_pair_rate_science": science_both / science_pairs if science_pairs else None,
                "science_image_count": alignment["science_image_count_total"],
                "science_gt_aligned_count": sum(aligned.get(key, 0) for key in ("EXACT", "INTERPOLATED")),
                "gt_observed_hz": state_capture["gt_observed_hz"],
                "gt_gap_ge_40ms_count": state_capture["gt_gap_ge_40ms_count"],
                "qc_warnings": ", ".join(qc["warnings"]),
            })
            if slot["path_id"] != "V0" and science_completed:
                if not alignment["tracking_metrics_revealed"]:
                    raise ValueError(f"calibration tracking missing: {slot['run_id']}")
                tracking = alignment["tracking"]
                row["tracking_3d_rmse_m_calibration_only"] = tracking["position_3d_rmse_m"]
                row["tracking_3d_p95_m_calibration_only"] = tracking["position_3d_p95_m"]
            elif slot["path_id"] == "V0" and alignment["tracking_metrics_revealed"]:
                raise ValueError(f"V0 metrics were opened before freeze: {slot['run_id']}")
        runs.append(row)
    by_path = defaultdict(list)
    for row in runs:
        if row["status"] in ("VALID", "VALID_WITH_WARNINGS"):
            by_path[(row["rig_group"], row["path_id"])].append(row)
    paths = []
    for (group, path), rows in sorted(by_path.items()):
        def mean(name):
            values = [item[name] for item in rows if item[name] is not None]
            return sum(values) / len(values) if values else None
        paths.append({
            "rig_group": group, "path_id": path, "valid_run_count": len(rows),
            "science_completed_count": sum(item["controller_status"] == "science_completed" for item in rows),
            "science_pair_count_total": sum(item["nominal_pairs_science"] or 0 for item in rows),
            "science_both_detected_pair_count_total": sum(item["both_detected_pairs_science"] or 0 for item in rows),
            "science_reference_duration_s": mean("science_reference_duration_s"),
            "science_reference_length_3d_m": mean("science_reference_length_3d_m"),
            "both_detected_pair_rate_full_capture_mean": mean("both_detected_pair_rate_full_capture"),
            "both_detected_pair_rate_science_mean": mean("both_detected_pair_rate_science"),
            "both_detected_pairs_science_mean": mean("both_detected_pairs_science"),
            "tracking_3d_p95_m_calibration_only_mean": mean("tracking_3d_p95_m_calibration_only"),
        })
    analysis_handoff = _analysis_handoff(root)
    return {
        "schema_version": "stereo_v1_1_workbook_summary_source_v1",
        "planned_run_count": 39, "valid_run_count": status["valid_run_count"],
        "run_scope": "separate SITL attempts; source capture plus offline YOLO observations",
        "measurement_estimator_status": "implemented_offline_A_B_C_RPY_IMU_YAW_ONLY",
        "v0_positioning_metrics_revealed": analysis_handoff["v0_position_metrics_available_after_freeze"],
        "offline_analysis": analysis_handoff,
        "raw_storage_contract": "one immutable run/attempt directory; separate left/right bag and state JSONL streams",
        "runs": runs, "by_path": paths,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--allow-partial", action="store_true")
    args = parser.parse_args()
    root = args.output_root.resolve()
    data = summarize(root, allow_partial=args.allow_partial)
    target = root / "요약_결과" / "summary_data.json"
    with target.open("x", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"output": str(target), "valid": data["valid_run_count"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
