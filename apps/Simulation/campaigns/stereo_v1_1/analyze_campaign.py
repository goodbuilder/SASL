#!/usr/bin/env python3
"""Run the frozen v1.1 calibration, path-selection, and V0 evaluation pipeline.

The estimator receives only detected image pairs, measured camera RTK/IMU
inputs, the declared yaw initialization, and drone RTK for A/B. GT is read only
inside the synthetic sensor producer and the post-freeze evaluator.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np
import scipy

SIM_ROOT = Path(__file__).resolve().parents[2]
CAMPAIGN_ROOT = Path(__file__).resolve().parent
PROTOCOL_ROOT = CAMPAIGN_ROOT / "protocol"
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.calibration_solver import (
    HUBER_F_SCALE,
    rotation_world_from_optical,
    solve_calibration,
    triangulate_stereo_pixels,
    triangulate_stereo_pairs,
    wrap_angle,
)
from campaigns.stereo_v1_1.camera_frame_contract import optical_rwc_from_entity_quaternion_xyzw
from campaigns.stereo_v1_1.generate_sensor_observations import generate as generate_camera_sensors
from campaigns.stereo_v1_1.generate_sensor_observations import make_sensitivity_observation
from campaigns.stereo_v1_1.generate_drone_rtk_observations import generate_attempt as generate_drone_rtk
from campaigns.stereo_v1_1.bag_index import pair_frames
from campaigns.stereo_v1_1.gt_alignment import GroundTruthSeries
from campaigns.stereo_v1_1.qc_attempt import validate as validate_attempt


METHODS = ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint")
VARIANTS = ("RPY_IMU", "YAW_ONLY")
ALL_DATA = "all_data"
MATCHED_DATA = "matched_200"
PIXEL_SIGMA_PX = 1.0


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _read_jsonl(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_text(payload: object) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True,
                      allow_nan=False) + "\n"


def _write_immutable(path: Path, payload: object) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = _json_text(payload)
    if path.exists():
        if path.read_text(encoding="utf-8") != encoded:
            raise FileExistsError(f"derived result exists with different content; refusing overwrite: {path}")
    else:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _copy_verified(source: Path, target: Path) -> str:
    target.parent.mkdir(parents=True, exist_ok=True)
    source_hash = _sha256(source)
    if target.exists():
        if _sha256(target) != source_hash:
            raise FileExistsError(f"derived source copy differs; refusing overwrite: {target}")
    else:
        with source.open("rb") as source_handle, target.open("xb") as target_handle:
            while chunk := source_handle.read(1024 * 1024):
                target_handle.write(chunk)
            target_handle.flush()
            os.fsync(target_handle.fileno())
        if _sha256(target) != source_hash:
            raise ValueError(f"derived artifact copy hash mismatch: {target}")
    return source_hash


def _verify_local_checksums(case_dir: Path, filename: str) -> None:
    manifest_path = case_dir / filename
    if not manifest_path.is_file():
        raise FileNotFoundError(f"derived checksum manifest missing: {manifest_path}")
    manifest = _read_json(manifest_path)
    for relative, expected in manifest.items():
        path = (case_dir / relative).resolve()
        if not path.is_relative_to(case_dir.resolve()):
            raise ValueError(f"derived checksum path escapes case directory: {relative}")
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"derived artifact hash mismatch: {relative}")


def _method_result_files(case_dir: Path, directory_name: str, rows: list[dict]) -> dict[str, str]:
    hashes = {}
    for result in rows:
        path = case_dir / directory_name / f"{result['method']}__{result['attitude_variant']}.json"
        _write_immutable(path, result)
        hashes[str(path.relative_to(case_dir))] = _sha256(path)
    return hashes


def _write_or_verify_checksums(case_dir: Path, filename: str, hashes: dict[str, str]) -> None:
    path = case_dir / filename
    if path.exists():
        _verify_local_checksums(case_dir, filename)
        saved = _read_json(path)
        if saved != hashes:
            raise ValueError(f"derived checksum manifest differs from current inputs: {path}")
    else:
        _write_immutable(path, hashes)


def _status_slots(root: Path) -> tuple[dict, list[dict]]:
    status = _read_json(root / "run_status.json")
    if status.get("planned_run_count") != 39 or len(status.get("run_slots", [])) != 39:
        raise ValueError("analysis requires the frozen 39-slot campaign status")
    if status.get("valid_run_count") != 39:
        raise ValueError("analysis requires 39 source-QC-valid raw runs")
    slots = status["run_slots"]
    for slot in slots:
        if slot["status"] not in ("VALID", "VALID_WITH_WARNINGS"):
            raise ValueError(f"nonvalid source slot: {slot['run_id']}={slot['status']}")
        record = slot["attempts"][-1]
        attempt_dir = root / record["recording_path"]
        qc = validate_attempt(attempt_dir, slot["run_id"], record["attempt_id"])
        if qc["status"] != slot["status"] or not qc["counted_in_39"]:
            raise ValueError(f"fresh source QC differs from status: {slot['run_id']}")
    return status, slots


def _slot_by_id(slots: list[dict], run_id: str) -> dict:
    matches = [row for row in slots if row["run_id"] == run_id]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one run slot: {run_id}")
    return matches[0]


def _attempt_dir(root: Path, slot: dict) -> Path:
    return root / slot["attempts"][-1]["recording_path"]


def _science_window(attempt_dir: Path, last_image_stamp: int) -> tuple[int | None, int | None, int | None]:
    events = _read_jsonl(attempt_dir / "events.jsonl")
    starts = [row for row in events if row.get("event") == "science_start"]
    ends = [row for row in events if row.get("event") == "science_end"]
    if not starts:
        return None, None, None
    start = starts[0]
    if len(starts) != 1:
        raise ValueError("flight has duplicate science_start events")
    if ends:
        if len(ends) != 1 or ends[0].get("simulation_epoch") != start.get("simulation_epoch"):
            raise ValueError("science phase end is duplicated or crosses simulation epoch")
        return int(start["simulation_timestamp_ns"]), int(ends[0]["simulation_timestamp_ns"]), int(start["simulation_epoch"])
    # Early dynamics/controller termination is retained as a partial science
    # outcome; observations stop at the last source exposure, never extrapolate.
    return int(start["simulation_timestamp_ns"]), int(last_image_stamp), int(start["simulation_epoch"])


def _image_pairs(root: Path, slot: dict) -> tuple[list[dict], dict]:
    """Join source pair IDs with YOLO centers and retain the science interval."""
    attempt_dir = _attempt_dir(root, slot)
    left = {row["frame_id"]: row for row in _read_jsonl(attempt_dir / "observations_left.jsonl")}
    right = {row["frame_id"]: row for row in _read_jsonl(attempt_dir / "observations_right.jsonl")}
    indexed = _read_jsonl(attempt_dir / "image_index.jsonl")
    if not indexed:
        return [], {"science_start_ns": None, "science_end_ns": None,
                    "simulation_epoch": None, "science_window_complete": False,
                    "science_window_status": "NO_SOURCE_IMAGES",
                    "nominal_science_pair_count": 0,
                    "both_detected_science_pair_count": 0,
                    "valid_solver_pair_count": 0, "detection_rate": None}
    last_stamp = max(int(row["image_timestamp_ns"]) for row in indexed)
    start_ns, end_ns, epoch = _science_window(attempt_dir, last_stamp)
    if start_ns is None or end_ns is None or epoch is None:
        return [], {"science_start_ns": None, "science_end_ns": None,
                    "simulation_epoch": None, "science_window_complete": False,
                    "science_window_status": "NO_SCIENCE_START",
                    "nominal_science_pair_count": 0,
                    "both_detected_science_pair_count": 0,
                    "valid_solver_pair_count": 0, "detection_rate": None}
    pairs = []
    nominal_science_pair_count = 0
    detected_science_pair_count = 0
    for pair in _read_jsonl(attempt_dir / "paired_observations.jsonl"):
        stamp = int(pair["pair_midpoint_timestamp_ns"])
        if pair.get("simulation_epoch") != epoch or not start_ns <= stamp <= end_ns:
            continue
        nominal_science_pair_count += 1
        lrow = left.get(pair["left_frame_id"])
        rrow = right.get(pair["right_frame_id"])
        if lrow is None or rrow is None:
            continue
        if lrow.get("detection_status") != "DETECTED" or rrow.get("detection_status") != "DETECTED":
            continue
        left_uv = lrow.get("bbox_center_uv_px")
        right_uv = rrow.get("bbox_center_uv_px")
        if left_uv is None or right_uv is None:
            continue
        detected_science_pair_count += 1
        pairs.append({
            "pair_id": pair["pair_id"],
            "simulation_epoch": epoch,
            "timestamp_ns": stamp,
            "left_detection_status": "DETECTED",
            "right_detection_status": "DETECTED",
            "left_uv_px": left_uv,
            "right_uv_px": right_uv,
            "image_width_px": int(lrow["image_width_px"]),
            "image_height_px": int(lrow["image_height_px"]),
        })
    pairs.sort(key=lambda row: (row["simulation_epoch"], row["timestamp_ns"], row["pair_id"]))
    complete = bool(_read_json(attempt_dir / "controller_result.json").get("controller_status") == "science_completed")
    summary = {
        "science_start_ns": start_ns,
        "science_end_ns": end_ns,
        "simulation_epoch": epoch,
        "science_window_complete": complete,
        "science_window_status": "COMPLETE" if complete else "PARTIAL_CAPTURE",
        "nominal_science_pair_count": nominal_science_pair_count,
        "both_detected_science_pair_count": detected_science_pair_count,
        "valid_solver_pair_count": len(pairs),
        "detection_rate": detected_science_pair_count / nominal_science_pair_count if nominal_science_pair_count else None,
    }
    return pairs, summary


def _select_time_stratified(pairs: list[dict], count: int = 200) -> list[dict]:
    """Select unique pairs from equal time bins; never synthesize frames."""
    if len(pairs) < count:
        return []
    ordered = sorted(pairs, key=lambda row: (row["timestamp_ns"], row["pair_id"]))
    first, last = int(ordered[0]["timestamp_ns"]), int(ordered[-1]["timestamp_ns"])
    if first == last:
        return []
    selected = []
    for bin_index in range(count):
        lower = first + (last - first) * bin_index / count
        upper = first + (last - first) * (bin_index + 1) / count
        center = (lower + upper) / 2.0
        candidates = [row for row in ordered
                      if lower <= int(row["timestamp_ns"]) < upper
                      or (bin_index == count - 1 and int(row["timestamp_ns"]) == last)]
        if not candidates:
            return []
        selected.append(min(candidates, key=lambda row: abs(int(row["timestamp_ns"]) - center)))
    if len({row["pair_id"] for row in selected}) != count:
        return []
    return sorted(selected, key=lambda row: (row["timestamp_ns"], row["pair_id"]))


def _interpolate_rtk(rows: list[dict], pair: dict) -> dict | None:
    candidates = [row for row in rows if int(row["simulation_epoch"]) == int(pair["simulation_epoch"])]
    if not candidates:
        return None
    stamps = [int(row["timestamp_ns"]) for row in candidates]
    target = int(pair["timestamp_ns"])
    import bisect

    index = bisect.bisect_left(stamps, target)
    if index < len(stamps) and stamps[index] == target:
        row = candidates[index]
        return {"position_world_enu_m": row["position_world_enu_m"],
                "covariance_world_enu_m2": row["covariance_world_enu_m2"]}
    if index == 0 or index == len(stamps):
        return None
    before, after = candidates[index - 1], candidates[index]
    gap = int(after["timestamp_ns"]) - int(before["timestamp_ns"])
    if gap <= 0 or gap > 200_000_000:
        return None
    alpha = (target - int(before["timestamp_ns"])) / gap
    position = [(1.0 - alpha) * float(before["position_world_enu_m"][axis])
                + alpha * float(after["position_world_enu_m"][axis]) for axis in range(3)]
    return {"position_world_enu_m": position,
            "covariance_world_enu_m2": before["covariance_world_enu_m2"],
            "before_timestamp_ns": int(before["timestamp_ns"]),
            "after_timestamp_ns": int(after["timestamp_ns"]),
            "interpolation_alpha": alpha}


def _science_rtk_pairs(root: Path, slot: dict, trial: int, image_pairs: list[dict],
                       output_root: Path | None = None) -> tuple[list[dict], list[dict]]:
    attempt = slot["attempts"][-1]
    target_dir = (Path(output_root).resolve() / slot["run_id"] / attempt["attempt_id"]
                  / "sensors" / f"trial_{trial:03d}") if output_root is not None else None
    rtk_path, _provenance_path = generate_drone_rtk(
        root, slot["run_id"], attempt["attempt_id"], trial, output_dir=target_dir,
    )
    rtk_rows = _read_jsonl(rtk_path)
    kept_pairs, paired_rtk = [], []
    for pair in image_pairs:
        measurement = _interpolate_rtk(rtk_rows, pair)
        if measurement is not None:
            kept_pairs.append(pair)
            paired_rtk.append(measurement)
    return kept_pairs, paired_rtk


def _camera_sensor_path(root: Path, slot: dict, trial: int,
                        output_root: Path | None = None) -> Path:
    attempt = slot["attempts"][-1]
    parent = Path(output_root).resolve() if output_root is not None else root / "파생_재처리"
    return parent / slot["run_id"] / attempt["attempt_id"] / "sensors" / f"trial_{trial:03d}" / "sensor_observations.json"


def _attempt_id(slot: dict) -> str:
    return str(slot["attempts"][-1]["attempt_id"])


def _evaluation_run_id(slots: list[dict], group: str, repetition: int) -> str:
    matches = [row for row in slots if row["rig_group"] == group
               and row["path_id"] == "V0" and int(row["repetition"]) == repetition]
    if len(matches) != 1:
        raise ValueError(f"missing unique matching V0 for {group} repetition {repetition}")
    return str(matches[0]["run_id"])


def _yaw_cases(group: str, spec: dict) -> list[tuple[str, dict[str, float]]]:
    if group == "G1":
        # G1's protocol primary initialization is exactly zero on both cameras;
        # its physical ±0.5° headings already create the declared bias.
        return [("G1_PRIMARY_ZERO", {"left": 0.0, "right": 0.0})]
    return [(row["id"], {"left": float(row["bias_left_deg"]),
                         "right": float(row["bias_right_deg"])})
            for row in spec["reprocessing"]["yaw_bias_cases_deg"]]


def _make_trial_config(group: str, yaw_case: str, bias_deg: dict[str, float],
                       spec: dict, trial: int, calibration_run_id: str,
                       evaluation_run_id: str, sample_scope: str) -> dict:
    if group == "G1":
        initial_deg = {"left": 0.0, "right": 0.0}
    else:
        initial_deg = {
            side: float(spec["camera"]["physical_pose_groups"][group][f"heading_{side}_deg"])
            + float(bias_deg[side]) for side in ("left", "right")
        }
    return {
        "schema_version": "stereo_v1_1_estimator_trial_config_v1",
        "yaw_case": yaw_case,
        "initial_yaw_rad": {side: math.radians(initial_deg[side]) for side in ("left", "right")},
        "pixel_sigma_px": PIXEL_SIGMA_PX,
        "mc_seed": trial,
        "trial_index": trial,
        "calibration_run_id": calibration_run_id,
        "evaluation_run_id": evaluation_run_id,
        "sample_scope": sample_scope,
    }


def _analysis_case_dir(analysis_root: Path, slot: dict, trial: int,
                       yaw_case: str, scope: str) -> Path:
    return (analysis_root / "derived" / slot["run_id"] / _attempt_id(slot)
            / f"trial_{trial:03d}" / yaw_case / scope)


def _calibration_input(root: Path, slot: dict, trial: int, cache: dict,
                       generated_roots: dict[str, Path] | None = None) -> dict:
    key = (slot["run_id"], trial)
    if key in cache:
        return cache[key]
    pairs, pair_summary = _image_pairs(root, slot)
    roots = generated_roots or {}
    rtk_pairs, rtk_values = _science_rtk_pairs(
        root, slot, trial, pairs, roots.get("drone_rtk"),
    )
    sensor_path = _camera_sensor_path(root, slot, trial, roots.get("camera"))
    if not sensor_path.is_file():
        raise FileNotFoundError(f"camera sensor observations missing: {sensor_path}")
    sensor = _read_json(sensor_path)
    cache[key] = {
        "pairs": rtk_pairs,
        "rtk": rtk_values,
        "sensor": sensor,
        "sensor_path": sensor_path,
        "pair_summary": pair_summary,
        "detected_pair_count_before_rtk": len(pairs),
        "pair_count_without_rtk": len(pairs) - len(rtk_pairs),
        "rtk_path": (roots.get("drone_rtk", root / "파생_재처리") / slot["run_id"]
                     / _attempt_id(slot) / "sensors" / f"trial_{trial:03d}"
                     / "drone_rtk_observations.jsonl"),
    }
    return cache[key]


def _not_run_result(method: str, variant: str, pair_count: int, reason: str,
                    trial_config: dict) -> dict:
    return {
        "schema_version": "stereo_v1_1_calibration_result_v1",
        "method": method,
        "attitude_variant": variant,
        "camera_dof": 6 if variant == "RPY_IMU" else 2,
        "latent_dof": None,
        "fixed_camera_centers_world_m": None,
        "imu_prior_means_rad": None,
        "imu_prior_covariance_rad2": None,
        "initial_R_WC": None,
        "estimated_R_WC": None,
        "estimated_phi_theta_psi_rad": None,
        "attitude_covariance_rad2": None,
        "conditioning": {"condition_number": None, "rank_deficient": None},
        "convergence_status": "NOT_RUN_INSUFFICIENT_INPUT",
        "failure_reason": reason,
        "observation_count": pair_count,
        "calibration_run_id": trial_config["calibration_run_id"],
        "evaluation_run_id": trial_config["evaluation_run_id"],
        "mc_seed": trial_config["mc_seed"],
    }


def _calibrate_scope(root: Path, slot: dict, trial: int, yaw_case: str,
                     bias_deg: dict[str, float], scope: str, spec: dict,
                     slots: list[dict], input_cache: dict,
                     analysis_root: Path,
                     generated_roots: dict[str, Path] | None = None) -> tuple[dict, list[dict]]:
    data = _calibration_input(root, slot, trial, input_cache, generated_roots)
    evaluation_run = _evaluation_run_id(slots, slot["rig_group"], int(slot["repetition"]))
    pairs = data["pairs"]
    rtk_rows = data["rtk"]
    if scope == MATCHED_DATA:
        selected = _select_time_stratified(pairs, 200)
        if len(selected) != 200:
            selected_pairs, selected_rtk = [], []
        else:
            selected_pairs = selected
            by_id = {row["pair_id"]: rtk for row, rtk in zip(pairs, rtk_rows)}
            selected_rtk = [by_id[row["pair_id"]] for row in selected]
    else:
        selected_pairs, selected_rtk = pairs, rtk_rows

    trial_config = _make_trial_config(
        slot["rig_group"], yaw_case, bias_deg, spec, trial,
        slot["run_id"], evaluation_run, scope,
    )
    case_dir = _analysis_case_dir(analysis_root, slot, trial, yaw_case, scope)
    truth_camera = _camera_truth(_attempt_dir(root, slot))
    ideal_sensor = {
        "schema_version": "stereo_v1_1_yaw_ideal_diagnostic_sensor_observations_v1",
        "synthetic": True,
        "diagnostic_only": True,
        "runtime_sensor_claim": False,
        "estimator_input_allowlist": ["cameras.left", "cameras.right", "shared_sensor_key"],
        "shared_sensor_key": f"YAW_IDEAL:{slot['rig_group']}:R{int(slot['repetition']):02d}:MC{trial:03d}",
        "cameras": {},
    }
    for side in ("left", "right"):
        ideal_sensor["cameras"][side] = {
            "optical_center_rtk_position_world_enu_m": truth_camera[side]["center"].tolist(),
            "optical_center_rtk_covariance_world_enu_m2": np.zeros((3, 3)).tolist(),
            "camera_imu_roll_pitch_mean_optical_rad": [0.0, 0.0],
            "camera_imu_roll_pitch_covariance_optical_rad2": np.zeros((2, 2)).tolist(),
        }
    ideal_sensor_text = _json_text(ideal_sensor)
    ideal_rtk_path = None
    ideal_rtk_rows = []
    if selected_pairs:
        attempt = slot["attempts"][-1]
        ideal_rtk_path, _ideal_rtk_provenance = generate_drone_rtk(
            root, slot["run_id"], attempt["attempt_id"], trial,
            sigma_xyz_m=[0.0, 0.0, 0.0],
            output_dir=case_dir / "inputs_yaw_ideal",
            scenario_id=f"yaw_ideal_{slot['run_id'].replace('-', '_')}_{scope}",
        )
        exact_rtk_rows = _read_jsonl(ideal_rtk_path)
        for pair in selected_pairs:
            exact_measurement = _interpolate_rtk(exact_rtk_rows, pair)
            if exact_measurement is None:
                raise ValueError("ideal YAW_IDEAL RTK alignment lost a selected common image pair")
            ideal_rtk_rows.append(exact_measurement)
    saved_config = {
        **trial_config,
        "method_matrix": list(METHODS),
        "attitude_variants": list(VARIANTS),
        "pair_selection": ("all valid science pairs with bracketed 10 Hz RTK" if scope == ALL_DATA
                           else "200 unique time-stratified pairs; no reuse or duplication"),
        "nominal_science_pairs": data["pair_summary"]["nominal_science_pair_count"],
        "both_detected_science_pairs": data["pair_summary"]["both_detected_science_pair_count"],
        "solver_pairs_before_rtk_alignment": data["detected_pair_count_before_rtk"],
        "pairs_without_rtk_bracket": data["pair_count_without_rtk"],
        "solver_pair_count": len(selected_pairs),
        "matched_200_requirement_met": scope != MATCHED_DATA or len(selected_pairs) == 200,
        "sensor_observations_sha256": _sha256(data["sensor_path"]),
        "yaw_ideal_diagnostic_sensor_observations_sha256": hashlib.sha256(ideal_sensor_text.encode("utf-8")).hexdigest(),
        "yaw_ideal_drone_rtk_sha256": _sha256(ideal_rtk_path) if ideal_rtk_path else None,
        "drone_rtk_observations_sha256": _sha256(data["rtk_path"]),
        "protocol_sha256": _read_json(CAMPAIGN_ROOT / "protocol" / "provenance.json")["files_sha256"],
        "estimator_input_allowlist": [
            "sensor_observations.cameras.left", "sensor_observations.cameras.right",
            "trial_config.initial_yaw_rad", "image_pairs", "drone_rtk_observations for A/B only",
        ],
    }
    _copy_verified(data["sensor_path"], case_dir / "sensor_observations.json")
    _copy_verified(data["sensor_path"].parent / "sensor_generation_provenance.json",
                   case_dir / "sensor_generation_provenance.json")
    _copy_verified(data["rtk_path"], case_dir / "drone_rtk_observations.jsonl")
    main_rtk_provenance = Path(data["rtk_path"]).parent / "drone_rtk_generation_provenance.json"
    _copy_verified(main_rtk_provenance, case_dir / "drone_rtk_generation_provenance.json")
    _write_immutable(case_dir / "sensor_observations_yaw_ideal.json", ideal_sensor)
    if ideal_rtk_path is not None:
        _copy_verified(ideal_rtk_path, case_dir / "drone_rtk_yaw_ideal.jsonl")
        _copy_verified(Path(ideal_rtk_path).parent / "drone_rtk_generation_provenance.json",
                       case_dir / "drone_rtk_yaw_ideal_generation_provenance.json")
    trial_config_path = case_dir / "trial_config.json"
    _write_immutable(trial_config_path, saved_config)

    existing_bundle = case_dir / "calibration_results.json"
    if existing_bundle.is_file():
        payload = _read_json(existing_bundle)
        if (payload.get("source_run_id") != slot["run_id"]
                or payload.get("trial_index") != trial
                or payload.get("yaw_case") != yaw_case
                or payload.get("data_scope") != scope):
            raise ValueError(f"stored calibration result identity differs: {existing_bundle}")
        hashes = {
            "sensor_observations.json": _sha256(case_dir / "sensor_observations.json"),
            "sensor_generation_provenance.json": _sha256(case_dir / "sensor_generation_provenance.json"),
            "sensor_observations_yaw_ideal.json": _sha256(case_dir / "sensor_observations_yaw_ideal.json"),
            "drone_rtk_observations.jsonl": _sha256(case_dir / "drone_rtk_observations.jsonl"),
            "drone_rtk_generation_provenance.json": _sha256(case_dir / "drone_rtk_generation_provenance.json"),
            "trial_config.json": _sha256(trial_config_path),
            "calibration_results.json": _sha256(existing_bundle),
        }
        if (case_dir / "drone_rtk_yaw_ideal.jsonl").is_file():
            hashes["drone_rtk_yaw_ideal.jsonl"] = _sha256(case_dir / "drone_rtk_yaw_ideal.jsonl")
            hashes["drone_rtk_yaw_ideal_generation_provenance.json"] = _sha256(
                case_dir / "drone_rtk_yaw_ideal_generation_provenance.json"
            )
        hashes.update(_method_result_files(case_dir, "calibration_results", payload.get("results", [])))
        _write_or_verify_checksums(case_dir, "calibration_checksums.json", hashes)
        return payload, payload["results"]

    if scope == MATCHED_DATA and len(selected_pairs) != 200:
        reason = f"fewer than 200 RTK-aligned valid pairs: {len(pairs)}"
        result_rows = [_not_run_result(method, variant, len(selected_pairs), reason, trial_config)
                       for method in METHODS for variant in VARIANTS]
    elif len(selected_pairs) < 6:
        reason = f"fewer than six common calibration pairs: {len(selected_pairs)}"
        result_rows = [_not_run_result(method, variant, len(selected_pairs), reason, trial_config)
                       for method in METHODS for variant in VARIANTS]
    else:
        solver_config = {
            key: trial_config[key] for key in (
                "initial_yaw_rad", "pixel_sigma_px", "mc_seed",
                "calibration_run_id", "evaluation_run_id", "yaw_case", "trial_index", "sample_scope",
            )
        }
        result_rows = []
        intrinsics = {
            "fx_px": float(spec["camera"]["fx_px"]),
            "fy_px": float(spec["camera"]["fy_px"]),
            "cx_px": float(spec["camera"]["cx_px"]),
            "cy_px": float(spec["camera"]["cy_px"]),
        }
        for method in METHODS:
            for variant in VARIANTS:
                try:
                    result = solve_calibration(
                        method=method,
                        attitude_variant=variant,
                        sensor_observations=data["sensor"],
                        trial_config=solver_config,
                        image_pairs=selected_pairs,
                        intrinsics=intrinsics,
                        drone_rtk_observations=(selected_rtk if method != "C_no_drone_rtk_joint" else None),
                    )
                except (ValueError, np.linalg.LinAlgError) as exc:
                    result = _not_run_result(method, variant, len(selected_pairs),
                                             f"{type(exc).__name__}: {exc}", solver_config)
                result["data_scope"] = scope
                result["matched_200_requirement_met"] = scope != MATCHED_DATA or len(selected_pairs) == 200
                result_rows.append(result)
        # YAW_IDEAL is a separate truth-assisted diagnostic, never included in
        # the RPY_IMU/YAW_ONLY main comparison. Its input source is explicitly
        # recorded and its true centers/roll/pitch never reach main estimators.
        for method in METHODS:
            try:
                ideal = solve_calibration(
                    method=method,
                    attitude_variant="YAW_ONLY",
                    sensor_observations=ideal_sensor,
                    trial_config=solver_config,
                    image_pairs=selected_pairs,
                    intrinsics=intrinsics,
                    drone_rtk_observations=(ideal_rtk_rows if method != "C_no_drone_rtk_joint" else None),
                )
            except (ValueError, np.linalg.LinAlgError) as exc:
                ideal = _not_run_result(method, "YAW_IDEAL", len(selected_pairs),
                                        f"{type(exc).__name__}: {exc}", trial_config)
            ideal["attitude_variant"] = "YAW_IDEAL"
            ideal["diagnostic_only"] = True
            ideal["diagnostic_input"] = (
                "true Gazebo camera centers and true roll/pitch plus exact synthetic 10 Hz drone RTK; "
                "separate from main noisy comparison"
            )
            ideal["data_scope"] = scope
            ideal["matched_200_requirement_met"] = scope != MATCHED_DATA or len(selected_pairs) == 200
            result_rows.append(ideal)
    payload = {
        "schema_version": "stereo_v1_1_calibration_results_bundle_v1",
        "source_run_id": slot["run_id"],
        "attempt_id": _attempt_id(slot),
        "trial_index": trial,
        "yaw_case": yaw_case,
        "data_scope": scope,
        "results": result_rows,
    }
    _write_immutable(case_dir / "calibration_results.json", payload)
    calibration_hashes = {
        "sensor_observations.json": _sha256(case_dir / "sensor_observations.json"),
        "sensor_generation_provenance.json": _sha256(case_dir / "sensor_generation_provenance.json"),
        "sensor_observations_yaw_ideal.json": _sha256(case_dir / "sensor_observations_yaw_ideal.json"),
        "drone_rtk_observations.jsonl": _sha256(case_dir / "drone_rtk_observations.jsonl"),
        "drone_rtk_generation_provenance.json": _sha256(case_dir / "drone_rtk_generation_provenance.json"),
        "trial_config.json": _sha256(trial_config_path),
        "calibration_results.json": _sha256(case_dir / "calibration_results.json"),
    }
    if (case_dir / "drone_rtk_yaw_ideal.jsonl").is_file():
        calibration_hashes["drone_rtk_yaw_ideal.jsonl"] = _sha256(case_dir / "drone_rtk_yaw_ideal.jsonl")
        calibration_hashes["drone_rtk_yaw_ideal_generation_provenance.json"] = _sha256(
            case_dir / "drone_rtk_yaw_ideal_generation_provenance.json"
        )
    calibration_hashes.update(_method_result_files(case_dir, "calibration_results", result_rows))
    _write_or_verify_checksums(case_dir, "calibration_checksums.json", calibration_hashes)
    return payload, result_rows


def _project_one(point_world: np.ndarray, center_world: np.ndarray,
                 rotation: np.ndarray, intrinsics: dict) -> list[float]:
    camera = rotation.T @ (point_world - center_world)
    if camera[2] <= 1e-6:
        raise ValueError("geometry target projects behind the camera")
    return [float(intrinsics["fx_px"] * camera[0] / camera[2] + intrinsics["cx_px"]),
            float(intrinsics["fy_px"] * camera[1] / camera[2] + intrinsics["cy_px"])]


def _stable_normal(key: str) -> float:
    digest = hashlib.sha256(key.encode("utf-8")).digest()
    first = (int.from_bytes(digest[:8], "big") + 0.5) / 2**64
    second = (int.from_bytes(digest[8:16], "big") + 0.5) / 2**64
    return math.sqrt(-2.0 * math.log(first)) * math.cos(2.0 * math.pi * second)


def _sensitivity_scenarios(spec: dict) -> list[dict]:
    """Build the protocol's staged, one-factor-at-a-time extension matrix."""
    reprocessing = spec["reprocessing"]
    main_camera_sigma = list(spec["calibration_contract"]["camera_centers"]["sigma_xyz_m"])
    main_drone_sigma = [float(value) for value in reprocessing["rtk_sigma_xyz_m"][1]]
    scenarios = []

    def add(scenario_id: str, factor: str, value, **options):
        scenarios.append({"scenario_id": scenario_id, "factor": factor,
                          "factor_value": value, **options})

    position_levels = reprocessing["rtk_sigma_xyz_m"]
    add("camera_rtk_sigma_zero", "camera_rtk_sigma_xyz_m", position_levels[0],
        camera_position_sigma_xyz_m=position_levels[0],
        sensor_note="idealized exact optical-center RTK diagnostic")
    add("camera_rtk_sigma_high", "camera_rtk_sigma_xyz_m", position_levels[2],
        camera_position_sigma_xyz_m=position_levels[2])
    add("camera_imu_sigma_0p05deg", "camera_imu_roll_pitch_sigma_deg",
        float(spec["calibration_contract"]["imu_prior"]["stress_sigma_deg"]),
        camera_imu_sigma_deg=float(spec["calibration_contract"]["imu_prior"]["stress_sigma_deg"]))

    for level_index in (0, 2):
        sigma = [float(value) for value in position_levels[level_index]]
        label = "zero" if level_index == 0 else "high"
        add(f"drone_rtk_sigma_{label}", "drone_rtk_sigma_xyz_m", sigma,
            drone_rtk_sigma_xyz_m=sigma,
            sensor_note="zero sigma means explicitly idealized exact ENU RTK points")

    for magnitude in reprocessing["yaw_extension_abs_deg"]:
        magnitude = float(magnitude)
        token = str(magnitude).replace(".", "p")
        for left_sign, right_sign, suffix in (
            (1, 1, "same_positive"), (-1, -1, "same_negative"),
            (1, -1, "opposed_positive"), (-1, 1, "opposed_negative"),
        ):
            bias = {"left": left_sign * magnitude, "right": right_sign * magnitude}
            add(f"yaw_{token}deg_{suffix}", "initial_yaw_bias_deg", bias,
                yaw_bias_deg=bias)

    for sigma in reprocessing["pixel_iid_sigma_px"]:
        sigma = float(sigma)
        token = str(sigma).replace(".", "p")
        add(f"ideal_pixel_sigma_{token}px", "ideal_projection_pixel_sigma_px", sigma,
            pixel_sigma_px=sigma, input_branch="synthetic_ground_truth_projection")

    for offset_s in reprocessing["timestamp_offsets_s"]:
        offset_s = float(offset_s)
        if offset_s == 0:
            continue
        token = str(abs(offset_s)).replace(".", "p")
        sign = "plus" if offset_s > 0 else "minus"
        offset_ns = int(round(offset_s * 1e9))
        add(f"drone_rtk_time_{sign}_{token}s", "drone_rtk_timestamp_offset_s", offset_s,
            drone_rtk_timestamp_offset_ns=offset_ns)
        add(f"right_camera_time_{sign}_{token}s", "right_camera_timestamp_offset_s", offset_s,
            right_camera_timestamp_offset_ns=offset_ns)

    add("drone_rtk_shared_bias", "drone_rtk_shared_enu_bias", {
        "sigma_xyz_m": main_drone_sigma,
    }, drone_rtk_sigma_xyz_m=main_drone_sigma, drone_rtk_shared_bias=True,
        drone_rtk_shared_bias_sigma_xyz_m=main_drone_sigma)
    for scenario in scenarios:
        scenario.setdefault("camera_position_sigma_xyz_m", main_camera_sigma)
        scenario.setdefault("camera_imu_sigma_deg", float(
            spec["calibration_contract"]["imu_prior"]["main_sigma_deg"]
        ))
        scenario.setdefault("drone_rtk_sigma_xyz_m", main_drone_sigma)
        scenario.setdefault("drone_rtk_shared_bias", False)
        scenario.setdefault("drone_rtk_timestamp_offset_ns", 0)
        scenario.setdefault("right_camera_timestamp_offset_ns", 0)
        scenario.setdefault("input_branch", "actual_yolo_original_pixel_observations")
        scenario.setdefault("yaw_bias_deg", {"left": 0.0, "right": 0.0})
        scenario.setdefault("pixel_sigma_px", PIXEL_SIGMA_PX)
    return scenarios


def _science_interval(attempt_dir: Path) -> tuple[int | None, int | None, int | None]:
    indexed = _read_jsonl(attempt_dir / "image_index.jsonl")
    if not indexed:
        return None, None, None
    return _science_window(attempt_dir, max(int(row["image_timestamp_ns"]) for row in indexed))


def _repaired_camera_pairs(root: Path, slot: dict, offset_ns: int) -> tuple[list[dict], dict, list[dict]]:
    """Shift right-camera reported timestamps and pair unique source frames again."""
    from campaigns.stereo_v1_1.bag_index import PAIR_TOLERANCE_NS

    attempt_dir = _attempt_dir(root, slot)
    start_ns, end_ns, epoch = _science_interval(attempt_dir)
    if start_ns is None or end_ns is None or epoch is None:
        return [], {"nominal_science_pair_count": 0,
                    "both_detected_science_pair_count": 0,
                    "valid_solver_pair_count": 0,
                    "camera_timestamp_offset_ns": offset_ns,
                    "science_window_status": "NO_SCIENCE_START"}, []
    left = _read_jsonl(attempt_dir / "observations_left.jsonl")
    right = _read_jsonl(attempt_dir / "observations_right.jsonl")
    left_by_id = {row["frame_id"]: row for row in left}
    right_by_id = {row["frame_id"]: row for row in right}
    shifted_right = [{**row, "image_timestamp_ns": int(row["image_timestamp_ns"]) + offset_ns}
                     for row in right]
    paired = pair_frames(left, shifted_right, tolerance_ns=PAIR_TOLERANCE_NS)
    source_pairs = [row for row in paired
                    if row["simulation_epoch"] == epoch
                    and start_ns <= int(left_by_id[row["left_frame_id"]]["image_timestamp_ns"]) <= end_ns
                    and start_ns <= int(right_by_id[row["right_frame_id"]]["image_timestamp_ns"]) <= end_ns]
    detected = []
    for row in source_pairs:
        lrow = left_by_id[row["left_frame_id"]]
        rrow = right_by_id[row["right_frame_id"]]
        if lrow.get("detection_status") != "DETECTED" or rrow.get("detection_status") != "DETECTED":
            continue
        if lrow.get("bbox_center_uv_px") is None or rrow.get("bbox_center_uv_px") is None:
            continue
        detected.append({
            "pair_id": row["pair_id"], "simulation_epoch": int(epoch),
            "timestamp_ns": int(row["pair_midpoint_timestamp_ns"]),
            "left_detection_status": "DETECTED", "right_detection_status": "DETECTED",
            "left_uv_px": lrow["bbox_center_uv_px"],
            "right_uv_px": rrow["bbox_center_uv_px"],
            "image_width_px": int(lrow["image_width_px"]),
            "image_height_px": int(lrow["image_height_px"]),
            "left_frame_id": row["left_frame_id"],
            "right_frame_id": row["right_frame_id"],
            "left_image_timestamp_ns": int(row["left_image_timestamp_ns"]),
            "right_image_timestamp_ns_reported": int(row["right_image_timestamp_ns"]),
            "right_timestamp_offset_ns": offset_ns,
            "pair_delta_ns": int(row["pair_delta_ns"]),
        })
    summary = {
        "nominal_science_pair_count": len(source_pairs),
        "both_detected_science_pair_count": len(detected),
        "valid_solver_pair_count": len(detected),
        "detection_rate": len(detected) / len(source_pairs) if source_pairs else None,
        "camera_timestamp_offset_ns": offset_ns,
        "pair_tolerance_ns": PAIR_TOLERANCE_NS,
        "pairing_recomputed_from_unique_source_frames": True,
    }
    return detected, summary, source_pairs


def _ideal_projection_pairs(root: Path, slot: dict, sigma_px: float,
                            trial: int) -> tuple[list[dict], dict, dict]:
    """Generate diagnostic RDF pixels from ENU GT, then add declared iid noise."""
    attempt_dir = _attempt_dir(root, slot)
    start_ns, end_ns, epoch = _science_interval(attempt_dir)
    intrinsics = {key: float(_read_json(PROTOCOL_ROOT / "03_experiment_spec.json")["camera"][key])
                  for key in ("fx_px", "fy_px", "cx_px", "cy_px")}
    camera = _camera_truth(attempt_dir)
    gt = GroundTruthSeries(_read_jsonl(attempt_dir / "ground_truth.jsonl"))
    if start_ns is None or end_ns is None or epoch is None:
        return [], {"nominal_science_pair_count": 0,
                    "both_detected_science_pair_count": 0,
                    "valid_solver_pair_count": 0,
                    "science_window_status": "NO_SCIENCE_START"}, {
                        "status": "NO_SCIENCE_START", "generated_pair_count": 0,
                    }
    raw_pairs = _read_jsonl(attempt_dir / "paired_observations.jsonl")
    science_pairs = [row for row in raw_pairs
                     if int(row["simulation_epoch"]) == epoch
                     and start_ns <= int(row["pair_midpoint_timestamp_ns"]) <= end_ns]
    output, exclusion_counts = [], defaultdict(int)
    for row in science_pairs:
        stamp = int(row["pair_midpoint_timestamp_ns"])
        aligned = gt.align(epoch, stamp)
        if aligned.status not in ("EXACT", "INTERPOLATED") or aligned.position_world_enu_m is None:
            exclusion_counts[f"ground_truth_{aligned.status}"] += 1
            continue
        position = np.asarray(aligned.position_world_enu_m, dtype=float)
        pixels = {}
        visible = True
        for side in ("left", "right"):
            try:
                uv = _project_one(position, camera[side]["center"], camera[side]["rotation"], intrinsics)
            except ValueError:
                visible = False
                exclusion_counts[f"{side}_behind_camera"] += 1
                break
            uv = [uv[axis] + sigma_px * _stable_normal(
                f"ideal-pixel:{slot['run_id']}:MC{trial:03d}:{row['pair_id']}:{side}:{axis}"
            ) for axis in range(2)]
            if not (0.0 <= uv[0] < 1280.0 and 0.0 <= uv[1] < 720.0):
                visible = False
                exclusion_counts[f"{side}_outside_image"] += 1
                break
            pixels[side] = uv
        if not visible:
            continue
        output.append({
            "pair_id": row["pair_id"], "simulation_epoch": epoch,
            "timestamp_ns": stamp, "left_detection_status": "DETECTED",
            "right_detection_status": "DETECTED", "left_uv_px": pixels["left"],
            "right_uv_px": pixels["right"], "image_width_px": 1280,
            "image_height_px": 720, "source_branch": "synthetic_ground_truth_projection",
        })
    summary = {
        "nominal_science_pair_count": len(science_pairs),
        "both_detected_science_pair_count": len(output),
        "valid_solver_pair_count": len(output),
        "detection_rate": len(output) / len(science_pairs) if science_pairs else None,
        "pair_source": "nominal unique source-frame pairs; ignores YOLO detection to create separate ideal branch",
        "pixel_noise_sigma_px": sigma_px,
    }
    provenance = {
        "schema_version": "stereo_v1_1_ideal_projection_provenance_v1",
        "method_name": "SYNTHETIC_GROUND_TRUTH_PROJECTION_PIXEL_STRESS",
        "purpose": "one-factor pixel-noise sensitivity only; never the actual-YOLO branch",
        "input_assumptions": {
            "point_frame": "world_enu:east,north,up_m",
            "camera_frame": "optical_rdf:right,down,forward",
            "rotation_contract": "R_WC maps optical RDF to world ENU",
            "reported_timestamp": "original source pair midpoint; GT is aligned without extrapolation",
            "pixel_noise": "independent SHA256-keyed Gaussian pixel offset per source pair, camera and axis",
            "noise_sigma_px": sigma_px,
        },
        "source_ground_truth_sha256": _sha256(attempt_dir / "ground_truth.jsonl"),
        "source_camera_truth_sha256": _sha256(attempt_dir / "camera_truth.json"),
        "source_pair_sha256": _sha256(attempt_dir / "paired_observations.jsonl"),
        "generated_pair_count": len(output),
        "source_pair_count": len(science_pairs),
        "exclusion_counts": dict(exclusion_counts),
        "estimator_input": False,
    }
    return output, summary, provenance


def _sensitivity_pairs(root: Path, slot: dict, scenario: dict,
                       trial: int) -> tuple[list[dict], dict, dict | None]:
    if scenario["input_branch"] == "synthetic_ground_truth_projection":
        return _ideal_projection_pairs(root, slot, float(scenario["pixel_sigma_px"]), trial)
    offset = int(scenario["right_camera_timestamp_offset_ns"])
    if offset:
        pairs, summary, source_pairs = _repaired_camera_pairs(root, slot, offset)
        provenance = {
            "schema_version": "stereo_v1_1_timestamp_repair_provenance_v1",
            "factor": "right_camera_timestamp_offset_s",
            "offset_ns": offset,
            "pair_tolerance_ns": summary["pair_tolerance_ns"],
            "pairing_recomputed_from_unique_source_frames": True,
            "source_observation_hashes": {
                side: _sha256(_attempt_dir(root, slot) / f"observations_{side}.jsonl")
                for side in ("left", "right")
            },
            "source_pair_candidate_count": len(source_pairs),
            "usable_pair_count": len(pairs),
            "estimator_input": False,
        }
        return pairs, summary, provenance
    pairs, summary = _image_pairs(root, slot)
    return pairs, summary, None


def _sensitivity_rtk_inputs(root: Path, slot: dict, pairs: list[dict],
                            scenario: dict, trial: int,
                            output_dir: Path) -> tuple[list[dict], list[dict], dict]:
    attempt = slot["attempts"][-1]
    path, provenance_path = generate_drone_rtk(
        root, slot["run_id"], attempt["attempt_id"], trial,
        shared_bias=bool(scenario["drone_rtk_shared_bias"]),
        sigma_xyz_m=scenario["drone_rtk_sigma_xyz_m"],
        shared_bias_sigma_xyz_m=scenario.get("drone_rtk_shared_bias_sigma_xyz_m"),
        timestamp_offset_ns=int(scenario["drone_rtk_timestamp_offset_ns"]),
        output_dir=output_dir,
        scenario_id=scenario["scenario_id"],
    )
    rtk_rows = _read_jsonl(path)
    kept_pairs, paired_rtk = [], []
    for pair in pairs:
        measurement = _interpolate_rtk(rtk_rows, pair)
        if measurement is not None:
            kept_pairs.append(pair)
            paired_rtk.append(measurement)
    return kept_pairs, paired_rtk, {
        "observation_path": str(path.relative_to(output_dir.parent)),
        "observation_sha256": _sha256(path),
        "provenance_sha256": _sha256(provenance_path),
        "observation_count": len(rtk_rows),
        "paired_observation_count": len(paired_rtk),
        "frame": "world_enu:east,north,up_m",
        "method_allowlist": ["A_rtk_independent", "B_rtk_joint"],
    }


def _run_sensitivity_calibrations(root: Path, slots: list[dict], spec: dict,
                                  path_selection: dict,
                                  analysis_root: Path) -> dict:
    scenarios = _sensitivity_scenarios(spec)
    path_ids = ["P0"]
    selected_path = path_selection.get("selected_path_id")
    if selected_path and selected_path not in path_ids:
        path_ids.append(selected_path)
    calibration_slots = [
        next(row for row in slots if row["rig_group"] == "G0"
             and row["path_id"] == path_id and int(row["repetition"]) == 1)
        for path_id in path_ids
    ]
    case_index = []
    for scenario in scenarios:
        for slot in calibration_slots:
            trial = 0
            evaluation_run_id = _evaluation_run_id(
                slots, slot["rig_group"], int(slot["repetition"])
            )
            case_dir = (analysis_root / "sensitivities" / scenario["scenario_id"]
                        / slot["run_id"] / f"trial_{trial:03d}")
            case_dir.mkdir(parents=True, exist_ok=True)
            sensor, sensor_sources = make_sensitivity_observation(
                root, slot["run_id"], trial, scenario["scenario_id"],
                position_sigma_xyz_m=scenario["camera_position_sigma_xyz_m"],
                imu_sigma_deg=float(scenario["camera_imu_sigma_deg"]),
            )
            _write_immutable(case_dir / "sensor_observations.json", sensor)
            _write_immutable(case_dir / "sensor_source_provenance.json", {
                "schema_version": "stereo_v1_1_sensitivity_sensor_source_v1",
                "offline_truth_sources": sensor_sources,
                "shared_sensor_key": sensor["shared_sensor_key"],
                "estimator_input": False,
            })
            trial_config = _make_trial_config(
                slot["rig_group"], f"SENSITIVITY_{scenario['scenario_id']}",
                scenario["yaw_bias_deg"], spec, trial, slot["run_id"],
                evaluation_run_id, ALL_DATA,
            )
            trial_config["pixel_sigma_px"] = (
                float(scenario["pixel_sigma_px"])
                if float(scenario["pixel_sigma_px"]) > 0 else 1.0
            )
            _write_immutable(case_dir / "trial_config.json", trial_config)
            input_manifest = {
                "schema_version": "stereo_v1_1_sensitivity_case_v1",
                "scenario": scenario,
                "calibration_run_id": slot["run_id"],
                "evaluation_run_id": evaluation_run_id,
                "trial_index": trial,
                "path_id": slot["path_id"],
                "path_role": ("P0_anchor" if slot["path_id"] == "P0"
                              else "selected_without_v0_outcomes"),
                "sample_scope": ALL_DATA,
                "simulator_runs_added": 0,
                "camera_sensor_shared_key": sensor["shared_sensor_key"],
                "camera_sensor_shared_across_calibration_and_v0": True,
                "pixel_fit_sigma_px": trial_config["pixel_sigma_px"],
                "pixel_sigma_zero_policy": (
                    "noise-free ideal projection; unit residual scale used only as an unweighted LS convention"
                    if scenario["input_branch"] == "synthetic_ground_truth_projection"
                    and float(scenario["pixel_sigma_px"]) == 0 else None
                ),
            }
            _write_immutable(case_dir / "case_manifest.json", input_manifest)

            pairs, pair_summary, branch_provenance = _sensitivity_pairs(
                root, slot, scenario, trial
            )
            if branch_provenance is not None:
                _write_immutable(case_dir / "input_branch_provenance.json", branch_provenance)
            if scenario["input_branch"] == "synthetic_ground_truth_projection":
                _write_text_immutable(
                    case_dir / "ideal_projection_pairs.jsonl",
                    "".join(_json_text(row) for row in pairs),
                )
            elif scenario["right_camera_timestamp_offset_ns"]:
                _write_text_immutable(
                    case_dir / "repaired_camera_pairs.jsonl",
                    "".join(_json_text(row) for row in pairs),
                )

            input_dir = case_dir / "inputs"
            selected_pairs, selected_rtk, rtk_input = _sensitivity_rtk_inputs(
                root, slot, pairs, scenario, trial, input_dir,
            )
            input_manifest["paired_observation_count_before_rtk"] = len(pairs)
            input_manifest["paired_observation_count_after_rtk"] = len(selected_pairs)
            input_manifest["pair_summary"] = pair_summary
            input_manifest["drone_rtk_input"] = rtk_input
            _write_immutable(case_dir / "case_manifest_with_inputs.json", input_manifest)

            existing_bundle = case_dir / "calibration_results.json"
            if existing_bundle.is_file():
                _verify_local_checksums(case_dir, "calibration_checksums.json")
                saved_bundle = _read_json(existing_bundle)
                if (saved_bundle.get("scenario_id") != scenario["scenario_id"]
                        or saved_bundle.get("calibration_run_id") != slot["run_id"]
                        or saved_bundle.get("evaluation_run_id") != evaluation_run_id
                        or saved_bundle.get("trial_index") != trial):
                    raise ValueError(f"stored sensitivity result identity differs: {existing_bundle}")
                saved_results = saved_bundle.get("results", [])
                case_index.append({
                    "scenario_id": scenario["scenario_id"],
                    "factor": scenario["factor"], "factor_value": scenario["factor_value"],
                    "calibration_run_id": slot["run_id"],
                    "evaluation_run_id": evaluation_run_id,
                    "path_id": slot["path_id"], "trial_index": trial,
                    "case_path": str(case_dir.relative_to(analysis_root)),
                    "calibration_results_sha256": _sha256(existing_bundle),
                    "calibration_checksums_sha256": _sha256(case_dir / "calibration_checksums.json"),
                    "pair_count": saved_bundle.get("paired_observation_count_after_rtk"),
                    "convergence_counts": {
                        result["method"] + "__" + result["attitude_variant"]:
                        result["convergence_status"] for result in saved_results
                    },
                })
                continue

            intrinsics = {key: float(spec["camera"][key])
                          for key in ("fx_px", "fy_px", "cx_px", "cy_px")}
            solver_config = {
                key: trial_config[key] for key in (
                    "initial_yaw_rad", "pixel_sigma_px", "mc_seed",
                    "calibration_run_id", "evaluation_run_id", "yaw_case",
                    "trial_index", "sample_scope",
                )
            }
            if len(selected_pairs) < 6:
                reason = f"fewer than six common pairs after declared input alignment: {len(selected_pairs)}"
                results = [_not_run_result(method, variant, len(selected_pairs),
                                           reason, solver_config)
                           for method in METHODS for variant in VARIANTS]
            else:
                results = []
                for method in METHODS:
                    for variant in VARIANTS:
                        try:
                            result = solve_calibration(
                                method=method, attitude_variant=variant,
                                sensor_observations=sensor,
                                trial_config=solver_config,
                                image_pairs=selected_pairs, intrinsics=intrinsics,
                                drone_rtk_observations=(selected_rtk
                                    if method != "C_no_drone_rtk_joint" else None),
                            )
                        except (ValueError, np.linalg.LinAlgError) as exc:
                            result = _not_run_result(
                                method, variant, len(selected_pairs),
                                f"{type(exc).__name__}: {exc}", solver_config,
                            )
                        results.append(result)
            for result in results:
                result["sensitivity_scenario_id"] = scenario["scenario_id"]
                result["sensitivity_factor"] = scenario["factor"]
                result["input_branch"] = scenario["input_branch"]
            bundle = {
                "schema_version": "stereo_v1_1_sensitivity_calibration_results_v1",
                "calibration_run_id": slot["run_id"],
                "evaluation_run_id": evaluation_run_id,
                "trial_index": trial,
                "scenario_id": scenario["scenario_id"],
                "factor": scenario["factor"],
                "factor_value": scenario["factor_value"],
                "data_scope": ALL_DATA,
                "input_branch": scenario["input_branch"],
                "pair_summary": pair_summary,
                "paired_observation_count_after_rtk": len(selected_pairs),
                "results": results,
            }
            _write_immutable(case_dir / "calibration_results.json", bundle)
            hashes = {}
            for path in sorted(case_dir.rglob("*")):
                if (path.is_file() and path.name != "calibration_checksums.json"
                        and "evaluation_results" not in path.parts):
                    hashes[str(path.relative_to(case_dir))] = _sha256(path)
            hashes.update(_method_result_files(case_dir, "calibration_results", results))
            _write_or_verify_checksums(case_dir, "calibration_checksums.json", hashes)
            case_index.append({
                "scenario_id": scenario["scenario_id"],
                "factor": scenario["factor"], "factor_value": scenario["factor_value"],
                "calibration_run_id": slot["run_id"],
                "evaluation_run_id": evaluation_run_id,
                "path_id": slot["path_id"], "trial_index": trial,
                "case_path": str(case_dir.relative_to(analysis_root)),
                "calibration_results_sha256": _sha256(case_dir / "calibration_results.json"),
                "calibration_checksums_sha256": _sha256(case_dir / "calibration_checksums.json"),
                "pair_count": len(selected_pairs),
                "convergence_counts": {
                    result["method"] + "__" + result["attitude_variant"]:
                    result["convergence_status"] for result in results
                },
            })

    manifest = {
        "schema_version": "stereo_v1_1_sensitivity_calibration_manifest_v1",
        "method_name": "staged_one_factor_at_a_time_sensitivity",
        "purpose": "quantify design-level sensitivity without adding Gazebo runs or using V0 outcomes for selection",
        "mc_trials_per_sensitivity_case": 1,
        "sensitivity_trial_index": 0,
        "calibration_paths": path_ids,
        "selected_path_id": selected_path,
        "path_selection_source": "G0 trial-0 calibration covariance only; V0 metrics were not read",
        "simulator_runs_added": 0,
        "full_factorial": False,
        "zero_camera_imu_sigma": "not inverted; represented by separate YAW_IDEAL diagnostic in main matrix",
        "scenarios": scenarios,
        "cases": case_index,
    }
    _write_immutable(analysis_root / "sensitivity_calibration_manifest.json", manifest)
    checksums = {
        row["case_path"] + "/calibration_checksums.json": row["calibration_checksums_sha256"]
        for row in case_index
    }
    _write_immutable(analysis_root / "sensitivity_calibration_checksums.json", checksums)
    return manifest


def _evaluate_sensitivity_cases(root: Path, slots: list[dict], spec: dict,
                                analysis_root: Path, manifest: dict) -> dict:
    scenarios = {row["scenario_id"]: row for row in manifest["scenarios"]}
    metrics_rows = []
    evaluation_case_count = 0
    for case in manifest["cases"]:
        scenario = scenarios[case["scenario_id"]]
        case_dir = analysis_root / case["case_path"]
        calibration_slot = _slot_by_id(slots, case["calibration_run_id"])
        evaluation_slot = _slot_by_id(slots, case["evaluation_run_id"])
        trial = int(case["trial_index"])
        calibration_sensor = _read_json(case_dir / "sensor_observations.json")
        evaluation_sensor, _source = make_sensitivity_observation(
            root, evaluation_slot["run_id"], trial, scenario["scenario_id"],
            position_sigma_xyz_m=scenario["camera_position_sigma_xyz_m"],
            imu_sigma_deg=float(scenario["camera_imu_sigma_deg"]),
        )
        if evaluation_sensor != calibration_sensor:
            raise ValueError("sensitivity camera RTK/IMU realization changed between calibration and matching V0")
        _write_immutable(case_dir / "evaluation_sensor_observations.json", evaluation_sensor)
        eval_pairs, pair_summary, branch_provenance = _sensitivity_pairs(
            root, evaluation_slot, scenario, trial,
        )
        if branch_provenance is not None:
            _write_immutable(case_dir / "evaluation_input_branch_provenance.json", branch_provenance)
        if scenario["input_branch"] == "synthetic_ground_truth_projection":
            _write_text_immutable(
                case_dir / "evaluation_ideal_projection_pairs.jsonl",
                "".join(_json_text(row) for row in eval_pairs),
            )
        elif scenario["right_camera_timestamp_offset_ns"]:
            _write_text_immutable(
                case_dir / "evaluation_repaired_camera_pairs.jsonl",
                "".join(_json_text(row) for row in eval_pairs),
            )
        true_camera = _camera_truth(_attempt_dir(root, evaluation_slot))
        gt_series = GroundTruthSeries(
            _read_jsonl(_attempt_dir(root, evaluation_slot) / "ground_truth.jsonl")
        )
        bundle = _read_json(case_dir / "calibration_results.json")
        baseline_path = _analysis_case_dir(
            analysis_root, calibration_slot, trial, "Y0", ALL_DATA,
        ) / "evaluation_results.json"
        baseline_results = {}
        if baseline_path.is_file():
            baseline_payload = _read_json(baseline_path)
            baseline_results = {
                (row.get("method"), row.get("attitude_variant")): row
                for row in baseline_payload.get("results", [])
            }
        results = []
        for calibration_result in bundle["results"]:
            row = dict(calibration_result)
            row["rig_group"] = evaluation_slot["rig_group"]
            evaluated = _evaluation_record(
                row, evaluation_slot["run_id"], ALL_DATA, pair_summary,
                eval_pairs, true_camera, gt_series, spec,
            )
            evaluated["sensitivity_scenario_id"] = scenario["scenario_id"]
            evaluated["sensitivity_factor"] = scenario["factor"]
            evaluated["factor_value"] = scenario["factor_value"]
            evaluated["input_branch"] = scenario["input_branch"]
            baseline = baseline_results.get((evaluated["method"], evaluated["attitude_variant"]))
            baseline_metrics = (baseline or {}).get("metrics") or {}
            current_metrics = evaluated.get("metrics") or {}
            baseline_rmse = baseline_metrics.get("position_3d_rmse_m")
            current_rmse = current_metrics.get("position_3d_rmse_m")
            baseline_count = baseline_metrics.get("valid_position_estimate_count")
            current_count = current_metrics.get("valid_position_estimate_count")
            comparable_branch = scenario["input_branch"] == "actual_yolo_original_pixel_observations"
            delta_rmse = None
            direction = "NOT_COMPARABLE"
            if not comparable_branch:
                comparison_status = "DIFFERENT_PIXEL_BRANCH"
            elif baseline_rmse is None or current_rmse is None:
                comparison_status = "BASELINE_OR_SCENARIO_METRIC_UNAVAILABLE"
                direction = "METRIC_UNAVAILABLE"
            elif baseline_count != current_count:
                comparison_status = "VALID_PAIR_COUNT_DIFFERENT"
                delta_rmse = float(current_rmse) - float(baseline_rmse)
                direction = "DESCRIPTIVE_DELTA_DIFFERENT_SAMPLE_COUNT"
            else:
                comparison_status = "COMPARABLE_SAME_VALID_PAIR_COUNT"
                delta_rmse = float(current_rmse) - float(baseline_rmse)
                direction = ("IMPROVED" if delta_rmse < -1e-12 else
                             "REGRESSED" if delta_rmse > 1e-12 else "NO_CHANGE")
            evaluated["main_y0_baseline"] = {
                "reference": "same G0 run, Y0, all_data, actual-YOLO branch",
                "calibration_status": (baseline or {}).get("calibration_status"),
                "position_3d_rmse_m": baseline_rmse,
                "valid_position_estimate_count": baseline_count,
            }
            evaluated["comparison_to_main_y0"] = {
                "status": comparison_status,
                "position_3d_rmse_delta_m": delta_rmse,
                "direction": direction,
            }
            results.append(evaluated)
            metrics = evaluated.get("metrics") or {}
            metrics_rows.append({
                "scenario_id": scenario["scenario_id"],
                "factor": scenario["factor"], "factor_value": json.dumps(scenario["factor_value"], sort_keys=True),
                "path_id": case["path_id"], "calibration_run_id": case["calibration_run_id"],
                "evaluation_run_id": case["evaluation_run_id"],
                "method": evaluated["method"], "attitude_variant": evaluated["attitude_variant"],
                "calibration_status": evaluated["calibration_status"],
                "evaluation_status": evaluated["evaluation_status"],
                "input_branch": scenario["input_branch"],
                "pair_count": metrics.get("valid_position_estimate_count"),
                "baseline_position_3d_rmse_m": baseline_rmse,
                "position_3d_rmse_delta_m_vs_main_y0": delta_rmse,
                "comparison_status_vs_main_y0": comparison_status,
                "direction_vs_main_y0": direction,
                "detection_rate": metrics.get("detection_rate"),
                "valid_measurement_rate": metrics.get("valid_measurement_rate"),
                "position_3d_rmse_m": metrics.get("position_3d_rmse_m"),
                "position_3d_p95_m": metrics.get("position_3d_p95_m"),
            })
        payload = {
            "schema_version": "stereo_v1_1_sensitivity_evaluation_results_v1",
            "scenario_id": scenario["scenario_id"],
            "factor": scenario["factor"], "factor_value": scenario["factor_value"],
            "calibration_run_id": case["calibration_run_id"],
            "evaluation_run_id": case["evaluation_run_id"],
            "trial_index": trial, "data_scope": ALL_DATA,
            "input_branch": scenario["input_branch"],
            "freeze_sha256": _sha256(analysis_root / "analysis_freeze.json"),
            "v0_recalibration_performed": False,
            "camera_sensor_shared_with_calibration": True,
            "camera_sensor_shared_key": evaluation_sensor["shared_sensor_key"],
            "pair_summary": pair_summary,
            "results": results,
        }
        _write_immutable(case_dir / "evaluation_results.json", payload)
        evaluation_hashes = {
            "evaluation_sensor_observations.json": _sha256(case_dir / "evaluation_sensor_observations.json"),
            "evaluation_results.json": _sha256(case_dir / "evaluation_results.json"),
        }
        evaluation_hashes.update(_method_result_files(case_dir, "evaluation_results", results))
        _write_or_verify_checksums(case_dir, "evaluation_checksums.json", evaluation_hashes)
        evaluation_case_count += 1

    _write_csv_immutable(
        analysis_root / "sensitivity_evaluation_metrics.csv",
        ["scenario_id", "factor", "factor_value", "path_id", "calibration_run_id",
         "evaluation_run_id", "method", "attitude_variant", "calibration_status",
         "evaluation_status", "input_branch", "pair_count",
         "baseline_position_3d_rmse_m", "position_3d_rmse_delta_m_vs_main_y0",
         "comparison_status_vs_main_y0", "direction_vs_main_y0", "detection_rate",
         "valid_measurement_rate", "position_3d_rmse_m", "position_3d_p95_m"],
        metrics_rows,
    )
    summary_groups: dict[tuple, list[dict]] = defaultdict(list)
    for row in metrics_rows:
        summary_groups[(row["scenario_id"], row["method"], row["attitude_variant"])].append(row)
    summary_rows = []
    for (scenario_id, method, variant), rows in sorted(summary_groups.items()):
        rmse = [float(row["position_3d_rmse_m"]) for row in rows
                if row["position_3d_rmse_m"] not in (None, "")]
        p95 = [float(row["position_3d_p95_m"]) for row in rows
               if row["position_3d_p95_m"] not in (None, "")]
        deltas = [float(row["position_3d_rmse_delta_m_vs_main_y0"]) for row in rows
                  if row["position_3d_rmse_delta_m_vs_main_y0"] is not None
                  and row["comparison_status_vs_main_y0"] == "COMPARABLE_SAME_VALID_PAIR_COUNT"]
        summary_rows.append({
            "scenario_id": scenario_id, "factor": rows[0]["factor"],
            "factor_value": rows[0]["factor_value"], "method": method,
            "attitude_variant": variant, "path_count": len(rows),
            "position_3d_rmse_mean_m": statistics.mean(rmse) if rmse else None,
            "position_3d_p95_mean_m": statistics.mean(p95) if p95 else None,
            "position_3d_rmse_delta_mean_m_vs_main_y0": statistics.mean(deltas) if deltas else None,
            "same_sample_improved_count": sum(row["direction_vs_main_y0"] == "IMPROVED" for row in rows),
            "same_sample_regressed_count": sum(row["direction_vs_main_y0"] == "REGRESSED" for row in rows),
            "same_sample_no_change_count": sum(row["direction_vs_main_y0"] == "NO_CHANGE" for row in rows),
            "incomparable_count": sum(row["direction_vs_main_y0"] not in
                                       ("IMPROVED", "REGRESSED", "NO_CHANGE") for row in rows),
            "calibration_statuses": sorted({row["calibration_status"] for row in rows}),
            "evaluation_statuses": sorted({row["evaluation_status"] for row in rows}),
        })
    summary = {
        "schema_version": "stereo_v1_1_sensitivity_summary_v1",
        "method_name": manifest["method_name"],
        "case_count": evaluation_case_count,
        "trial_count_per_case": manifest["mc_trials_per_sensitivity_case"],
        "simulator_runs_added": 0,
        "main_y0_comparison": "same G0 run/path and matched V0; directional deltas are called comparable only when the pixel branch and valid pair count match",
        "same_sample_improved_count": sum(row["same_sample_improved_count"] for row in summary_rows),
        "same_sample_regressed_count": sum(row["same_sample_regressed_count"] for row in summary_rows),
        "incomparable_count": sum(row["incomparable_count"] for row in summary_rows),
        "summary_rows": summary_rows,
        "limitations": [
            "Sensitivity cases use one deterministic reprocessing realization each; the main representative Monte Carlo carries the configured trial count.",
            "Synthetic camera RTK/IMU, drone RTK, and ideal-projection pixel noise are design-level models, not hardware validation.",
            "Right-camera timestamp offsets are re-paired at the protocol 1 ms tolerance and can yield no usable stereo pairs.",
        ],
    }
    _write_immutable(analysis_root / "sensitivity_summary.json", summary)
    return summary


def _position_uncertainty_for_target(result: dict, sensor: dict, target: np.ndarray,
                                     intrinsics: dict) -> dict | None:
    covariance_data = result.get("attitude_covariance_rad2")
    if not isinstance(covariance_data, list) or len(covariance_data) != 6:
        return None
    if any(value is None for row in covariance_data for value in row):
        return None
    pose_cov = np.asarray(covariance_data, dtype=float)
    if not np.all(np.isfinite(pose_cov)):
        return None
    pose_cov = (pose_cov + pose_cov.T) / 2.0
    angles = {side: np.asarray(result["estimated_phi_theta_psi_rad"][side], dtype=float)
              for side in ("left", "right")}
    centers = {side: np.asarray(result["fixed_camera_centers_world_m"][side], dtype=float)
               for side in ("left", "right")}
    rotations = {side: np.asarray(result["estimated_R_WC"][side], dtype=float)
                 for side in ("left", "right")}
    pixels = {side: _project_one(target, centers[side], rotations[side], intrinsics)
              for side in ("left", "right")}

    def reconstruct(current_centers, current_rotations):
        return triangulate_stereo_pixels(
            pixels["left"], pixels["right"],
            camera_centers_world_enu_m=current_centers,
            rotations_world_from_optical=current_rotations,
            intrinsics=intrinsics,
            pixel_sigma_px=PIXEL_SIGMA_PX,
        )

    base = reconstruct(centers, rotations)
    epsilon = 1e-5
    pose_jac = np.zeros((3, 6), dtype=float)
    for camera_index, side in enumerate(("left", "right")):
        for axis in range(3):
            plus_angles = {name: values.copy() for name, values in angles.items()}
            minus_angles = {name: values.copy() for name, values in angles.items()}
            plus_angles[side][axis] += epsilon
            minus_angles[side][axis] -= epsilon
            plus_rotations = {name: rotation_world_from_optical(*values)
                              for name, values in plus_angles.items()}
            minus_rotations = {name: rotation_world_from_optical(*values)
                               for name, values in minus_angles.items()}
            pose_jac[:, camera_index * 3 + axis] = (
                reconstruct(centers, plus_rotations) - reconstruct(centers, minus_rotations)
            ) / (2.0 * epsilon)

    center_covariance = np.zeros((6, 6), dtype=float)
    for camera_index, side in enumerate(("left", "right")):
        cov = np.asarray(sensor["cameras"][side]["optical_center_rtk_covariance_world_enu_m2"], dtype=float)
        center_covariance[camera_index * 3:camera_index * 3 + 3,
                          camera_index * 3:camera_index * 3 + 3] = cov
    center_jac = np.zeros((3, 6), dtype=float)
    for camera_index, side in enumerate(("left", "right")):
        for axis in range(3):
            plus_centers = {name: values.copy() for name, values in centers.items()}
            minus_centers = {name: values.copy() for name, values in centers.items()}
            plus_centers[side][axis] += epsilon
            minus_centers[side][axis] -= epsilon
            center_jac[:, camera_index * 3 + axis] = (
                reconstruct(plus_centers, rotations) - reconstruct(minus_centers, rotations)
            ) / (2.0 * epsilon)

    propagated = pose_jac @ pose_cov @ pose_jac.T + center_jac @ center_covariance @ center_jac.T
    propagated = (propagated + propagated.T) / 2.0
    eigenvalues = np.linalg.eigvalsh(propagated)
    if not np.all(np.isfinite(eigenvalues)):
        return None
    sigma_3d = math.sqrt(max(0.0, float(np.trace(propagated))))
    return {
        "target_world_enu_m": target.tolist(),
        "position_covariance_world_enu_m2": propagated.tolist(),
        "predicted_3d_sigma_m": sigma_3d,
        "baseline_reconstruction_error_m": float(np.linalg.norm(base - target)),
    }


def _select_calibration_path(root: Path, slots: list[dict], path_calibrations: dict,
                             camera_sensor_paths: dict, spec: dict) -> dict:
    intrinsics = {key: float(spec["camera"][key]) for key in ("fx_px", "fy_px", "cx_px", "cy_px")}
    distances = (600.0, 550.0, 500.0, 450.0, 400.0)
    targets = [np.array([0.0, distance,
                         distance * math.tan(math.radians(float(spec["scope"]["current_glide_slope_deg"])))])
               for distance in distances]
    rows = []
    for (run_id, yaw_case, scope), results in sorted(path_calibrations.items()):
        if yaw_case != "Y0" or scope != ALL_DATA:
            continue
        slot = _slot_by_id(slots, run_id)
        if slot["rig_group"] != "G0" or slot["path_id"] == "V0":
            continue
        sensor = _read_json(camera_sensor_paths[(run_id, 0)])
        for result in results:
            if result["method"] not in METHODS or result["attitude_variant"] != "RPY_IMU":
                continue
            if result.get("convergence_status") != "CONVERGED":
                continue
            estimates = [_position_uncertainty_for_target(result, sensor, target, intrinsics)
                         for target in targets]
            estimates = [item for item in estimates if item is not None]
            if not estimates:
                continue
            sigmas = [item["predicted_3d_sigma_m"] for item in estimates]
            reference = _read_json(_attempt_dir(root, slot) / "flight_reference.json")
            rows.append({
                "run_id": run_id,
                "rig_group": slot["rig_group"],
                "path_id": slot["path_id"],
                "repetition": int(slot["repetition"]),
                "method": result["method"],
                "attitude_variant": result["attitude_variant"],
                "predicted_position_uncertainty_mean_sigma_m": statistics.mean(sigmas),
                "predicted_position_uncertainty_p95_sigma_m": sorted(sigmas)[int(0.95 * (len(sigmas) - 1))],
                "predictions_by_target": estimates,
                "reference_duration_s": reference["science_duration_s"],
            })
    by_path: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        by_path[row["path_id"]].append(row)
    candidates = []
    for path_id, values in sorted(by_path.items()):
        sigmas = [row["predicted_position_uncertainty_mean_sigma_m"] for row in values]
        durations = [row["reference_duration_s"] for row in values]
        candidates.append({
            "path_id": path_id,
            "calibration_run_count": len({row["run_id"] for row in values}),
            "predicted_position_uncertainty_mean_sigma_m": statistics.median(sigmas),
            "predicted_position_uncertainty_p95_sigma_m": statistics.median(
                row["predicted_position_uncertainty_p95_sigma_m"] for row in values
            ),
            "reference_duration_mean_s": statistics.mean(durations),
            "methods": sorted({row["method"] for row in values}),
        })
    candidates.sort(key=lambda row: (row["predicted_position_uncertainty_mean_sigma_m"],
                                     row["reference_duration_mean_s"], row["path_id"]))
    selected = candidates[0]["path_id"] if candidates else None
    return {
        "schema_version": "stereo_v1_1_calibration_path_selection_v1",
        "status": "GEOMETRY_SELECTION_AVAILABLE" if selected else "NO_VALID_COVARIANCE_PREDICTIONS",
        "selected_path_id": selected,
        "selection_rule": "lowest median 3-D position sigma propagated from G0 trial-0 RPY_IMU attitude covariance to V0 geometry targets; break ties by shorter 3-D reference duration, then path ID",
        "selection_scope": "G0 calibration runs only; V0 observations and location errors were not read",
        "target_geometry": {
            "frame": "world_enu:east,north,up_m",
            "distances_m": list(distances),
            "vertical_profile": "D*tan(20deg)",
            "origin": "landing pad reference",
        },
        "candidate_summary": candidates,
        "per_calibration_prediction": rows,
        "limitations": [
            "This ranks the specified five candidate paths by local covariance propagation; it does not prove global path optimality.",
            "The protocol has no numeric allowable position-error threshold, so the output is an accuracy/time tradeoff.",
        ],
    }


def _analysis_hashes() -> dict[str, str]:
    names = ("calibration_solver.py", "analyze_campaign.py",
             "generate_sensor_observations.py", "generate_drone_rtk_observations.py",
             "trajectory_contract.py", "gt_alignment.py", "bag_index.py",
             "camera_frame_contract.py")
    return {name: _sha256(CAMPAIGN_ROOT / name) for name in names}


def _analysis_dependencies() -> dict[str, str]:
    return {"numpy": np.__version__, "scipy": scipy.__version__}


def _freeze_analysis(analysis_root: Path, path_selection: dict, mc_trials: int,
                     protocol: dict, sensitivity_manifest: dict) -> dict:
    calibration_checksum_files = sorted(
        (analysis_root / "derived").rglob("calibration_checksums.json")
    )
    if not calibration_checksum_files:
        raise RuntimeError("primary calibration checksums are missing; analysis cannot freeze")
    calibration_manifest = {
        str(path.relative_to(analysis_root)): _sha256(path)
        for path in calibration_checksum_files
    }
    _write_immutable(analysis_root / "calibration_freeze_manifest.json", calibration_manifest)
    freeze = {
        "schema_version": "stereo_v1_1_analysis_freeze_v1",
        "status": "FROZEN_BEFORE_V0_POSITION_EVALUATION",
        "algorithm_sha256": _analysis_hashes(),
        "dependencies": _analysis_dependencies(),
        "protocol_sha256": _read_json(PROTOCOL_ROOT / "provenance.json")["files_sha256"],
        "path_selection_sha256": hashlib.sha256(_json_text(path_selection).encode("utf-8")).hexdigest(),
        "primary_calibration_checksums_manifest_sha256": _sha256(
            analysis_root / "calibration_freeze_manifest.json"
        ),
        "sensitivity_calibration_manifest_sha256": _sha256(
            analysis_root / "sensitivity_calibration_manifest.json"
        ),
        "sensitivity_calibration_checksums_sha256": _sha256(
            analysis_root / "sensitivity_calibration_checksums.json"
        ),
        "sensitivity_case_count": len(sensitivity_manifest["cases"]),
        "selected_path_id": path_selection["selected_path_id"],
        "mc_trial_count": mc_trials,
        "pixel_sigma_px": PIXEL_SIGMA_PX,
        "huber_f_scale": HUBER_F_SCALE,
        "robust_loss": {"name": "Huber", "f_scale_standardized_residual": HUBER_F_SCALE},
        "evaluation_policy": "V0 is evaluated once with frozen G0 calibration poses; V0 observations never re-calibrate camera pose.",
        "v0_position_error_read_before_freeze": False,
        "numeric_pass_threshold": protocol["split_policy"]["selection_target_error_m"],
    }
    path_selection_path = analysis_root / "path_selection.json"
    _write_immutable(path_selection_path, path_selection)
    freeze_path = analysis_root / "analysis_freeze.json"
    _write_immutable(freeze_path, freeze)
    return freeze


def _quaternion_matrix_xyzw(values: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("camera truth quaternion must be finite and nonzero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _camera_truth(attempt_dir: Path) -> dict:
    payload = _read_json(attempt_dir / "camera_truth.json")["camera_payloads"]
    result = {}
    for side in ("left", "right"):
        camera = payload[side]
        result[side] = {
            "center": np.asarray(camera["observed_optical_position_world_enu_m"], dtype=float),
            "rotation": optical_rwc_from_entity_quaternion_xyzw(
                camera["observed_optical_orientation_world_from_entity_xyzw"]
            ),
            "position_frame": "gazebo_world_enu:east,north,up_m",
            "orientation_frame": "optical_rdf:right,down,forward_to_world_enu",
            "source_orientation_frame": "gazebo_camera_link_flu:x_forward,y_left,z_up_to_world_enu",
        }
    return result


def _orientation_error_deg(estimated: np.ndarray, truth: np.ndarray) -> float:
    cosine = (float(np.trace(truth.T @ estimated)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = fraction * (len(ordered) - 1)
    lower, upper = int(math.floor(position)), int(math.ceil(position))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _evaluate_pose(pairs: list[dict], pair_summary: dict, centers: dict,
                   rotations: dict, intrinsics: dict, truth_series: GroundTruthSeries) -> dict:
    aligned_pairs = []
    aligned_truth = []
    alignment_counts: dict[str, int] = defaultdict(int)
    for pair in pairs:
        aligned = truth_series.align(pair["simulation_epoch"], pair["timestamp_ns"])
        alignment_counts[aligned.status] += 1
        if aligned.status not in ("EXACT", "INTERPOLATED") or aligned.position_world_enu_m is None:
            continue
        aligned_pairs.append(pair)
        aligned_truth.append(np.asarray(aligned.position_world_enu_m, dtype=float))
    reconstruction_failures = 0
    errors = []
    if aligned_pairs:
        left_pixels = np.asarray([pair["left_uv_px"] for pair in aligned_pairs], dtype=float)
        right_pixels = np.asarray([pair["right_uv_px"] for pair in aligned_pairs], dtype=float)
        try:
            estimates = triangulate_stereo_pairs(
                left_pixels, right_pixels,
                camera_centers_world_enu_m=centers,
                rotations_world_from_optical=rotations,
                intrinsics=intrinsics,
                pixel_sigma_px=PIXEL_SIGMA_PX,
            )
            valid = np.all(np.isfinite(estimates), axis=1)
            for side in ("left", "right"):
                camera = (estimates - np.asarray(centers[side], dtype=float)) @ np.asarray(rotations[side], dtype=float)
                valid &= camera[:, 2] > 0
            reconstruction_failures = int(np.count_nonzero(~valid))
            if np.any(valid):
                errors = (estimates[valid] - np.asarray(aligned_truth, dtype=float)).tolist()
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            reconstruction_failures = len(aligned_pairs)
    if errors:
        matrix = np.asarray(errors)
        magnitudes = np.linalg.norm(matrix, axis=1)
        axis_names = {"lateral_east": 0, "forward_north": 1, "vertical_up": 2}
        axis_rmse = {name: float(np.sqrt(np.mean(matrix[:, index] ** 2)))
                     for name, index in axis_names.items()}
        axis_bias = {name: float(np.mean(matrix[:, index]))
                     for name, index in axis_names.items()}
        rmse = float(np.sqrt(np.mean(np.sum(matrix * matrix, axis=1))))
        p95 = _percentile(magnitudes.tolist(), 0.95)
    else:
        axis_rmse = {"lateral_east": None, "forward_north": None, "vertical_up": None}
        axis_bias = {"lateral_east": None, "forward_north": None, "vertical_up": None}
        rmse = p95 = None
    nominal = int(pair_summary["nominal_science_pair_count"])
    detected = int(pair_summary["both_detected_science_pair_count"])
    valid = len(errors)
    return {
        "metric_method": "bbox-center stereo triangulation with fixed calibrated R_WC and camera ENU centers",
        "truth_source": "world ENU vehicle_reference_link at pair midpoint; evaluator only",
        "position_error_definition": "estimated minus truth; lateral=world east, forward=world north, vertical=world up",
        "nominal_science_pair_count": nominal,
        "both_detected_pair_count": detected,
        "ground_truth_aligned_pair_count": sum(alignment_counts.get(key, 0) for key in ("EXACT", "INTERPOLATED")),
        "valid_position_estimate_count": valid,
        "reconstruction_failure_count": reconstruction_failures,
        "ground_truth_alignment_status_counts": dict(alignment_counts),
        "detection_rate": detected / nominal if nominal else None,
        "valid_measurement_rate": valid / nominal if nominal else None,
        "axis_bias_m": axis_bias,
        "axis_rmse_m": axis_rmse,
        "position_3d_rmse_m": rmse,
        "position_3d_p95_m": p95,
        "metrics_scope": "final position errors were computed only after analysis_freeze.json was written",
    }


def _attitude_errors(result: dict, true_camera: dict,
                     true_heading_deg: dict[str, float]) -> dict:
    errors = {}
    for side in ("left", "right"):
        estimated_angles = result.get("estimated_phi_theta_psi_rad", {}).get(side)
        estimated_rotation = result.get("estimated_R_WC", {}).get(side)
        if estimated_angles is None or estimated_rotation is None:
            errors[side] = None
            continue
        truth_angles = [0.0, 0.0, math.radians(float(true_heading_deg[side]))]
        errors[side] = {
            "roll_error_rad": wrap_angle(float(estimated_angles[0]) - truth_angles[0]),
            "pitch_correction_error_rad": float(estimated_angles[1]) - truth_angles[1],
            "yaw_error_rad": wrap_angle(float(estimated_angles[2]) - truth_angles[2]),
            "rotation_geodesic_error_deg": _orientation_error_deg(
                np.asarray(estimated_rotation, dtype=float), true_camera[side]["rotation"]
            ),
        }
    relative = None
    if errors["left"] is not None and errors["right"] is not None:
        relative = wrap_angle(errors["right"]["yaw_error_rad"] - errors["left"]["yaw_error_rad"])
    return {"per_camera": errors, "relative_yaw_error_rad": relative}


def _initial_pose(sensor: dict, trial_config: dict) -> tuple[dict, dict]:
    centers, rotations = {}, {}
    for side in ("left", "right"):
        camera = sensor["cameras"][side]
        centers[side] = camera["optical_center_rtk_position_world_enu_m"]
        angles = [*camera["camera_imu_roll_pitch_mean_optical_rad"],
                  trial_config["initial_yaw_rad"][side]]
        rotations[side] = rotation_world_from_optical(*angles)
    return centers, rotations


def _evaluation_record(result: dict, eval_run_id: str, scope: str,
                       pair_summary: dict, pairs: list[dict], true_camera: dict,
                       truth_series: GroundTruthSeries, spec: dict) -> dict:
    true_heading = {
        side: float(spec["camera"]["physical_pose_groups"][result.get("rig_group", "G0")][f"heading_{side}_deg"])
        for side in ("left", "right")
    }
    output = {
        "method": result["method"],
        "attitude_variant": result["attitude_variant"],
        "calibration_run_id": result.get("calibration_run_id"),
        "evaluation_run_id": eval_run_id,
        "mc_seed": result.get("mc_seed"),
        "data_scope": scope,
        "calibration_status": result.get("convergence_status"),
        "calibration_camera_dof": result.get("camera_dof"),
        "calibration_latent_dof": result.get("latent_dof"),
        "attitude_errors": _attitude_errors(result, true_camera, true_heading)
            if (result.get("estimated_R_WC") is not None
                and result.get("estimated_phi_theta_psi_rad") is not None) else None,
        "diagnostic_only": bool(result.get("diagnostic_only", False)),
    }
    if result.get("estimated_R_WC") is None or result.get("fixed_camera_centers_world_m") is None:
        output.update({"evaluation_status": "NOT_EVALUATED_CALIBRATION_FAILED", "metrics": None})
        return output
    rotations = {side: np.asarray(result["estimated_R_WC"][side], dtype=float)
                 for side in ("left", "right")}
    metrics = _evaluate_pose(pairs, pair_summary,
                             result["fixed_camera_centers_world_m"], rotations,
                             {key: float(spec["camera"][key]) for key in ("fx_px", "fy_px", "cx_px", "cy_px")},
                             truth_series)
    output.update({"evaluation_status": "EVALUATED" if metrics["valid_position_estimate_count"] else "NO_VALID_POSITION_ESTIMATES",
                   "metrics": metrics})
    return output


def _v0_evaluation_inputs(root: Path, slot: dict, trial: int, scope: str,
                          cache: dict,
                          generated_roots: dict[str, Path] | None = None) -> dict:
    key = (slot["run_id"], trial, scope)
    if key in cache:
        return cache[key]
    attempt_dir = _attempt_dir(root, slot)
    all_pairs, pair_summary = _image_pairs(root, slot)
    if scope == MATCHED_DATA:
        pairs = _select_time_stratified(all_pairs, 200)
        if len(pairs) == 200:
            pair_summary = dict(pair_summary)
            pair_summary["nominal_science_pair_count"] = 200
            pair_summary["both_detected_science_pair_count"] = 200
    else:
        pairs = all_pairs
    gt_series = GroundTruthSeries(_read_jsonl(attempt_dir / "ground_truth.jsonl"))
    sensor_path = _camera_sensor_path(
        root, slot, trial, (generated_roots or {}).get("camera"),
    )
    sensor = _read_json(sensor_path)
    truth = _camera_truth(attempt_dir)
    cache[key] = {
        "pairs": pairs,
        "all_pairs": all_pairs,
        "pair_summary": pair_summary,
        "matched_200_met": scope != MATCHED_DATA or len(pairs) == 200,
        "gt_series": gt_series,
        "sensor": sensor,
        "sensor_path": sensor_path,
        "true_camera": truth,
        "attempt_dir": attempt_dir,
    }
    return cache[key]


def _evaluate_calibration_batch(root: Path, slots: list[dict], batch: dict,
                                eval_cache: dict, spec: dict,
                                analysis_root: Path,
                                generated_roots: dict[str, Path] | None = None) -> dict:
    calibration_slot = _slot_by_id(slots, batch["run_id"])
    trial, yaw_case, scope = batch["trial_index"], batch["yaw_case"], batch["data_scope"]
    evaluation_run_id = _evaluation_run_id(slots, calibration_slot["rig_group"],
                                           int(calibration_slot["repetition"]))
    evaluation_slot = _slot_by_id(slots, evaluation_run_id)
    case_dir = _analysis_case_dir(analysis_root, calibration_slot, trial, yaw_case, scope)
    freeze_hash = _sha256(analysis_root / "analysis_freeze.json")
    existing_evaluation = case_dir / "evaluation_results.json"
    if existing_evaluation.is_file():
        saved = _read_json(existing_evaluation)
        if saved.get("freeze_sha256") != freeze_hash:
            raise ValueError(f"stored V0 result was produced under a different freeze: {existing_evaluation}")
        result_rows = saved.get("results", [])
        evaluation_hashes = {"evaluation_results.json": _sha256(existing_evaluation)}
        sensor_copy = case_dir / "evaluation_sensor_observations.json"
        if sensor_copy.is_file():
            evaluation_hashes["evaluation_sensor_observations.json"] = _sha256(sensor_copy)
        sensor_provenance_copy = case_dir / "evaluation_sensor_generation_provenance.json"
        if sensor_provenance_copy.is_file():
            evaluation_hashes["evaluation_sensor_generation_provenance.json"] = _sha256(
                sensor_provenance_copy
            )
        evaluation_hashes.update(_method_result_files(case_dir, "evaluation_results", result_rows))
        _write_or_verify_checksums(case_dir, "evaluation_checksums.json", evaluation_hashes)
        calibration_hashes = _read_json(case_dir / "calibration_checksums.json")
        combined = {**calibration_hashes, **evaluation_hashes}
        _write_or_verify_checksums(case_dir, "result_checksums.json", combined)
        return saved
    eval_data = _v0_evaluation_inputs(
        root, evaluation_slot, trial, scope, eval_cache, generated_roots,
    )
    if not eval_data["matched_200_met"]:
        status = {
            "schema_version": "stereo_v1_1_evaluation_results_v1",
            "calibration_run_id": calibration_slot["run_id"],
            "evaluation_run_id": evaluation_run_id,
            "trial_index": trial,
            "yaw_case": yaw_case,
            "data_scope": scope,
            "evaluation_status": "MATCHED_200_UNMET",
            "freeze_sha256": freeze_hash,
            "pair_count_available": len(eval_data["all_pairs"]),
            "required_pair_count": 200,
            "results": [],
        }
        _write_immutable(case_dir / "evaluation_results.json", status)
        evaluation_hashes = {
            "evaluation_results.json": _sha256(case_dir / "evaluation_results.json"),
        }
        _write_or_verify_checksums(case_dir, "evaluation_checksums.json", evaluation_hashes)
        calibration_checksums = _read_json(case_dir / "calibration_checksums.json")
        _write_or_verify_checksums(case_dir, "result_checksums.json", {
            **calibration_checksums,
            "evaluation_results.json": _sha256(case_dir / "evaluation_results.json"),
        })
        return status

    if eval_data["sensor"] != _read_json(case_dir / "sensor_observations.json"):
        raise ValueError("camera RTK/IMU trial realization changed between calibration and matching V0")
    _copy_verified(eval_data["sensor_path"], case_dir / "evaluation_sensor_observations.json")
    _copy_verified(eval_data["sensor_path"].parent / "sensor_generation_provenance.json",
                   case_dir / "evaluation_sensor_generation_provenance.json")
    true_camera = eval_data["true_camera"]
    results = []
    for calibration_result in batch["results"]:
        row = dict(calibration_result)
        row["rig_group"] = calibration_slot["rig_group"]
        results.append(_evaluation_record(
            row, evaluation_run_id, scope, eval_data["pair_summary"],
            eval_data["pairs"], true_camera, eval_data["gt_series"], spec,
        ))

    # M0 uses the measured camera RTK/IMU and initialized yaw without fitting.
    trial_config_path = case_dir / "trial_config.json"
    trial_config = _read_json(trial_config_path)
    initial_centers, initial_rotations = _initial_pose(eval_data["sensor"], trial_config)
    initial = {
        "method": "M0_sensor_only", "attitude_variant": "INITIAL",
        "camera_dof": 0, "latent_dof": 0,
        "fixed_camera_centers_world_m": initial_centers,
        "estimated_R_WC": {side: rotation.tolist() for side, rotation in initial_rotations.items()},
        "estimated_phi_theta_psi_rad": {
            side: [*eval_data["sensor"]["cameras"][side]["camera_imu_roll_pitch_mean_optical_rad"],
                  trial_config["initial_yaw_rad"][side]] for side in ("left", "right")
        },
        "convergence_status": "INITIAL_SENSOR_VALUES",
        "calibration_run_id": calibration_slot["run_id"], "mc_seed": trial,
        "rig_group": calibration_slot["rig_group"],
    }
    results.append(_evaluation_record(initial, evaluation_run_id, scope,
                                      eval_data["pair_summary"], eval_data["pairs"],
                                      true_camera, eval_data["gt_series"], spec))

    # ORACLE is a geometric reference, not a fitted calibration method.
    oracle = {
        "method": "ORACLE_true_camera_pose", "attitude_variant": "ORACLE",
        "camera_dof": 0, "latent_dof": 0,
        "fixed_camera_centers_world_m": {side: true_camera[side]["center"].tolist()
                                          for side in ("left", "right")},
        "estimated_R_WC": {side: true_camera[side]["rotation"].tolist()
                           for side in ("left", "right")},
        "estimated_phi_theta_psi_rad": None,
        "convergence_status": "REFERENCE_ONLY_NOT_A_CALIBRATION_ALGORITHM",
        "calibration_run_id": calibration_slot["run_id"], "mc_seed": trial,
        "rig_group": calibration_slot["rig_group"],
        "diagnostic_only": True,
    }
    results.append(_evaluation_record(oracle, evaluation_run_id, scope,
                                      eval_data["pair_summary"], eval_data["pairs"],
                                      true_camera, eval_data["gt_series"], spec))

    payload = {
        "schema_version": "stereo_v1_1_evaluation_results_v1",
        "calibration_run_id": calibration_slot["run_id"],
        "evaluation_run_id": evaluation_run_id,
        "trial_index": trial,
        "yaw_case": yaw_case,
        "data_scope": scope,
        "evaluation_status": "COMPLETE",
        "freeze_sha256": freeze_hash,
        "v0_recalibration_performed": False,
        "results": results,
    }
    _write_immutable(case_dir / "evaluation_results.json", payload)
    evaluation_hashes = {
        "evaluation_sensor_observations.json": _sha256(case_dir / "evaluation_sensor_observations.json"),
        "evaluation_sensor_generation_provenance.json": _sha256(
            case_dir / "evaluation_sensor_generation_provenance.json"
        ),
        "evaluation_results.json": _sha256(case_dir / "evaluation_results.json"),
    }
    evaluation_hashes.update(_method_result_files(case_dir, "evaluation_results", results))
    _write_or_verify_checksums(case_dir, "evaluation_checksums.json", evaluation_hashes)
    calibration_checksums = _read_json(case_dir / "calibration_checksums.json")
    combined = {
        **calibration_checksums,
        **evaluation_hashes,
    }
    _write_or_verify_checksums(case_dir, "result_checksums.json", combined)
    return payload


def _analysis_version_id(mc_trials: int, source_identity_sha256: str = "") -> str:
    identity = {
        "algorithm_sha256": _analysis_hashes(),
        "dependencies": _analysis_dependencies(),
        "protocol_sha256": _read_json(PROTOCOL_ROOT / "provenance.json")["files_sha256"],
        "source_identity_sha256": source_identity_sha256,
        "mc_trial_count": int(mc_trials),
        "pixel_sigma_px": PIXEL_SIGMA_PX,
        "huber_f_scale": HUBER_F_SCALE,
        "matched_sample_count": 200,
    }
    digest = hashlib.sha256(_json_text(identity).encode("utf-8")).hexdigest()[:12]
    return f"analysis_{digest}"


def _write_text_immutable(path: Path, content: str) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = content if content.endswith("\n") else content + "\n"
    if path.exists():
        if path.read_text(encoding="utf-8") != encoded:
            raise FileExistsError(f"derived report exists with different content; refusing overwrite: {path}")
    else:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _methodology_markdown(mc_trials: int, selected_path: str | None) -> str:
    return f"""# Stereo calibration analysis protocol

Analysis version uses {mc_trials} deterministic offline Monte Carlo trials for the primary comparison. The selected G0 path is `{selected_path or 'undetermined'}` from covariance propagation before V0 position errors are read. The staged sensitivity matrix uses one deterministic offline realization per factor level and adds no Gazebo runs.

## Methods

- `M0_sensor_only`: fixed measured camera optical-center RTK positions, measured camera-IMU roll/pitch, and declared yaw initialization; no image calibration.
- `A_rtk_independent`: each camera attitude is calibrated independently from its own YOLO image centers and the same noisy 10 Hz drone RTK positions. Drone RTK covariance is propagated through the image projection Jacobian into pixel residual covariance.
- `B_rtk_joint`: both cameras share one latent world-ENU point per image pair; a Gaussian residual uses the same drone RTK observation and covariance as A.
- `C_no_drone_rtk_joint`: both cameras share one latent world-ENU point per pair, optimized from stereo pixels only. It receives no drone RTK, GT positions, or commanded path.
- `RPY_IMU`: estimates each camera's optical roll, pitch correction, and world heading with finite camera-mounted IMU roll/pitch priors.
- `YAW_ONLY`: estimates yaw only; roll/pitch equal the same measured camera-IMU means used by RPY_IMU.
- `YAW_IDEAL`: separate diagnostic using true camera centers, true roll/pitch, and exact synthetic drone RTK; it is never included in the main comparison.
- `ORACLE_true_camera_pose`: geometric reference using the recorded Gazebo camera poses; it is not a calibration algorithm.

## Input and coordinate contracts

The estimator reads only allowlisted camera RTK/IMU observations, declared yaw initialization, original-pixel left/right detections, and drone RTK for methods A/B. Camera positions remain fixed. World positions use ENU `[east,north,up]`; camera rays use optical RDF `[right,down,forward]`; `R_WC` maps optical vectors to world. Recorded camera quaternions are world-from-Gazebo-camera-link FLU and are converted through the explicit link-FLU-to-optical-RDF matrix before truth evaluation. Angles and their covariance use radians and radians squared. Main results use actual YOLO bbox centers without adding synthetic pixel noise.

Drone RTK is explicitly synthetic at the `vehicle_reference_link` and uses world ENU. Camera RTK/IMU are explicitly synthetic optical-center and camera-mounted observations. Pixel-noise sensitivity uses a separate synthetic-GT-projection branch; it never replaces the actual-YOLO main branch. Timestamp sensitivity shifts reported observation time before re-pairing/interpolation and never edits GT time. These files describe simulation inputs and do not claim physical sensor performance.

## Outputs and pass/fail criteria

Each trial writes `sensor_observations.json`, `trial_config.json`, `calibration_results.json`, and `evaluation_results.json`, plus method/variant-specific result files and checksums. A fit is `CONVERGED` only when the optimizer succeeds and the fitted Jacobian has full column rank; otherwise it remains `OPTIMIZER_FAILED` or `RANK_DEFICIENT` and its evidence is retained. The locked robust loss is Huber with standardized residual scale {HUBER_F_SCALE}. Fewer than 200 unique time-stratified pairs is `MATCHED_200_UNMET`; frames are never duplicated. There is no invented RMSE acceptance threshold because the protocol's target error is null. Detection, convergence, tracking, and position errors are reported as outcomes, not reasons to discard a run.

Sensitivity factors include camera-position RTK sigma, camera-IMU sigma, drone RTK sigma, shared drone RTK bias, initial yaw bias, ideal-projection pixel sigma, drone-RTK timestamp offset, and right-camera timestamp offset. Each is evaluated on G0 P0 and the path selected without V0 outcomes; cases keep all other factors at the main setting. The right-camera offset is re-paired at the protocol 1 ms tolerance and may yield zero usable pairs. Sigma-zero RTK is an exact offline diagnostic; zero camera-IMU covariance is used only by YAW_ONLY/YAW_IDEAL and is never inverted by RPY_IMU.

Sensitivity error deltas are compared with the same run's frozen Y0/all-data result only when both use the actual-YOLO branch and the same number of valid V0 estimates. Other rows retain their absolute metrics and are explicitly labeled as branch- or sample-incomparable. The directional labels describe this finite simulation sample, not hardware performance.

The analysis freezes the algorithm hashes, primary and sensitivity calibration checksums, settings, and path-selection result before reading V0 position error. V0 is used only to evaluate fixed calibration rotations and never to recalibrate the cameras.
"""


def _write_csv_immutable(path: Path, fieldnames: list[str], rows: list[dict]) -> str:
    import io

    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=fieldnames, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)
    # DictWriter emits CRLF records even on POSIX. Canonicalize before the
    # text-level immutable comparison, whose reads normalize newlines.
    content = buffer.getvalue().replace("\r\n", "\n").replace("\r", "\n")
    return _write_text_immutable(path, content)


def _summarize_evaluations(evaluation_payloads: list[dict]) -> dict:
    rows = [result for payload in evaluation_payloads for result in payload.get("results", [])]
    main = [row for row in rows if row.get("attitude_variant") in VARIANTS
            and row.get("method") in METHODS]
    convergence: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for payload in evaluation_payloads:
        for result in payload.get("calibration_results", []):
            key = f"{result.get('method')}__{result.get('attitude_variant')}"
            convergence[key][str(result.get("convergence_status"))] += 1
    grouped: dict[tuple, list[dict]] = defaultdict(list)
    for row in main:
        metrics = row.get("metrics") or {}
        key = (row.get("data_scope"), row.get("method"), row.get("attitude_variant"))
        grouped[key].append(metrics)
    summary_rows = []
    for (scope, method, variant), metrics_rows in sorted(grouped.items()):
        rmse = [row["position_3d_rmse_m"] for row in metrics_rows if row.get("position_3d_rmse_m") is not None]
        p95 = [row["position_3d_p95_m"] for row in metrics_rows if row.get("position_3d_p95_m") is not None]
        detect = [row["detection_rate"] for row in metrics_rows if row.get("detection_rate") is not None]
        valid = [row["valid_measurement_rate"] for row in metrics_rows if row.get("valid_measurement_rate") is not None]
        summary_rows.append({
            "data_scope": scope, "method": method, "attitude_variant": variant,
            "evaluated_run_count": len(metrics_rows),
            "position_3d_rmse_mean_m": statistics.mean(rmse) if rmse else None,
            "position_3d_p95_mean_m": statistics.mean(p95) if p95 else None,
            "detection_rate_mean": statistics.mean(detect) if detect else None,
            "valid_measurement_rate_mean": statistics.mean(valid) if valid else None,
        })
    return {
        "schema_version": "stereo_v1_1_analysis_summary_v1",
        "evaluation_result_count": len(rows),
        "main_comparison_result_count": len(main),
        "main_comparison_summary": summary_rows,
        "calibration_convergence_counts": {key: dict(value) for key, value in convergence.items()},
        "position_metrics_status": "V0 errors were opened after analysis freeze",
        "methodology": "analysis_methodology.md",
    }


def _input_root_identity(input_root: Path, output_root: Path) -> str:
    if input_root == output_root:
        return "output_root"
    if input_root.is_relative_to(SIM_ROOT.parents[1].resolve()):
        return str(input_root.relative_to(SIM_ROOT.parents[1].resolve()))
    return "configured_external_input_root"


def analyze(output_root: Path, mc_trials: int = 100,
            input_root: Path | None = None) -> dict:
    output_root = Path(output_root).resolve()
    root = Path(input_root).resolve() if input_root is not None else output_root
    if root != output_root and (root.is_relative_to(output_root) or output_root.is_relative_to(root)):
        raise ValueError("separate analysis output and source input roots must not overlap")
    output_root.mkdir(parents=True, exist_ok=True)
    status, slots = _status_slots(root)
    environment_report_path = next((candidate for candidate in (
        root / "environment_report.json", root / "environment_report.md",
        output_root / "environment_report.json", output_root / "environment_report.md",
    ) if candidate.is_file()), None)
    if environment_report_path is None:
        raise ValueError("environment report missing; run preflight before collecting or add the source report for offline reprocessing")
    if environment_report_path.suffix == ".json":
        environment_report = _read_json(environment_report_path)
        if environment_report.get("status") != "PREFLIGHT_PASS":
            raise ValueError("environment preflight is not passing")
        environment_report_source = "environment_report.json:PREFLIGHT_PASS"
    else:
        # Existing, already-QC-valid campaigns may carry the earlier Markdown
        # environment inventory. This permits offline reprocessing while keeping
        # it distinct from a fresh live preflight result.
        environment_report = {"status": "HISTORICAL_SOURCE_REPORT"}
        environment_report_source = "environment_report.md; no live preflight rerun claimed"
    protocol = _read_json(PROTOCOL_ROOT / "03_experiment_spec.json")
    source_manifest_names = (
        "run_status.json", "manifest_actual.json", "qc_summary.json",
        "checksums.sha256", "frozen_protocol.json", "schema_mapping.md",
    )
    source_manifest_hashes = {
        name: _sha256(root / name) for name in source_manifest_names
        if (root / name).is_file()
    }
    source_attempt_checksums = {
        f"{slot['run_id']}/{slot['attempts'][-1]['attempt_id']}": _sha256(
            root / slot["attempts"][-1]["recording_path"] / "checksums.sha256"
        )
        for slot in slots
        if (root / slot["attempts"][-1]["recording_path"] / "checksums.sha256").is_file()
    }
    source_identity = {
        "input_root_identity": _input_root_identity(root, output_root),
        "environment_report_sha256": _sha256(environment_report_path),
        "campaign_manifest_sha256": source_manifest_hashes,
        "attempt_checksums_sha256": source_attempt_checksums,
    }
    source_identity_sha256 = hashlib.sha256(_json_text(source_identity).encode("utf-8")).hexdigest()
    analysis_id = _analysis_version_id(mc_trials, source_identity_sha256)
    analysis_root = output_root / "analysis_versions" / analysis_id
    analysis_root.mkdir(parents=True, exist_ok=True)
    run_manifest = {
        "schema_version": "stereo_v1_1_analysis_run_manifest_v1",
        "analysis_id": analysis_id,
        "mc_trial_count": mc_trials,
        "source_valid_run_count": status["valid_run_count"],
        "source_identity_sha256": source_identity_sha256,
        "input_root_identity": _input_root_identity(root, output_root),
        "protocol_sha256": _read_json(PROTOCOL_ROOT / "provenance.json")["files_sha256"],
        "algorithm_sha256": _analysis_hashes(),
        "dependencies": _analysis_dependencies(),
        "pixel_sigma_px": PIXEL_SIGMA_PX,
        "huber_f_scale": HUBER_F_SCALE,
        "environment_report_source": environment_report_source,
        "environment_report_sha256": _sha256(environment_report_path),
        "source_campaign_manifest_sha256": source_manifest_hashes,
        "source_attempt_checksums_sha256": source_attempt_checksums,
    }
    _write_immutable(analysis_root / "analysis_run_manifest.json", run_manifest)

    print(f"Generating shared camera sensor observations for {mc_trials} MC trials…", flush=True)
    calibration_manifest, path_calibrations, sensor_paths, generated_roots = _calibrate_all(
        root, slots, protocol, mc_trials, analysis_root
    )
    path_selection = _select_calibration_path(root, slots, path_calibrations,
                                              sensor_paths, protocol)
    sensitivity_manifest = _run_sensitivity_calibrations(
        root, slots, protocol, path_selection, analysis_root,
    )
    _write_text_immutable(analysis_root / "analysis_methodology.md",
                          _methodology_markdown(mc_trials, path_selection["selected_path_id"]))
    freeze = _freeze_analysis(analysis_root, path_selection, mc_trials, protocol,
                              sensitivity_manifest)
    if not (analysis_root / "analysis_freeze.json").is_file():
        raise RuntimeError("analysis freeze was not persisted; V0 evaluation is forbidden")
    print(f"Analysis frozen ({analysis_id}); starting locked V0 evaluation…", flush=True)

    eval_cache: dict = {}
    evaluation_payloads = []
    batches = calibration_manifest["calibration_batches"]
    for index, batch in enumerate(batches, start=1):
        print(f"V0 evaluation {index}/{len(batches)}: {batch['run_id']} "
              f"trial={batch['trial_index']} yaw={batch['yaw_case']} scope={batch['data_scope']}", flush=True)
        payload = _evaluate_calibration_batch(root, slots, batch, eval_cache,
                                              protocol, analysis_root, generated_roots)
        evaluation_payloads.append({
            "calibration_results": batch["results"],
            **payload,
        })
    sensitivity_summary = _evaluate_sensitivity_cases(
        root, slots, protocol, analysis_root, sensitivity_manifest,
    )
    summary = _summarize_evaluations(evaluation_payloads)
    summary.update({
        "analysis_id": analysis_id,
        "source_identity_sha256": source_identity_sha256,
        "input_root_identity": _input_root_identity(root, output_root),
        "analysis_freeze_sha256": _sha256(analysis_root / "analysis_freeze.json"),
        "source_valid_run_count": status["valid_run_count"],
        "calibration_batch_count": len(batches),
        "calibration_result_count": calibration_manifest["calibration_result_count"],
        "mc_trial_count": mc_trials,
        "sensitivity_case_count": sensitivity_summary["case_count"],
        "sensitivity_trial_count_per_case": sensitivity_summary["trial_count_per_case"],
        "sensitivity_summary": "sensitivity_summary.json",
        "selected_path_id": path_selection["selected_path_id"],
        "path_selection_status": path_selection["status"],
        "v0_position_errors_evaluated_after_freeze": True,
        "v0_recalibration_performed": False,
        "limitations": [
            "Synthetic RTK/IMU models are design-level observations, not measured hardware performance.",
            "A's RTK uncertainty is propagated through a first-order pixel covariance approximation.",
            "C may be rank-deficient; its result is preserved without adding a hidden yaw or trajectory prior.",
            "No numeric pass threshold is defined by the protocol.",
        ],
    })
    _write_immutable(analysis_root / "analysis_summary.json", summary)
    _write_immutable(analysis_root / "calibration_run_index.json", {
        "schema_version": "stereo_v1_1_calibration_run_index_v1",
        "batch_count": len(batches),
        "batches": [{key: row[key] for key in ("run_id", "trial_index", "yaw_case", "data_scope")}
                    for row in batches],
    })
    evaluation_rows = []
    calibration_rows = []
    for payload in evaluation_payloads:
        for result in payload.get("calibration_results", []):
            calibration_rows.append({
                "calibration_run_id": payload["calibration_run_id"],
                "evaluation_run_id": payload["evaluation_run_id"],
                "trial_index": payload["trial_index"], "yaw_case": payload["yaw_case"],
                "data_scope": payload["data_scope"], "method": result.get("method"),
                "attitude_variant": result.get("attitude_variant"),
                "camera_dof": result.get("camera_dof"), "latent_dof": result.get("latent_dof"),
                "convergence_status": result.get("convergence_status"),
                "condition_number": (result.get("conditioning") or {}).get("condition_number"),
            })
        for result in payload.get("results", []):
            metrics = result.get("metrics") or {}
            evaluation_rows.append({
                "calibration_run_id": result.get("calibration_run_id"),
                "evaluation_run_id": result.get("evaluation_run_id"),
                "trial_index": result.get("mc_seed"), "yaw_case": payload["yaw_case"],
                "data_scope": payload["data_scope"], "method": result.get("method"),
                "attitude_variant": result.get("attitude_variant"),
                "evaluation_status": result.get("evaluation_status"),
                "detection_rate": metrics.get("detection_rate"),
                "valid_measurement_rate": metrics.get("valid_measurement_rate"),
                "lateral_east_rmse_m": (metrics.get("axis_rmse_m") or {}).get("lateral_east"),
                "forward_north_rmse_m": (metrics.get("axis_rmse_m") or {}).get("forward_north"),
                "vertical_up_rmse_m": (metrics.get("axis_rmse_m") or {}).get("vertical_up"),
                "position_3d_rmse_m": metrics.get("position_3d_rmse_m"),
                "position_3d_p95_m": metrics.get("position_3d_p95_m"),
            })
    calibration_fields = ["calibration_run_id", "evaluation_run_id", "trial_index", "yaw_case",
                          "data_scope", "method", "attitude_variant", "camera_dof", "latent_dof",
                          "convergence_status", "condition_number"]
    evaluation_fields = ["calibration_run_id", "evaluation_run_id", "trial_index", "yaw_case",
                         "data_scope", "method", "attitude_variant", "evaluation_status",
                         "detection_rate", "valid_measurement_rate", "lateral_east_rmse_m",
                         "forward_north_rmse_m", "vertical_up_rmse_m", "position_3d_rmse_m",
                         "position_3d_p95_m"]
    _write_csv_immutable(analysis_root / "calibration_metrics.csv", calibration_fields, calibration_rows)
    _write_csv_immutable(analysis_root / "evaluation_metrics.csv", evaluation_fields, evaluation_rows)
    checksums = {}
    for path in sorted(analysis_root.rglob("*")):
        if path.is_file() and path.name != "analysis_checksums.sha256":
            checksums[str(path.relative_to(analysis_root))] = _sha256(path)
    lines = "".join(f"{digest}  {relative}\n" for relative, digest in sorted(checksums.items()))
    _write_text_immutable(analysis_root / "analysis_checksums.sha256", lines)
    latest_pointer = output_root / "analysis_latest.json"
    latest_temporary = output_root / ".analysis_latest.json.tmp"
    with latest_temporary.open("w", encoding="utf-8") as handle:
        handle.write(_json_text({
            "schema_version": "stereo_v1_1_latest_analysis_pointer_v1",
            "analysis_id": analysis_id,
            "analysis_summary": str((analysis_root / "analysis_summary.json").relative_to(output_root)),
            "analysis_freeze_sha256": _sha256(analysis_root / "analysis_freeze.json"),
        }))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(latest_temporary, latest_pointer)
    print(json.dumps({"status": "ANALYSIS_COMPLETE", "analysis_id": analysis_id,
                      "selected_path_id": path_selection["selected_path_id"],
                      "calibration_batches": len(batches),
                      "evaluation_rows": len(evaluation_rows),
                      "output_relative": str(analysis_root.relative_to(output_root))},
                     ensure_ascii=False), flush=True)
    return summary


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, default=None,
                        help="read-only existing campaign root; defaults to --output-root")
    parser.add_argument("--mc-trials", type=int, default=100)
    args = parser.parse_args()
    if args.mc_trials < 1 or args.mc_trials > 1000:
        parser.error("--mc-trials must be between 1 and 1000")
    try:
        analyze(args.output_root, args.mc_trials, input_root=args.input_root)
    except (OSError, ValueError, RuntimeError, KeyError, FileExistsError) as exc:
        print(json.dumps({"status": "ANALYSIS_FAILED", "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2
    return 0


def _calibrate_all(root: Path, slots: list[dict], spec: dict, mc_trials: int,
                   analysis_root: Path) -> tuple[dict, dict, dict, dict[str, Path]]:
    if mc_trials < 1:
        raise ValueError("mc_trials must be at least one")
    all_runs = []
    calibration_slots = [row for row in slots if row["path_id"] != "V0"]
    representative_ids = {
        (group, path_id, 1)
        for group in ("G0", "G1") for path_id in ("P0", "P4")
    }
    representative_slots = [row for row in calibration_slots
                            if (row["rig_group"], row["path_id"], int(row["repetition"])) in representative_ids]
    input_cache: dict = {}
    path_calibrations: dict[tuple[str, str, str], list[dict]] = {}
    sensor_paths: dict[tuple[str, int], Path] = {}
    generated_roots = {
        "camera": analysis_root / "generated_inputs" / "camera",
        "drone_rtk": analysis_root / "generated_inputs" / "drone_rtk",
    }

    for trial in range(mc_trials):
        generated = generate_camera_sensors(
            root, trial, output_root=generated_roots["camera"],
        )
        for sensor_path in generated:
            parts = sensor_path.parts
            # .../<run_id>/<attempt_id>/sensors/trial_NNN/sensor_observations.json
            run_id = parts[-5]
            sensor_paths[(run_id, trial)] = sensor_path
        active_slots = calibration_slots if trial == 0 else representative_slots
        for slot_index, slot in enumerate(active_slots, start=1):
            if trial == 0:
                cases = _yaw_cases(slot["rig_group"], spec)
                scopes = (ALL_DATA, MATCHED_DATA)
            else:
                cases = ([ ("G1_PRIMARY_ZERO", {"left": 0.0, "right": 0.0}) ]
                         if slot["rig_group"] == "G1" else
                         [("Y0", {"left": 0.0, "right": 0.0})])
                scopes = (ALL_DATA,)
            print(f"CALIBRATION trial={trial:03d}/{mc_trials - 1:03d} "
                  f"slot={slot_index}/{len(active_slots)} run={slot['run_id']} "
                  f"yaw_cases={len(cases)}", flush=True)
            for yaw_case, bias in cases:
                for scope in scopes:
                    _payload, results = _calibrate_scope(
                        root, slot, trial, yaw_case, bias, scope, spec, slots,
                        input_cache, analysis_root, generated_roots,
                    )
                    all_runs.append({
                        "run_id": slot["run_id"], "trial_index": trial,
                        "yaw_case": yaw_case, "data_scope": scope,
                        "results": results,
                    })
                    if trial == 0 and yaw_case == "Y0" and scope == ALL_DATA:
                        path_calibrations[(slot["run_id"], yaw_case, scope)] = results
    return {"calibration_result_count": sum(len(row["results"]) for row in all_runs),
            "calibration_batches": all_runs}, path_calibrations, sensor_paths, generated_roots


if __name__ == "__main__":
    raise SystemExit(main())

