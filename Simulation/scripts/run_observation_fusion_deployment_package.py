#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path
from typing import Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

import apply_observation_fusion_runtime_manifest as fusion_apply  # noqa: E402
import apply_observation_fusion_with_runtime_profile as profile_apply  # noqa: E402
import audit_observation_fusion_runtime_manifest as manifest_audit  # noqa: E402
import audit_runtime_profile_fresh_seed_replay as profile_replay  # noqa: E402
import check_observation_fusion_deployment_package as acceptance_check  # noqa: E402
import check_observation_fusion_path_equivalence as path_equivalence  # noqa: E402
import check_observation_fusion_tier_override_smoke as tier_override_smoke  # noqa: E402
import check_stereodistance_observation_fusion_node_smoke as node_smoke  # noqa: E402
import check_stereodistance_operator_view_launch as operator_launch_check  # noqa: E402
import convert_stereo_bbox_observations_to_fusion_input as bbox_converter  # noqa: E402
import export_observation_fusion_deployment_inventory as package_inventory  # noqa: E402
import export_observation_fusion_runtime_manifest as manifest_export  # noqa: E402
import export_observation_fusion_public_topic_payload as public_payload_export  # noqa: E402
import export_observation_fusion_topic_contract_manifest as topic_contract_export  # noqa: E402
import extract_observation_fusion_input_from_calibration_csv as calibration_extractor  # noqa: E402
import publish_observation_fusion_public_topic_payload as public_payload_publisher  # noqa: E402
import validate_observation_fusion_runtime_input_schema as input_validator  # noqa: E402
import validate_observation_fusion_runtime_output_schema as output_validator  # noqa: E402
import validate_observation_fusion_public_topic_payload as public_payload_validator  # noqa: E402
from stereo_ground_sim.runtime_deployment_selector import RuntimeStereoCameraContract  # noqa: E402


DEFAULT_PROFILE_JSON = (
    "Simulation/reports/experiments/"
    "EXP-20260617-198-bbox-outlier-causal-image-prefilter/"
    "base_profile/recommended_runtime_profile.json"
)


def synthetic_multiburst_rows(
    *,
    camera: RuntimeStereoCameraContract,
    right_m: float = 4.0,
    down_m: float = 1.5,
    forward_m: float = 800.0,
    group_id: str = "sample0",
    burst_count: int = 3,
    observations_per_burst: int = 5,
) -> list[dict[str, object]]:
    disparity_px = camera.fx_px * camera.baseline_m / forward_m
    left_u_px = camera.cx_px + (right_m + camera.baseline_m * 0.5) * camera.fx_px / forward_m
    left_v_px = camera.cy_px + down_m * camera.fy_px / forward_m
    rows: list[dict[str, object]] = []
    base_offsets = (-1.0, 0.0, 1.0, -0.5, 0.5)
    for burst_index in range(max(1, int(burst_count))):
        common_offset = base_offsets[burst_index % len(base_offsets)]
        for obs_index in range(max(1, int(observations_per_burst))):
            within_offset = 0.1 * ((obs_index % 3) - 1)
            rows.append(
                {
                    "group_id": group_id,
                    "burst_id": f"b{burst_index}",
                    "left_u_px": left_u_px + common_offset + within_offset,
                    "left_v_px": left_v_px + within_offset,
                    "measured_disparity_px": disparity_px,
                }
            )
    return rows


def default_smoke_shape(manifest: Mapping[str, object]) -> tuple[int, int]:
    default_tier = str(manifest.get("default_recommendation", ""))
    tiers = manifest.get("tiers", [])
    if not isinstance(tiers, list):
        return 3, 5
    for tier in tiers:
        if not isinstance(tier, Mapping) or str(tier.get("tier", "")) != default_tier:
            continue
        adapter = tier.get("adapter")
        if not isinstance(adapter, Mapping):
            break
        if str(adapter.get("class", "")) == "MultiBurstObservationFusion":
            return (
                max(1, int(float(adapter.get("min_bursts", 3)))),
                max(1, int(float(adapter.get("min_observations_per_burst", 5)))),
            )
        if str(adapter.get("class", "")) == "RepeatedObservationFusion":
            return 1, max(1, int(float(adapter.get("min_observations", 3))))
    return 3, 5


def default_tier_name(manifest: Mapping[str, object]) -> str:
    return str(manifest.get("default_recommendation", ""))


def low_latency_tier_name(manifest: Mapping[str, object]) -> str:
    default_tier = default_tier_name(manifest)
    tiers = manifest.get("tiers", [])
    if not isinstance(tiers, list):
        return ""
    repeated_candidates: list[str] = []
    for tier in tiers:
        if not isinstance(tier, Mapping):
            continue
        tier_name = str(tier.get("tier", ""))
        adapter = tier.get("adapter", {})
        if not isinstance(adapter, Mapping):
            continue
        if tier_name != default_tier and adapter.get("class") == "RepeatedObservationFusion":
            repeated_candidates.append(tier_name)
    if repeated_candidates:
        return sorted(repeated_candidates)[0]
    for tier in tiers:
        if isinstance(tier, Mapping) and str(tier.get("tier", "")) != default_tier:
            return str(tier.get("tier", ""))
    return ""


def synthetic_bbox_rows(observation_rows: list[Mapping[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for row in observation_rows:
        left_u = float(row["left_u_px"])
        left_v = float(row["left_v_px"])
        disparity = float(row["measured_disparity_px"])
        right_u = left_u - disparity
        width_px = 24.0
        height_px = 18.0
        rows.append(
            {
                "group_id": row["group_id"],
                "burst_id": row["burst_id"],
                "left_bbox_xmin_px": left_u - width_px * 0.5,
                "left_bbox_ymin_px": left_v - height_px * 0.5,
                "left_bbox_xmax_px": left_u + width_px * 0.5,
                "left_bbox_ymax_px": left_v + height_px * 0.5,
                "right_bbox_xmin_px": right_u - width_px * 0.5,
                "right_bbox_ymin_px": left_v - height_px * 0.5,
                "right_bbox_xmax_px": right_u + width_px * 0.5,
                "right_bbox_ymax_px": left_v + height_px * 0.5,
            }
        )
    return rows


def synthetic_calibration_rows(observation_rows: list[Mapping[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for index, row in enumerate(observation_rows):
        left_u = float(row["left_u_px"])
        left_v = float(row["left_v_px"])
        disparity = float(row["measured_disparity_px"])
        rows.append(
            {
                "time_sec": index * 0.05,
                "group_id": row["group_id"],
                "burst_id": row["burst_id"],
                "truth_north_m": 800.0,
                "truth_east_m": 4.0,
                "truth_down_m": 1.5,
                "truth_forward_m": 800.0,
                "truth_right_m": 4.0,
                "left_cx_px": left_u,
                "left_cy_px": left_v,
                "right_cx_px": left_u - disparity,
                "right_cy_px": left_v,
                "measured_disparity_px": disparity,
            }
        )
    return rows


def write_rows(path: Path, rows: list[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_validation_text(path: Path, validation: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion runtime output schema validation",
        f"input_csv: {validation['input_csv']}",
        f"status: {validation['status']}",
        f"row_count: {validation['row_count']}",
        f"failure_count: {validation['failure_count']}",
        "Coordinate contract: internal RDF right/down/forward; public aliases FRD+range forward/right/down/range.",
    ]
    for failure in validation.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_package(
    *,
    root_dir: Path,
    output_dir: Path,
    profile_json: Path,
    disparity_residual_profile_json: Path | None = None,
    tier_specs: object | None = None,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    packaged_profile_json = output_dir / "active_runtime_profile.json"
    shutil.copyfile(profile_json, packaged_profile_json)
    packaged_disparity_residual_profile_json: Path | None = None
    if disparity_residual_profile_json is not None:
        packaged_disparity_residual_profile_json = output_dir / "disparity_residual_profile.json"
        shutil.copyfile(disparity_residual_profile_json, packaged_disparity_residual_profile_json)
    topic_manifest = topic_contract_export.build_manifest(topic_contract_export.DEFAULT_TOPICS)
    topic_contract_dir = output_dir / "topic_contract"
    write_json(topic_contract_dir / "observation_fusion_topic_contract_manifest.json", topic_manifest)
    topic_contract_export.write_csv(
        topic_contract_dir / "observation_fusion_topic_contract_manifest.csv",
        topic_manifest["interfaces"],
    )
    topic_contract_export.write_summary(
        topic_contract_dir / "observation_fusion_topic_contract_manifest_summary.txt",
        topic_manifest,
    )

    specs = tier_specs if tier_specs is not None else manifest_export.DEFAULT_TIERS
    manifest = manifest_export.build_manifest(root_dir.resolve(), specs)
    manifest_json = output_dir / "observation_fusion_runtime_manifest.json"
    write_json(manifest_json, manifest)
    manifest_export.write_csv(output_dir / "observation_fusion_runtime_manifest.csv", manifest["tiers"])
    manifest_export.write_summary(output_dir / "observation_fusion_runtime_manifest_summary.txt", manifest)

    camera = RuntimeStereoCameraContract()
    smoke_burst_count, smoke_observations_per_burst = default_smoke_shape(manifest)
    audit_rows = [
        manifest_audit.audit_tier(
            manifest_path=manifest_json,
            tier_name=tier_name,
            camera=camera,
            right_m=4.0,
            down_m=1.5,
            forward_m=800.0,
            tolerance_m=0.2,
        )
        for tier_name in manifest_audit.manifest_tier_names(manifest_json)
    ]
    audit_dir = output_dir / "audit"
    manifest_audit.write_csv(audit_dir / "observation_fusion_runtime_manifest_audit.csv", audit_rows)
    manifest_audit.write_summary(
        audit_dir / "observation_fusion_runtime_manifest_audit_summary.txt",
        audit_rows,
        manifest_json,
    )
    audit_payload = {
        "manifest_json": str(manifest_json),
        "pass_count": sum(1 for row in audit_rows if row.get("status") == "PASS"),
        "tier_count": len(audit_rows),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "rows": audit_rows,
    }
    write_json(audit_dir / "observation_fusion_runtime_manifest_audit.json", audit_payload)

    smoke_dir = output_dir / "profile_apply_smoke"
    input_csv = smoke_dir / "synthetic_multiburst_observations.csv"
    synthetic_observations = synthetic_multiburst_rows(
        camera=camera,
        burst_count=smoke_burst_count,
        observations_per_burst=smoke_observations_per_burst,
    )
    write_rows(input_csv, synthetic_observations)
    bbox_dir = output_dir / "bbox_conversion_smoke"
    bbox_input_csv = bbox_dir / "synthetic_stereo_bbox_observations.csv"
    write_rows(bbox_input_csv, synthetic_bbox_rows(synthetic_observations))
    bbox_columns, bbox_input_rows = bbox_converter.read_rows(bbox_input_csv)
    bbox_converted_rows, bbox_conversion_failures = bbox_converter.convert_rows(
        bbox_input_rows,
        columns=bbox_columns,
        group_field="group_id",
        burst_field="burst_id",
    )
    bbox_converted_csv = bbox_dir / "observation_fusion_bbox_converted_input.csv"
    bbox_converter.write_csv(bbox_converted_csv, bbox_converted_rows)
    bbox_conversion_summary = {
        "input_csv": str(bbox_input_csv),
        "output_csv": str(bbox_converted_csv),
        "status": "PASS" if bbox_converted_rows and not bbox_conversion_failures else "FAIL",
        "input_row_count": len(bbox_input_rows),
        "output_row_count": len(bbox_converted_rows),
        "failure_count": len(bbox_conversion_failures),
        "failures": bbox_conversion_failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    write_json(bbox_dir / "observation_fusion_bbox_conversion_summary.json", bbox_conversion_summary)
    bbox_converter.write_text(bbox_dir / "observation_fusion_bbox_conversion_summary.txt", bbox_conversion_summary)
    bbox_input_validation = input_validator.validate_csv(bbox_converted_csv, require_burst_id=True)
    write_json(
        bbox_dir / "observation_fusion_bbox_converted_input_schema_validation.json",
        bbox_input_validation,
    )
    input_validator.write_text(
        bbox_dir / "observation_fusion_bbox_converted_input_schema_validation.txt",
        bbox_input_validation,
    )
    input_validation_dir = smoke_dir / "input_schema_validation"
    input_validation = input_validator.validate_csv(input_csv, require_burst_id=True)
    write_json(
        input_validation_dir / "observation_fusion_runtime_input_schema_validation.json",
        input_validation,
    )
    input_validator.write_text(
        input_validation_dir / "observation_fusion_runtime_input_schema_validation.txt",
        input_validation,
    )
    adapter, tier_payload = fusion_apply.load_observation_fusion_adapter_from_manifest(manifest_json)
    input_rows = fusion_apply.read_rows(input_csv)
    fused_rows = fusion_apply.apply_adapter_to_groups(
        adapter=adapter,
        tier_payload=tier_payload,
        rows=input_rows,
        group_field="group_id",
        burst_field="burst_id",
    )
    raw_apply_dir = output_dir / "apply_smoke"
    raw_apply_csv = raw_apply_dir / "observation_fusion_runtime_samples.csv"
    fusion_apply.write_csv(raw_apply_csv, fused_rows)
    raw_apply_summary = {
        "manifest_json": str(manifest_json),
        "input_csv": str(input_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(fused_rows),
        "valid_sample_count": sum(1 for row in fused_rows if row.get("sample_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(raw_apply_csv),
    }
    write_json(raw_apply_dir / "observation_fusion_runtime_apply_summary.json", raw_apply_summary)
    fusion_apply.write_summary(
        raw_apply_dir / "observation_fusion_runtime_apply_summary.txt",
        fused_rows,
        manifest_path=manifest_json,
        tier_name=str(tier_payload.get("tier", "")),
    )

    profile = profile_replay.load_profile(profile_json)
    corrected_rows = profile_apply.apply_profile_to_fused_rows(profile=profile, fused_rows=fused_rows)
    corrected_csv = smoke_dir / "observation_fusion_corrected_runtime_samples.csv"
    fusion_apply.write_csv(corrected_csv, corrected_rows)
    profile_summary = {
        "manifest_json": str(manifest_json),
        "profile_json": str(profile_json),
        "packaged_profile_json": str(packaged_profile_json),
        "input_csv": str(input_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(corrected_rows),
        "valid_fused_sample_count": sum(1 for row in corrected_rows if row.get("sample_valid") is True),
        "valid_profile_output_count": sum(1 for row in corrected_rows if row.get("profile_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(corrected_csv),
    }
    write_json(smoke_dir / "observation_fusion_profile_apply_summary.json", profile_summary)
    profile_apply.write_summary(
        smoke_dir / "observation_fusion_profile_apply_summary.txt",
        corrected_rows,
        manifest_path=manifest_json,
        profile_path=profile_json,
        tier_name=str(tier_payload.get("tier", "")),
    )

    validation_dir = smoke_dir / "schema_validation"
    validation = output_validator.validate_csv(corrected_csv, tolerance_m=1.0e-6)
    write_json(validation_dir / "observation_fusion_runtime_output_schema_validation.json", validation)
    write_validation_text(validation_dir / "observation_fusion_runtime_output_schema_validation.txt", validation)

    bbox_fused_rows = fusion_apply.apply_adapter_to_groups(
        adapter=adapter,
        tier_payload=tier_payload,
        rows=fusion_apply.read_rows(bbox_converted_csv),
        group_field="group_id",
        burst_field="burst_id",
    )
    bbox_raw_apply_csv = bbox_dir / "observation_fusion_bbox_runtime_samples.csv"
    fusion_apply.write_csv(bbox_raw_apply_csv, bbox_fused_rows)
    bbox_raw_apply_summary = {
        "manifest_json": str(manifest_json),
        "input_csv": str(bbox_converted_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(bbox_fused_rows),
        "valid_sample_count": sum(1 for row in bbox_fused_rows if row.get("sample_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(bbox_raw_apply_csv),
    }
    write_json(bbox_dir / "observation_fusion_bbox_runtime_apply_summary.json", bbox_raw_apply_summary)
    fusion_apply.write_summary(
        bbox_dir / "observation_fusion_bbox_runtime_apply_summary.txt",
        bbox_fused_rows,
        manifest_path=manifest_json,
        tier_name=str(tier_payload.get("tier", "")),
    )
    bbox_corrected_rows = profile_apply.apply_profile_to_fused_rows(profile=profile, fused_rows=bbox_fused_rows)
    bbox_corrected_csv = bbox_dir / "observation_fusion_bbox_corrected_runtime_samples.csv"
    fusion_apply.write_csv(bbox_corrected_csv, bbox_corrected_rows)
    bbox_profile_summary = {
        "manifest_json": str(manifest_json),
        "profile_json": str(profile_json),
        "input_csv": str(bbox_converted_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(bbox_corrected_rows),
        "valid_fused_sample_count": sum(1 for row in bbox_corrected_rows if row.get("sample_valid") is True),
        "valid_profile_output_count": sum(1 for row in bbox_corrected_rows if row.get("profile_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(bbox_corrected_csv),
    }
    write_json(bbox_dir / "observation_fusion_bbox_profile_apply_summary.json", bbox_profile_summary)
    profile_apply.write_summary(
        bbox_dir / "observation_fusion_bbox_profile_apply_summary.txt",
        bbox_corrected_rows,
        manifest_path=manifest_json,
        profile_path=profile_json,
        tier_name=str(tier_payload.get("tier", "")),
    )
    bbox_output_validation = output_validator.validate_csv(bbox_corrected_csv, tolerance_m=1.0e-6)
    write_json(
        bbox_dir / "observation_fusion_bbox_output_schema_validation.json",
        bbox_output_validation,
    )
    write_validation_text(
        bbox_dir / "observation_fusion_bbox_output_schema_validation.txt",
        bbox_output_validation,
    )

    calibration_dir = output_dir / "calibration_csv_bridge_smoke"
    calibration_input_csv = calibration_dir / "synthetic_calibration_recorder_rows.csv"
    write_rows(calibration_input_csv, synthetic_calibration_rows(synthetic_observations))
    calibration_columns, calibration_input_rows = calibration_extractor.bbox_converter.read_rows(calibration_input_csv)
    calibration_extracted_rows, calibration_extract_failures, dropped_truth_columns = calibration_extractor.extract_rows(
        calibration_input_rows,
        columns=calibration_columns,
        group_field="group_id",
        burst_field="burst_id",
    )
    calibration_runtime_csv = calibration_dir / "observation_fusion_calibration_runtime_input.csv"
    bbox_converter.write_csv(calibration_runtime_csv, calibration_extracted_rows)
    calibration_extract_summary = {
        "input_csv": str(calibration_input_csv),
        "output_csv": str(calibration_runtime_csv),
        "status": "PASS" if calibration_extracted_rows and not calibration_extract_failures else "FAIL",
        "input_row_count": len(calibration_input_rows),
        "output_row_count": len(calibration_extracted_rows),
        "dropped_truth_column_count": len(dropped_truth_columns),
        "dropped_truth_columns": dropped_truth_columns,
        "failure_count": len(calibration_extract_failures),
        "failures": calibration_extract_failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    write_json(calibration_dir / "observation_fusion_calibration_extract_summary.json", calibration_extract_summary)
    calibration_extractor.write_text(
        calibration_dir / "observation_fusion_calibration_extract_summary.txt",
        calibration_extract_summary,
    )
    calibration_input_validation = input_validator.validate_csv(calibration_runtime_csv, require_burst_id=True)
    write_json(
        calibration_dir / "observation_fusion_calibration_input_schema_validation.json",
        calibration_input_validation,
    )
    input_validator.write_text(
        calibration_dir / "observation_fusion_calibration_input_schema_validation.txt",
        calibration_input_validation,
    )
    calibration_fused_rows = fusion_apply.apply_adapter_to_groups(
        adapter=adapter,
        tier_payload=tier_payload,
        rows=fusion_apply.read_rows(calibration_runtime_csv),
        group_field="group_id",
        burst_field="burst_id",
    )
    calibration_raw_apply_csv = calibration_dir / "observation_fusion_calibration_runtime_samples.csv"
    fusion_apply.write_csv(calibration_raw_apply_csv, calibration_fused_rows)
    calibration_raw_apply_summary = {
        "manifest_json": str(manifest_json),
        "input_csv": str(calibration_runtime_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(calibration_fused_rows),
        "valid_sample_count": sum(1 for row in calibration_fused_rows if row.get("sample_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(calibration_raw_apply_csv),
    }
    write_json(calibration_dir / "observation_fusion_calibration_runtime_apply_summary.json", calibration_raw_apply_summary)
    fusion_apply.write_summary(
        calibration_dir / "observation_fusion_calibration_runtime_apply_summary.txt",
        calibration_fused_rows,
        manifest_path=manifest_json,
        tier_name=str(tier_payload.get("tier", "")),
    )
    calibration_corrected_rows = profile_apply.apply_profile_to_fused_rows(
        profile=profile,
        fused_rows=calibration_fused_rows,
    )
    calibration_corrected_csv = calibration_dir / "observation_fusion_calibration_corrected_runtime_samples.csv"
    fusion_apply.write_csv(calibration_corrected_csv, calibration_corrected_rows)
    calibration_profile_summary = {
        "manifest_json": str(manifest_json),
        "profile_json": str(profile_json),
        "input_csv": str(calibration_runtime_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(calibration_corrected_rows),
        "valid_fused_sample_count": sum(1 for row in calibration_corrected_rows if row.get("sample_valid") is True),
        "valid_profile_output_count": sum(1 for row in calibration_corrected_rows if row.get("profile_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(calibration_corrected_csv),
    }
    write_json(
        calibration_dir / "observation_fusion_calibration_profile_apply_summary.json",
        calibration_profile_summary,
    )
    profile_apply.write_summary(
        calibration_dir / "observation_fusion_calibration_profile_apply_summary.txt",
        calibration_corrected_rows,
        manifest_path=manifest_json,
        profile_path=profile_json,
        tier_name=str(tier_payload.get("tier", "")),
    )
    calibration_output_validation = output_validator.validate_csv(calibration_corrected_csv, tolerance_m=1.0e-6)
    write_json(
        calibration_dir / "observation_fusion_calibration_output_schema_validation.json",
        calibration_output_validation,
    )
    write_validation_text(
        calibration_dir / "observation_fusion_calibration_output_schema_validation.txt",
        calibration_output_validation,
    )
    equivalence_dir = output_dir / "path_equivalence"
    equivalence = path_equivalence.compare_paths(
        reference_csv=corrected_csv,
        candidate_csvs={
            "bbox_converted": bbox_corrected_csv,
            "calibration_extracted": calibration_corrected_csv,
        },
        tolerance_m=1.0e-9,
    )
    write_json(equivalence_dir / "observation_fusion_path_equivalence.json", equivalence)
    path_equivalence.write_text(
        equivalence_dir / "observation_fusion_path_equivalence.txt",
        equivalence,
    )
    public_payload_dir = output_dir / "public_topic_payload"
    public_payload_rows, public_payload_failures = public_payload_export.payload_rows(
        corrected_rows,
        tolerance_m=1.0e-6,
    )
    public_payload_csv = public_payload_dir / "observation_fusion_public_topic_payload.csv"
    public_payload_export.write_csv(public_payload_csv, public_payload_rows)
    public_payload_summary = {
        "input_csv": str(corrected_csv),
        "output_csv": str(public_payload_csv),
        "topic": public_payload_export.PUBLIC_TOPIC,
        "message_type": public_payload_export.MESSAGE_TYPE,
        "status": "PASS" if public_payload_rows and not public_payload_failures else "FAIL",
        "input_row_count": len(corrected_rows),
        "payload_row_count": len(public_payload_rows),
        "failure_count": len(public_payload_failures),
        "failures": public_payload_failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    write_json(
        public_payload_dir / "observation_fusion_public_topic_payload_summary.json",
        public_payload_summary,
    )
    public_payload_export.write_text(
        public_payload_dir / "observation_fusion_public_topic_payload_summary.txt",
        public_payload_summary,
    )
    public_payload_validation = public_payload_validator.validate_csv(public_payload_csv, tolerance_m=1.0e-6)
    write_json(
        public_payload_dir / "observation_fusion_public_topic_payload_validation.json",
        public_payload_validation,
    )
    public_payload_validator.write_text(
        public_payload_dir / "observation_fusion_public_topic_payload_validation.txt",
        public_payload_validation,
    )
    public_payload_source_equivalence = public_payload_validator.validate_against_source(
        payload_csv=public_payload_csv,
        source_csv=corrected_csv,
        tolerance_m=1.0e-6,
    )
    write_json(
        public_payload_dir / "observation_fusion_public_topic_payload_source_equivalence.json",
        public_payload_source_equivalence,
    )
    public_payload_validator.write_source_equivalence_text(
        public_payload_dir / "observation_fusion_public_topic_payload_source_equivalence.txt",
        public_payload_source_equivalence,
    )
    public_payload_publish = public_payload_publisher.run(
        input_csv=public_payload_csv,
        output_dir=public_payload_dir,
        topic=public_payload_export.PUBLIC_TOPIC,
        rate_hz=10.0,
        once=True,
        dry_run=True,
    )
    node_smoke_payload = node_smoke.run_smoke(
        manifest_json=manifest_json,
        profile_json=profile_json,
        disparity_residual_profile_json=packaged_disparity_residual_profile_json,
    )
    write_json(output_dir / "stereodistance_observation_fusion_node_smoke.json", node_smoke_payload)
    node_smoke.write_text(
        output_dir / "stereodistance_observation_fusion_node_smoke.txt",
        node_smoke_payload,
    )
    tier_override_dir = output_dir / "tier_override_smoke"
    tier_override_rows = tier_override_smoke.smoke_rows(
        manifest_json=manifest_json,
        profile_json=profile_json,
    )
    tier_override_payload = tier_override_smoke.summary_payload(
        manifest_json=manifest_json,
        profile_json=profile_json,
        rows=tier_override_rows,
    )
    tier_override_smoke.write_csv(
        tier_override_dir / "observation_fusion_tier_override_smoke.csv",
        tier_override_rows,
    )
    write_json(tier_override_dir / "observation_fusion_tier_override_smoke.json", tier_override_payload)
    tier_override_smoke.write_text(
        tier_override_dir / "observation_fusion_tier_override_smoke.txt",
        tier_override_payload,
    )
    operator_view_fusion_launch = operator_launch_check.check_operator_view_launch(
        profile_json=profile_json,
        observation_fusion_manifest=manifest_json,
    )
    write_json(output_dir / "operator_view_fusion_launch_check.json", operator_view_fusion_launch)
    operator_launch_check.write_text(
        output_dir / "operator_view_fusion_launch_check.txt",
        operator_view_fusion_launch,
    )
    operator_view_tier_override_launch: dict[str, object] = {
        "status": "SKIP",
        "observation_fusion_tier": "",
        "runtime_truth_used": False,
    }
    override_tier = low_latency_tier_name(manifest)
    if override_tier:
        operator_view_tier_override_launch = operator_launch_check.check_operator_view_launch(
            profile_json=profile_json,
            observation_fusion_manifest=manifest_json,
            observation_fusion_tier=override_tier,
        )
    write_json(
        output_dir / "operator_view_fusion_tier_override_launch_check.json",
        operator_view_tier_override_launch,
    )
    operator_launch_check.write_text(
        output_dir / "operator_view_fusion_tier_override_launch_check.txt",
        operator_view_tier_override_launch,
    )
    operator_view_package_launch = operator_launch_check.check_operator_view_launch(
        package_dir=output_dir,
    )
    write_json(output_dir / "operator_view_package_launch_check.json", operator_view_package_launch)
    operator_launch_check.write_text(
        output_dir / "operator_view_package_launch_check.txt",
        operator_view_package_launch,
    )

    package_summary = {
        "output_dir": str(output_dir),
        "manifest_json": str(manifest_json),
        "profile_json": str(profile_json),
        "packaged_profile_json": str(packaged_profile_json),
        "disparity_residual_profile_json": (
            str(disparity_residual_profile_json) if disparity_residual_profile_json is not None else ""
        ),
        "packaged_disparity_residual_profile_json": (
            str(packaged_disparity_residual_profile_json)
            if packaged_disparity_residual_profile_json is not None
            else ""
        ),
        "default_tier": manifest.get("default_recommendation", ""),
        "topic_contract_manifest": str(topic_contract_dir / "observation_fusion_topic_contract_manifest.json"),
        "manifest_audit_pass_count": audit_payload["pass_count"],
        "manifest_audit_tier_count": audit_payload["tier_count"],
        "bbox_conversion_status": bbox_conversion_summary["status"],
        "bbox_converted_input_schema_validation_status": bbox_input_validation["status"],
        "bbox_raw_apply_valid_sample_count": bbox_raw_apply_summary["valid_sample_count"],
        "bbox_profile_apply_valid_output_count": bbox_profile_summary["valid_profile_output_count"],
        "bbox_output_schema_validation_status": bbox_output_validation["status"],
        "calibration_extract_status": calibration_extract_summary["status"],
        "calibration_dropped_truth_column_count": calibration_extract_summary["dropped_truth_column_count"],
        "calibration_input_schema_validation_status": calibration_input_validation["status"],
        "calibration_raw_apply_valid_sample_count": calibration_raw_apply_summary["valid_sample_count"],
        "calibration_profile_apply_valid_output_count": calibration_profile_summary["valid_profile_output_count"],
        "calibration_output_schema_validation_status": calibration_output_validation["status"],
        "path_equivalence_status": equivalence["status"],
        "public_topic_payload_status": public_payload_summary["status"],
        "public_topic_payload_validation_status": public_payload_validation["status"],
        "public_topic_payload_source_equivalence_status": public_payload_source_equivalence["status"],
        "public_topic_payload_publish_dry_run_status": public_payload_publish["status"],
        "public_topic_payload_row_count": public_payload_summary["payload_row_count"],
        "stereodistance_node_smoke_status": node_smoke_payload["status"],
        "stereodistance_node_smoke_fusion_status": node_smoke_payload["observation_fusion_status"],
        "stereodistance_node_smoke_disparity_residual_status": node_smoke_payload.get(
            "disparity_residual_status",
            "",
        ),
        "tier_override_smoke_status": tier_override_payload["status"],
        "tier_override_smoke_pass_count": tier_override_payload["pass_count"],
        "tier_override_smoke_tier_count": tier_override_payload["tier_count"],
        "operator_view_fusion_launch_status": operator_view_fusion_launch["status"],
        "operator_view_fusion_manifest_status": operator_view_fusion_launch["observation_fusion_manifest_status"],
        "operator_view_fusion_tier_override_launch_status": operator_view_tier_override_launch["status"],
        "operator_view_fusion_tier_override": operator_view_tier_override_launch.get("observation_fusion_tier", ""),
        "operator_view_package_launch_status": operator_view_package_launch["status"],
        "input_schema_validation_status": input_validation["status"],
        "raw_apply_valid_sample_count": raw_apply_summary["valid_sample_count"],
        "profile_apply_valid_output_count": profile_summary["valid_profile_output_count"],
        "schema_validation_status": validation["status"],
        "smoke_burst_count": smoke_burst_count,
        "smoke_observations_per_burst": smoke_observations_per_burst,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    write_json(output_dir / "observation_fusion_deployment_package_summary.json", package_summary)
    lines = [
        "Observation fusion deployment package",
        f"default_tier: {package_summary['default_tier']}",
        f"topic_contract_manifest: {package_summary['topic_contract_manifest']}",
        f"packaged_profile_json: {package_summary['packaged_profile_json']}",
        f"packaged_disparity_residual_profile_json: {package_summary['packaged_disparity_residual_profile_json']}",
        f"manifest_audit: {package_summary['manifest_audit_pass_count']}/{package_summary['manifest_audit_tier_count']}",
        f"bbox_conversion_status: {package_summary['bbox_conversion_status']}",
        f"bbox_converted_input_schema_validation_status: {package_summary['bbox_converted_input_schema_validation_status']}",
        f"bbox_raw_apply_valid_sample_count: {package_summary['bbox_raw_apply_valid_sample_count']}",
        f"bbox_profile_apply_valid_output_count: {package_summary['bbox_profile_apply_valid_output_count']}",
        f"bbox_output_schema_validation_status: {package_summary['bbox_output_schema_validation_status']}",
        f"calibration_extract_status: {package_summary['calibration_extract_status']}",
        f"calibration_dropped_truth_column_count: {package_summary['calibration_dropped_truth_column_count']}",
        f"calibration_input_schema_validation_status: {package_summary['calibration_input_schema_validation_status']}",
        f"calibration_raw_apply_valid_sample_count: {package_summary['calibration_raw_apply_valid_sample_count']}",
        f"calibration_profile_apply_valid_output_count: {package_summary['calibration_profile_apply_valid_output_count']}",
        f"calibration_output_schema_validation_status: {package_summary['calibration_output_schema_validation_status']}",
        f"path_equivalence_status: {package_summary['path_equivalence_status']}",
        f"public_topic_payload_status: {package_summary['public_topic_payload_status']}",
        f"public_topic_payload_validation_status: {package_summary['public_topic_payload_validation_status']}",
        f"public_topic_payload_source_equivalence_status: {package_summary['public_topic_payload_source_equivalence_status']}",
        f"public_topic_payload_publish_dry_run_status: {package_summary['public_topic_payload_publish_dry_run_status']}",
        f"public_topic_payload_row_count: {package_summary['public_topic_payload_row_count']}",
        f"stereodistance_node_smoke_status: {package_summary['stereodistance_node_smoke_status']}",
        f"stereodistance_node_smoke_fusion_status: {package_summary['stereodistance_node_smoke_fusion_status']}",
        "stereodistance_node_smoke_disparity_residual_status: "
        f"{package_summary['stereodistance_node_smoke_disparity_residual_status']}",
        f"tier_override_smoke_status: {package_summary['tier_override_smoke_status']}",
        f"tier_override_smoke: {package_summary['tier_override_smoke_pass_count']}/{package_summary['tier_override_smoke_tier_count']}",
        f"operator_view_fusion_launch_status: {package_summary['operator_view_fusion_launch_status']}",
        f"operator_view_fusion_manifest_status: {package_summary['operator_view_fusion_manifest_status']}",
        f"operator_view_fusion_tier_override_launch_status: {package_summary['operator_view_fusion_tier_override_launch_status']}",
        f"operator_view_fusion_tier_override: {package_summary['operator_view_fusion_tier_override']}",
        f"operator_view_package_launch_status: {package_summary['operator_view_package_launch_status']}",
        f"input_schema_validation_status: {package_summary['input_schema_validation_status']}",
        f"raw_apply_valid_sample_count: {package_summary['raw_apply_valid_sample_count']}",
        f"profile_apply_valid_output_count: {package_summary['profile_apply_valid_output_count']}",
        f"schema_validation_status: {package_summary['schema_validation_status']}",
        f"smoke_burst_count: {package_summary['smoke_burst_count']}",
        f"smoke_observations_per_burst: {package_summary['smoke_observations_per_burst']}",
        "runtime_truth_used: False",
        "Coordinate contract: image u/v/disparity -> raw RDF -> corrected RDF -> public FRD aliases.",
    ]
    (output_dir / "observation_fusion_deployment_package_summary.txt").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    acceptance = acceptance_check.check_package(output_dir)
    acceptance_dir = output_dir / "acceptance_check"
    write_json(acceptance_dir / "observation_fusion_deployment_package_acceptance.json", acceptance)
    acceptance_check.write_text(
        acceptance_dir / "observation_fusion_deployment_package_acceptance.txt",
        acceptance,
    )
    package_summary["acceptance_status"] = acceptance["status"]
    write_json(output_dir / "observation_fusion_deployment_package_summary.json", package_summary)
    lines.insert(-2, f"acceptance_status: {package_summary['acceptance_status']}")
    (output_dir / "observation_fusion_deployment_package_summary.txt").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    inventory = package_inventory.export_inventory(output_dir, output_dir / "inventory")
    return {
        **package_summary,
        "inventory_status": inventory["status"],
        "inventory_file_count": inventory["file_count"],
        "inventory_total_size_bytes": inventory["total_size_bytes"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the full observation-fusion runtime deployment package checks.")
    parser.add_argument("--root-dir", type=Path, default=Path("."))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, default=Path(DEFAULT_PROFILE_JSON))
    parser.add_argument(
        "--disparity-residual-profile-json",
        type=Path,
        default=None,
        help="Optional truth-free runtime residual disparity correction profile to package and smoke in StereoDistanceNode.",
    )
    parser.add_argument(
        "--tier-spec-json",
        type=Path,
        default=None,
        help="Optional JSON tier specs for observation fusion manifest export.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = run_package(
        root_dir=args.root_dir,
        output_dir=args.output_dir,
        profile_json=args.profile_json,
        disparity_residual_profile_json=args.disparity_residual_profile_json,
        tier_specs=manifest_export.load_tier_specs(args.tier_spec_json),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
