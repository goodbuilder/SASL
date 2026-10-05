#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Mapping


def load_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def bool_false(payload: Mapping[str, object], key: str) -> bool:
    return payload.get(key) is False


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def contains_truth_runtime_token(values: object) -> bool:
    if not isinstance(values, list):
        return False
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def default_tier_payload(manifest: Mapping[str, object]) -> Mapping[str, object]:
    default_tier = str(manifest.get("default_recommendation", ""))
    tiers = manifest.get("tiers", [])
    if not isinstance(tiers, list):
        return {}
    for tier in tiers:
        if isinstance(tier, Mapping) and str(tier.get("tier", "")) == default_tier:
            return tier
    return {}


def check_package(package_dir: Path) -> dict[str, object]:
    failures: list[str] = []
    required_files = {
        "package_summary": package_dir / "observation_fusion_deployment_package_summary.json",
        "packaged_profile": package_dir / "active_runtime_profile.json",
        "topic_contract": package_dir / "topic_contract" / "observation_fusion_topic_contract_manifest.json",
        "manifest": package_dir / "observation_fusion_runtime_manifest.json",
        "manifest_audit": package_dir / "audit" / "observation_fusion_runtime_manifest_audit.json",
        "bbox_conversion": package_dir / "bbox_conversion_smoke" / "observation_fusion_bbox_conversion_summary.json",
        "bbox_input_schema": package_dir
        / "bbox_conversion_smoke"
        / "observation_fusion_bbox_converted_input_schema_validation.json",
        "bbox_raw_apply": package_dir
        / "bbox_conversion_smoke"
        / "observation_fusion_bbox_runtime_apply_summary.json",
        "bbox_profile_apply": package_dir
        / "bbox_conversion_smoke"
        / "observation_fusion_bbox_profile_apply_summary.json",
        "bbox_output_schema": package_dir
        / "bbox_conversion_smoke"
        / "observation_fusion_bbox_output_schema_validation.json",
        "calibration_extract": package_dir
        / "calibration_csv_bridge_smoke"
        / "observation_fusion_calibration_extract_summary.json",
        "calibration_input_schema": package_dir
        / "calibration_csv_bridge_smoke"
        / "observation_fusion_calibration_input_schema_validation.json",
        "calibration_raw_apply": package_dir
        / "calibration_csv_bridge_smoke"
        / "observation_fusion_calibration_runtime_apply_summary.json",
        "calibration_profile_apply": package_dir
        / "calibration_csv_bridge_smoke"
        / "observation_fusion_calibration_profile_apply_summary.json",
        "calibration_output_schema": package_dir
        / "calibration_csv_bridge_smoke"
        / "observation_fusion_calibration_output_schema_validation.json",
        "path_equivalence": package_dir
        / "path_equivalence"
        / "observation_fusion_path_equivalence.json",
        "public_topic_payload": package_dir
        / "public_topic_payload"
        / "observation_fusion_public_topic_payload_summary.json",
        "public_topic_payload_validation": package_dir
        / "public_topic_payload"
        / "observation_fusion_public_topic_payload_validation.json",
        "public_topic_payload_source_equivalence": package_dir
        / "public_topic_payload"
        / "observation_fusion_public_topic_payload_source_equivalence.json",
        "public_topic_payload_publish": package_dir
        / "public_topic_payload"
        / "observation_fusion_public_topic_payload_publish_summary.json",
        "raw_apply": package_dir / "apply_smoke" / "observation_fusion_runtime_apply_summary.json",
        "profile_apply": package_dir / "profile_apply_smoke" / "observation_fusion_profile_apply_summary.json",
        "input_schema": package_dir
        / "profile_apply_smoke"
        / "input_schema_validation"
        / "observation_fusion_runtime_input_schema_validation.json",
        "output_schema": package_dir
        / "profile_apply_smoke"
        / "schema_validation"
        / "observation_fusion_runtime_output_schema_validation.json",
        "operator_view_fusion_launch": package_dir / "operator_view_fusion_launch_check.json",
        "operator_view_fusion_tier_override_launch": package_dir
        / "operator_view_fusion_tier_override_launch_check.json",
        "operator_view_package_launch": package_dir / "operator_view_package_launch_check.json",
        "stereodistance_node_smoke": package_dir / "stereodistance_observation_fusion_node_smoke.json",
        "tier_override_smoke": package_dir / "tier_override_smoke" / "observation_fusion_tier_override_smoke.json",
    }
    missing = [name for name, path in required_files.items() if not path.is_file()]
    for name in missing:
        failures.append(f"missing_required_file:{name}:{required_files[name]}")
    if missing:
        return {
            "status": "FAIL",
            "package_dir": str(package_dir),
            "failure_count": len(failures),
            "failures": failures,
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": False,
        }

    package_summary = load_json(required_files["package_summary"])
    packaged_profile = load_json(required_files["packaged_profile"])
    topic_contract = load_json(required_files["topic_contract"])
    manifest = load_json(required_files["manifest"])
    manifest_audit = load_json(required_files["manifest_audit"])
    bbox_conversion = load_json(required_files["bbox_conversion"])
    bbox_input_schema = load_json(required_files["bbox_input_schema"])
    bbox_raw_apply = load_json(required_files["bbox_raw_apply"])
    bbox_profile_apply = load_json(required_files["bbox_profile_apply"])
    bbox_output_schema = load_json(required_files["bbox_output_schema"])
    calibration_extract = load_json(required_files["calibration_extract"])
    calibration_input_schema = load_json(required_files["calibration_input_schema"])
    calibration_raw_apply = load_json(required_files["calibration_raw_apply"])
    calibration_profile_apply = load_json(required_files["calibration_profile_apply"])
    calibration_output_schema = load_json(required_files["calibration_output_schema"])
    path_equivalence = load_json(required_files["path_equivalence"])
    public_topic_payload = load_json(required_files["public_topic_payload"])
    public_topic_payload_validation = load_json(required_files["public_topic_payload_validation"])
    public_topic_payload_source_equivalence = load_json(required_files["public_topic_payload_source_equivalence"])
    public_topic_payload_publish = load_json(required_files["public_topic_payload_publish"])
    raw_apply = load_json(required_files["raw_apply"])
    profile_apply = load_json(required_files["profile_apply"])
    input_schema = load_json(required_files["input_schema"])
    output_schema = load_json(required_files["output_schema"])
    operator_view_fusion_launch = load_json(required_files["operator_view_fusion_launch"])
    operator_view_fusion_tier_override_launch = load_json(required_files["operator_view_fusion_tier_override_launch"])
    operator_view_package_launch = load_json(required_files["operator_view_package_launch"])
    stereodistance_node_smoke = load_json(required_files["stereodistance_node_smoke"])
    tier_override_smoke = load_json(required_files["tier_override_smoke"])
    residual_profile_path = package_dir / "disparity_residual_profile.json"
    residual_profile: object | None = None
    if residual_profile_path.is_file():
        residual_profile = load_json(residual_profile_path)

    for name, payload in (
        ("package_summary", package_summary),
        ("topic_contract", topic_contract),
        ("manifest", manifest),
        ("manifest_audit", manifest_audit),
        ("bbox_conversion", bbox_conversion),
        ("bbox_input_schema", bbox_input_schema),
        ("bbox_raw_apply", bbox_raw_apply),
        ("bbox_profile_apply", bbox_profile_apply),
        ("bbox_output_schema", bbox_output_schema),
        ("calibration_extract", calibration_extract),
        ("calibration_input_schema", calibration_input_schema),
        ("calibration_raw_apply", calibration_raw_apply),
        ("calibration_profile_apply", calibration_profile_apply),
        ("calibration_output_schema", calibration_output_schema),
        ("path_equivalence", path_equivalence),
        ("public_topic_payload", public_topic_payload),
        ("public_topic_payload_validation", public_topic_payload_validation),
        ("public_topic_payload_source_equivalence", public_topic_payload_source_equivalence),
        ("public_topic_payload_publish", public_topic_payload_publish),
        ("raw_apply", raw_apply),
        ("profile_apply", profile_apply),
        ("input_schema", input_schema),
        ("output_schema", output_schema),
        ("operator_view_fusion_launch", operator_view_fusion_launch),
        ("operator_view_fusion_tier_override_launch", operator_view_fusion_tier_override_launch),
        ("operator_view_package_launch", operator_view_package_launch),
        ("stereodistance_node_smoke", stereodistance_node_smoke),
        ("tier_override_smoke", tier_override_smoke),
    ):
        assert isinstance(payload, dict)
        require(bool_false(payload, "runtime_truth_used"), failures, f"{name}:runtime_truth_used_not_false")

    assert isinstance(topic_contract, dict)
    runtime_inputs = set(topic_contract.get("runtime_input_interfaces", []))
    require("calibration_truth" not in runtime_inputs, failures, "topic_contract:calibration_truth_marked_runtime_input")
    require("simulation_truth" not in runtime_inputs, failures, "topic_contract:simulation_truth_marked_runtime_input")
    require("stereo_bbox_observation" in runtime_inputs, failures, "topic_contract:bbox_not_runtime_input")
    require("measured_disparity" in runtime_inputs, failures, "topic_contract:disparity_not_runtime_input")
    frames = dict(topic_contract.get("frames", {}))
    require(
        frames.get("internal_measurement") == "stereo-center RDF right/down/forward",
        failures,
        "topic_contract:internal_frame_not_rdf",
    )
    require(
        frames.get("public_stereodistance") == "FRD+range forward/right/down/range",
        failures,
        "topic_contract:public_frame_not_frd_range",
    )

    assert isinstance(manifest_audit, dict)
    assert isinstance(manifest, dict)
    assert isinstance(package_summary, dict)
    assert isinstance(packaged_profile, dict)
    require(
        str(package_summary.get("packaged_profile_json", "")).endswith("active_runtime_profile.json"),
        failures,
        "package_summary:packaged_profile_missing",
    )
    require(
        "RDF(right,down,forward)" in str(packaged_profile.get("axis_contract", "")),
        failures,
        "packaged_profile:rdf_contract",
    )
    require(
        not contains_truth_runtime_token(packaged_profile.get("runtime_inputs", [])),
        failures,
        "packaged_profile:runtime_truth_input",
    )
    source_profile = package_summary.get("profile_json", "")
    if source_profile:
        source_path = Path(str(source_profile))
        if not source_path.is_absolute():
            source_path = Path.cwd() / source_path
        require(
            source_path.is_file() and sha256_file(source_path) == sha256_file(required_files["packaged_profile"]),
            failures,
            "packaged_profile:source_hash_mismatch",
        )
    packaged_residual_profile = str(package_summary.get("packaged_disparity_residual_profile_json", ""))
    source_residual_profile = str(package_summary.get("disparity_residual_profile_json", ""))
    residual_profile_status = "SKIP"
    residual_profile_id = ""
    residual_smoke_status = str(stereodistance_node_smoke.get("disparity_residual_status", ""))
    if packaged_residual_profile or source_residual_profile or residual_profile is not None:
        residual_profile_status = "PASS"
        require(residual_profile_path.is_file(), failures, "disparity_residual_profile:missing")
        require(
            packaged_residual_profile.endswith("disparity_residual_profile.json"),
            failures,
            "package_summary:disparity_residual_profile_missing",
        )
        if source_residual_profile:
            source_residual_path = Path(source_residual_profile)
            if not source_residual_path.is_absolute():
                source_residual_path = Path.cwd() / source_residual_path
            require(
                source_residual_path.is_file() and residual_profile_path.is_file()
                and sha256_file(source_residual_path) == sha256_file(residual_profile_path),
                failures,
                "disparity_residual_profile:source_hash_mismatch",
            )
        if isinstance(residual_profile, dict):
            residual_profile_id = str(residual_profile.get("profile_id", ""))
            require(
                str(residual_profile.get("profile_type", "")) in {
                    "DisparityResidualRuntimeProfile",
                    "disparity_residual_runtime_profile",
                },
                failures,
                "disparity_residual_profile:type",
            )
            require(
                residual_profile.get("runtime_truth_used") is False,
                failures,
                "disparity_residual_profile:runtime_truth_used",
            )
            require(
                residual_profile.get("coordinate_contracts_explicit") is True,
                failures,
                "disparity_residual_profile:coordinate_contract_missing",
            )
            require(
                not contains_truth_runtime_token(residual_profile.get("runtime_inputs", [])),
                failures,
                "disparity_residual_profile:runtime_truth_input",
            )
            require(
                str(residual_profile.get("feature_space", ""))
                in {
                    "disparity",
                    "inverse_depth",
                    "log_disparity",
                    "raw_forward",
                    "log_raw_forward",
                    "disparity_forward_product",
                    "log_disparity_forward_product",
                },
                failures,
                "disparity_residual_profile:feature_space",
            )
            require(
                str(residual_profile.get("correction_space", "")) in {"forward_add", "ray_scale"},
                failures,
                "disparity_residual_profile:correction_space",
            )
        else:
            require(False, failures, "disparity_residual_profile:not_json_object")
        require(
            residual_smoke_status.startswith("applied:"),
            failures,
            "stereodistance_node_smoke:disparity_residual_not_applied",
        )
    default_tier = default_tier_payload(manifest)
    default_adapter = default_tier.get("adapter", {}) if isinstance(default_tier, Mapping) else {}
    default_evidence = default_tier.get("evidence", {}) if isinstance(default_tier, Mapping) else {}
    require(bool(default_tier), failures, "manifest:default_tier_missing")
    require(
        manifest.get("default_recommendation") == package_summary.get("default_tier"),
        failures,
        "manifest:default_tier_summary_mismatch",
    )
    require(
        str(default_tier.get("selection_scope", "")) == "all_scenario_aggregate",
        failures,
        "manifest:default_tier_not_all_scenario_aggregate",
    )
    require(
        isinstance(default_adapter, Mapping)
        and default_adapter.get("class") == "MultiBurstObservationFusion",
        failures,
        "manifest:default_tier_not_multiburst",
    )
    require(
        isinstance(default_evidence, Mapping)
        and default_evidence.get("status") == "STRICT_PASS",
        failures,
        "manifest:default_tier_not_strict_pass",
    )
    require(
        isinstance(default_evidence, Mapping)
        and default_evidence.get("runtime_truth_used") is False,
        failures,
        "manifest:default_tier_runtime_truth_used",
    )
    require(
        isinstance(default_evidence, Mapping)
        and int(default_evidence.get("pass_count", 0)) == int(default_evidence.get("trial_count", -1))
        and int(default_evidence.get("trial_count", 0)) > 1,
        failures,
        "manifest:default_tier_not_full_aggregate_pass",
    )
    require(
        int(manifest_audit.get("pass_count", -1)) == int(manifest_audit.get("tier_count", -2)),
        failures,
        "manifest_audit:not_all_tiers_passed",
    )
    require(int(manifest_audit.get("tier_count", 0)) > 0, failures, "manifest_audit:no_tiers")

    assert isinstance(bbox_conversion, dict)
    require(bbox_conversion.get("status") == "PASS", failures, "bbox_conversion:not_pass")
    require(
        int(bbox_conversion.get("output_row_count", 0)) == int(bbox_conversion.get("input_row_count", -1)),
        failures,
        "bbox_conversion:row_count_mismatch",
    )
    require(int(bbox_conversion.get("failure_count", -1)) == 0, failures, "bbox_conversion:failures_present")

    assert isinstance(calibration_extract, dict)
    require(calibration_extract.get("status") == "PASS", failures, "calibration_extract:not_pass")
    require(
        int(calibration_extract.get("output_row_count", 0)) == int(calibration_extract.get("input_row_count", -1)),
        failures,
        "calibration_extract:row_count_mismatch",
    )
    require(
        int(calibration_extract.get("dropped_truth_column_count", 0)) > 0,
        failures,
        "calibration_extract:no_truth_columns_dropped",
    )
    require(
        int(calibration_extract.get("failure_count", -1)) == 0,
        failures,
        "calibration_extract:failures_present",
    )

    for name, payload in (
        ("bbox_input_schema", bbox_input_schema),
        ("bbox_output_schema", bbox_output_schema),
        ("calibration_input_schema", calibration_input_schema),
        ("calibration_output_schema", calibration_output_schema),
        ("path_equivalence", path_equivalence),
        ("public_topic_payload", public_topic_payload),
        ("public_topic_payload_validation", public_topic_payload_validation),
        ("public_topic_payload_source_equivalence", public_topic_payload_source_equivalence),
        ("public_topic_payload_publish", public_topic_payload_publish),
        ("input_schema", input_schema),
        ("output_schema", output_schema),
    ):
        assert isinstance(payload, dict)
        require(payload.get("status") == "PASS", failures, f"{name}:not_pass")
        require(int(payload.get("failure_count", -1)) == 0, failures, f"{name}:failures_present")

    assert isinstance(operator_view_fusion_launch, dict)
    require(
        operator_view_fusion_launch.get("observation_fusion_manifest_status") == "PASS",
        failures,
        "operator_view_fusion_launch:manifest_not_pass",
    )
    require(
        operator_view_fusion_launch.get("launch_contract_status") == "PASS",
        failures,
        "operator_view_fusion_launch:launch_contract_not_pass",
    )
    require(
        operator_view_fusion_launch.get("observation_fusion_arg_present") is True,
        failures,
        "operator_view_fusion_launch:fusion_arg_missing",
    )
    require(
        operator_view_fusion_launch.get("bbox_adjustment_arg_present") is True,
        failures,
        "operator_view_fusion_launch:bbox_arg_missing",
    )
    assert isinstance(operator_view_fusion_tier_override_launch, dict)
    require(
        operator_view_fusion_tier_override_launch.get("status") == "PASS",
        failures,
        "operator_view_fusion_tier_override_launch:not_pass",
    )
    require(
        operator_view_fusion_tier_override_launch.get("observation_fusion_manifest_status") == "PASS",
        failures,
        "operator_view_fusion_tier_override_launch:manifest_not_pass",
    )
    require(
        operator_view_fusion_tier_override_launch.get("observation_fusion_arg_present") is True,
        failures,
        "operator_view_fusion_tier_override_launch:fusion_arg_missing",
    )
    require(
        bool(str(operator_view_fusion_tier_override_launch.get("observation_fusion_tier", ""))),
        failures,
        "operator_view_fusion_tier_override_launch:tier_missing",
    )
    require(
        "--observation-fusion-tier" in str(operator_view_fusion_tier_override_launch.get("operator_command_stdout", "")),
        failures,
        "operator_view_fusion_tier_override_launch:tier_arg_missing",
    )
    assert isinstance(operator_view_package_launch, dict)
    require(operator_view_package_launch.get("status") == "PASS", failures, "operator_view_package_launch:not_pass")
    require(
        operator_view_package_launch.get("package_type") == "lightweight_profile_package",
        failures,
        "operator_view_package_launch:package_type",
    )
    require(
        operator_view_package_launch.get("observation_fusion_manifest_status") == "PASS",
        failures,
        "operator_view_package_launch:manifest_not_pass",
    )
    require(
        operator_view_package_launch.get("observation_fusion_arg_present") is True,
        failures,
        "operator_view_package_launch:fusion_arg_missing",
    )
    require(
        str(operator_view_package_launch.get("resolved_runtime_profile", "")).endswith("active_runtime_profile.json"),
        failures,
        "operator_view_package_launch:profile_not_packaged",
    )
    assert isinstance(stereodistance_node_smoke, dict)
    require(
        str(stereodistance_node_smoke.get("observation_fusion_status", "")).startswith("fused:"),
        failures,
        "stereodistance_node_smoke:not_fused",
    )
    require(
        str(stereodistance_node_smoke.get("adapter_class", "")) in {"RepeatedObservationFusion", "MultiBurstObservationFusion"},
        failures,
        "stereodistance_node_smoke:adapter",
    )
    assert isinstance(tier_override_smoke, dict)
    require(tier_override_smoke.get("status") == "PASS", failures, "tier_override_smoke:not_pass")
    require(
        int(tier_override_smoke.get("pass_count", 0)) == int(tier_override_smoke.get("tier_count", -1)),
        failures,
        "tier_override_smoke:not_all_tiers_passed",
    )
    require(
        int(tier_override_smoke.get("default_tier_count", 0)) == 1,
        failures,
        "tier_override_smoke:default_tier_count",
    )
    require(
        tier_override_smoke.get("runtime_truth_used") is False,
        failures,
        "tier_override_smoke:runtime_truth_used",
    )

    assert isinstance(raw_apply, dict)
    require(int(raw_apply.get("valid_sample_count", 0)) > 0, failures, "raw_apply:no_valid_samples")
    assert isinstance(profile_apply, dict)
    require(
        int(profile_apply.get("valid_profile_output_count", 0)) > 0,
        failures,
        "profile_apply:no_valid_outputs",
    )
    assert isinstance(bbox_raw_apply, dict)
    require(
        int(bbox_raw_apply.get("valid_sample_count", 0)) > 0,
        failures,
        "bbox_raw_apply:no_valid_samples",
    )
    assert isinstance(bbox_profile_apply, dict)
    require(
        int(bbox_profile_apply.get("valid_profile_output_count", 0)) > 0,
        failures,
        "bbox_profile_apply:no_valid_outputs",
    )
    assert isinstance(calibration_raw_apply, dict)
    require(
        int(calibration_raw_apply.get("valid_sample_count", 0)) > 0,
        failures,
        "calibration_raw_apply:no_valid_samples",
    )
    assert isinstance(calibration_profile_apply, dict)
    require(
        int(calibration_profile_apply.get("valid_profile_output_count", 0)) > 0,
        failures,
        "calibration_profile_apply:no_valid_outputs",
    )

    return {
        "status": "PASS" if not failures else "FAIL",
        "package_dir": str(package_dir),
        "packaged_profile": str(required_files["packaged_profile"]),
        "default_tier": package_summary.get("default_tier", ""),
        "default_selection_scope": default_tier.get("selection_scope", "") if isinstance(default_tier, Mapping) else "",
        "default_adapter_class": default_adapter.get("class", "") if isinstance(default_adapter, Mapping) else "",
        "default_evidence_status": default_evidence.get("status", "") if isinstance(default_evidence, Mapping) else "",
        "default_evidence_pass": (
            f"{default_evidence.get('pass_count')}/{default_evidence.get('trial_count')}"
            if isinstance(default_evidence, Mapping)
            else ""
        ),
        "manifest_audit": f"{manifest_audit.get('pass_count')}/{manifest_audit.get('tier_count')}",
        "bbox_conversion_status": bbox_conversion.get("status", ""),
        "bbox_output_schema_validation_status": bbox_output_schema.get("status", ""),
        "calibration_extract_status": calibration_extract.get("status", ""),
        "calibration_output_schema_validation_status": calibration_output_schema.get("status", ""),
        "path_equivalence_status": path_equivalence.get("status", ""),
        "public_topic_payload_status": public_topic_payload.get("status", ""),
        "public_topic_payload_validation_status": public_topic_payload_validation.get("status", ""),
        "public_topic_payload_source_equivalence_status": public_topic_payload_source_equivalence.get("status", ""),
        "public_topic_payload_publish_dry_run_status": public_topic_payload_publish.get("status", ""),
        "input_schema_validation_status": input_schema.get("status", ""),
        "output_schema_validation_status": output_schema.get("status", ""),
        "operator_view_fusion_launch_status": operator_view_fusion_launch.get("status", ""),
        "operator_view_fusion_manifest_status": operator_view_fusion_launch.get(
            "observation_fusion_manifest_status", ""
        ),
        "operator_view_fusion_tier_override_launch_status": operator_view_fusion_tier_override_launch.get("status", ""),
        "operator_view_fusion_tier_override": operator_view_fusion_tier_override_launch.get(
            "observation_fusion_tier", ""
        ),
        "operator_view_package_launch_status": operator_view_package_launch.get("status", ""),
        "stereodistance_node_smoke_status": stereodistance_node_smoke.get("status", ""),
        "stereodistance_node_smoke_fusion_status": stereodistance_node_smoke.get("observation_fusion_status", ""),
        "disparity_residual_profile_status": residual_profile_status,
        "disparity_residual_profile": str(residual_profile_path) if residual_profile_path.is_file() else "",
        "disparity_residual_profile_id": residual_profile_id,
        "stereodistance_node_smoke_disparity_residual_status": residual_smoke_status,
        "tier_override_smoke_status": tier_override_smoke.get("status", ""),
        "tier_override_smoke_pass": f"{tier_override_smoke.get('pass_count')}/{tier_override_smoke.get('tier_count')}",
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def write_text(path: Path, result: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion deployment package acceptance check",
        f"package_dir: {result['package_dir']}",
        f"status: {result['status']}",
        f"packaged_profile: {result.get('packaged_profile', '')}",
        f"default_tier: {result.get('default_tier', '')}",
        f"default_selection_scope: {result.get('default_selection_scope', '')}",
        f"default_adapter_class: {result.get('default_adapter_class', '')}",
        f"default_evidence_status: {result.get('default_evidence_status', '')}",
        f"default_evidence_pass: {result.get('default_evidence_pass', '')}",
        f"manifest_audit: {result.get('manifest_audit', '')}",
        f"bbox_conversion_status: {result.get('bbox_conversion_status', '')}",
        f"bbox_output_schema_validation_status: {result.get('bbox_output_schema_validation_status', '')}",
        f"calibration_extract_status: {result.get('calibration_extract_status', '')}",
        f"calibration_output_schema_validation_status: {result.get('calibration_output_schema_validation_status', '')}",
        f"path_equivalence_status: {result.get('path_equivalence_status', '')}",
        f"public_topic_payload_status: {result.get('public_topic_payload_status', '')}",
        f"public_topic_payload_validation_status: {result.get('public_topic_payload_validation_status', '')}",
        f"public_topic_payload_source_equivalence_status: {result.get('public_topic_payload_source_equivalence_status', '')}",
        f"public_topic_payload_publish_dry_run_status: {result.get('public_topic_payload_publish_dry_run_status', '')}",
        f"input_schema_validation_status: {result.get('input_schema_validation_status', '')}",
        f"output_schema_validation_status: {result.get('output_schema_validation_status', '')}",
        f"operator_view_fusion_launch_status: {result.get('operator_view_fusion_launch_status', '')}",
        f"operator_view_fusion_manifest_status: {result.get('operator_view_fusion_manifest_status', '')}",
        f"operator_view_fusion_tier_override_launch_status: {result.get('operator_view_fusion_tier_override_launch_status', '')}",
        f"operator_view_fusion_tier_override: {result.get('operator_view_fusion_tier_override', '')}",
        f"operator_view_package_launch_status: {result.get('operator_view_package_launch_status', '')}",
        f"stereodistance_node_smoke_status: {result.get('stereodistance_node_smoke_status', '')}",
        f"stereodistance_node_smoke_fusion_status: {result.get('stereodistance_node_smoke_fusion_status', '')}",
        f"disparity_residual_profile_status: {result.get('disparity_residual_profile_status', '')}",
        f"disparity_residual_profile: {result.get('disparity_residual_profile', '')}",
        f"disparity_residual_profile_id: {result.get('disparity_residual_profile_id', '')}",
        "stereodistance_node_smoke_disparity_residual_status: "
        f"{result.get('stereodistance_node_smoke_disparity_residual_status', '')}",
        f"tier_override_smoke_status: {result.get('tier_override_smoke_status', '')}",
        f"tier_override_smoke_pass: {result.get('tier_override_smoke_pass', '')}",
        f"failure_count: {result['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: topic truth offline-only; runtime image u/v/disparity; internal RDF; public FRD+range.",
    ]
    for failure in result.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check observation-fusion deployment package acceptance gates.")
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = check_package(args.package_dir)
    output_dir = args.output_dir if args.output_dir is not None else args.package_dir / "acceptance_check"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "observation_fusion_deployment_package_acceptance.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(output_dir / "observation_fusion_deployment_package_acceptance.txt", result)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if result["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
