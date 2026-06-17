#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Mapping


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
RUNTIME_FEATURE_FIELDS = (
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "measured_disparity_px",
    "left_u_px",
    "left_v_px",
)
FORBIDDEN_RUNTIME_FIELDS = (
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "validation_range_rmse_corrected_m",
    "target_max_abs_error_corrected_m",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a StereoDistance runtime deployment package from the known/unknown deployment "
            "strategy recommendation. The package contains a directly usable single pooled active "
            "profile plus a manifest for the known-catalog guard with conservative override."
        )
    )
    parser.add_argument("--strategy-recommendation-json", type=Path, required=True)
    parser.add_argument("--single-pooled-profile-json", type=Path, required=True)
    parser.add_argument("--scenario-catalog-json", type=Path, required=True)
    parser.add_argument("--transfer-matrix-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--known-selector-policy", default="intrinsic_scale_gap_transfer_risk")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def write_txt(path: Path, manifest: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance runtime deployment package",
        "",
        f"package_mode: {manifest.get('package_mode', '')}",
        f"known_catalog_recommendation: {manifest.get('known_catalog_recommendation', '')}",
        f"known_catalog_runtime_rule: {manifest.get('known_catalog_runtime_rule', '')}",
        f"unknown_regime_recommendation: {manifest.get('unknown_regime_recommendation', '')}",
        f"unknown_active_profile: {manifest.get('unknown_active_profile', '')}",
        f"single_pooled_profile: {manifest.get('single_pooled_profile', '')}",
        f"scenario_catalog_json: {manifest.get('scenario_catalog_json', '')}",
        f"transfer_matrix_csv: {manifest.get('transfer_matrix_csv', '')}",
        f"target_range_m: {manifest.get('target_range_m', '')}",
        f"target_error_limit_m: {manifest.get('target_error_limit_m', '')}",
        f"axis_contract: {manifest.get('axis_contract', '')}",
        f"public_topic_contract: {manifest.get('public_topic_contract', '')}",
        f"selection_contract: {manifest.get('selection_contract', '')}",
        "",
        "Runtime fields:",
        f"selection_input_fields: {','.join(manifest.get('selection_input_fields', []))}",
        f"forbidden_runtime_fields: {','.join(manifest.get('forbidden_runtime_fields', []))}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def relative_to_output(path: Path, output_dir: Path) -> str:
    try:
        return str(path.relative_to(output_dir))
    except ValueError:
        return str(path)


def export_runtime_deployment_package(
    *,
    strategy_recommendation_json: Path,
    single_pooled_profile_json: Path,
    scenario_catalog_json: Path,
    transfer_matrix_csv: Path,
    output_dir: Path,
    known_selector_policy: str = "intrinsic_scale_gap_transfer_risk",
    target_range_m: float = 1852.0,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    recommendation = read_json(strategy_recommendation_json)
    output_dir.mkdir(parents=True, exist_ok=True)
    profiles_dir = output_dir / "profiles"
    catalogs_dir = output_dir / "catalogs"
    profiles_dir.mkdir(parents=True, exist_ok=True)
    catalogs_dir.mkdir(parents=True, exist_ok=True)

    single_profile_out = profiles_dir / "single_pooled_runtime_profile.json"
    unknown_active_profile = output_dir / "active_runtime_profile_unknown.json"
    scenario_catalog_out = catalogs_dir / "scenario_profile_catalog.json"
    transfer_matrix_out = catalogs_dir / "scenario_profile_catalog_transfer_matrix.csv"
    shutil.copy2(single_pooled_profile_json, single_profile_out)
    shutil.copy2(single_pooled_profile_json, unknown_active_profile)
    shutil.copy2(scenario_catalog_json, scenario_catalog_out)
    shutil.copy2(transfer_matrix_csv, transfer_matrix_out)

    known_selector = {
        "mode": "stereodistance_known_catalog_runtime_selector",
        "known_selector_policy": known_selector_policy,
        "baseline_selector": "catalog_guard",
        "scenario_catalog_json": relative_to_output(scenario_catalog_out, output_dir),
        "transfer_matrix_csv": relative_to_output(transfer_matrix_out, output_dir),
        "override_profile_label": "single_pooled",
        "override_profile_json": relative_to_output(single_profile_out, output_dir),
        "runtime_rule": recommendation.get("known_catalog_runtime_rule", ""),
        "known_catalog_recommendation": recommendation.get("known_catalog_recommendation", ""),
        "target_range_m": target_range_m,
        "target_error_limit_m": target_error_limit_m,
        "selection_input_fields": list(RUNTIME_FEATURE_FIELDS),
        "forbidden_runtime_fields": list(FORBIDDEN_RUNTIME_FIELDS),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Known-catalog runtime selection may use raw RDF/disparity/image-center features and fixed "
            "deployment metadata only. PX4/RTK truth and replay errors are forbidden at runtime."
        ),
    }
    write_json(output_dir / "known_catalog_runtime_selector.json", known_selector)

    manifest = {
        "package_mode": "stereodistance_runtime_deployment_package",
        "strategy_recommendation_json": str(strategy_recommendation_json),
        "known_catalog_recommendation": recommendation.get("known_catalog_recommendation", ""),
        "known_catalog_runtime_rule": recommendation.get("known_catalog_runtime_rule", ""),
        "known_catalog_target_max_abs_error_max_m": recommendation.get("known_catalog_target_max_abs_error_max_m", ""),
        "unknown_regime_recommendation": recommendation.get("unknown_regime_recommendation", ""),
        "unknown_regime_runtime_rule": recommendation.get("unknown_regime_runtime_rule", ""),
        "unknown_regime_target_max_abs_error_max_m": recommendation.get("unknown_regime_target_max_abs_error_max_m", ""),
        "single_pooled_profile": relative_to_output(single_profile_out, output_dir),
        "unknown_active_profile": relative_to_output(unknown_active_profile, output_dir),
        "known_selector_manifest": "known_catalog_runtime_selector.json",
        "scenario_catalog_json": relative_to_output(scenario_catalog_out, output_dir),
        "transfer_matrix_csv": relative_to_output(transfer_matrix_out, output_dir),
        "target_range_m": target_range_m,
        "target_error_limit_m": target_error_limit_m,
        "selection_input_fields": list(RUNTIME_FEATURE_FIELDS),
        "forbidden_runtime_fields": list(FORBIDDEN_RUNTIME_FIELDS),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Use active_runtime_profile_unknown.json directly when the deployment regime is unknown. "
            "When a known scenario catalog is available, use known_catalog_runtime_selector.json: its "
            "baseline guard and override rule consume raw feature fields only."
        ),
    }
    write_json(output_dir / "runtime_deployment_package_manifest.json", manifest)
    write_txt(output_dir / "runtime_deployment_package_manifest.txt", manifest)
    return manifest


def main() -> int:
    args = parse_args()
    manifest = export_runtime_deployment_package(
        strategy_recommendation_json=args.strategy_recommendation_json,
        single_pooled_profile_json=args.single_pooled_profile_json,
        scenario_catalog_json=args.scenario_catalog_json,
        transfer_matrix_csv=args.transfer_matrix_csv,
        output_dir=args.output_dir,
        known_selector_policy=args.known_selector_policy,
        target_range_m=args.target_range_m,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
