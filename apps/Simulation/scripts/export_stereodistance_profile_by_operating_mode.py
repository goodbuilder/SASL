#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    parse_split_list,
    select_by_calibration_replay,
    write_json,
)
from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    build_feature_library,
    catalog_profile_path,
    default_transfer_matrix_csv,
    load_transfer_matrix,
    read_csv,
    runtime_measurement_features,
    select_source_by_runtime_measurement,
    source_scenarios_for_mode,
)
from select_stereodistance_profile_from_catalog import load_catalog  # noqa: E402
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


TRUTH_FIELDS = {
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "gt_right_m",
    "gt_down_m",
    "gt_forward_m",
}

RAW_RUNTIME_FIELDS = {
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "measured_disparity_px",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export one active StereoDistance runtime profile by operating mode. "
            "Use calibration replay when calibration-time truth is available; otherwise use the "
            "truth-free runtime measurement guard."
        )
    )
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sweep-dir", type=Path)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument(
        "--mode",
        choices=("auto", "calibration_replay", "runtime_guard"),
        default="auto",
        help="auto chooses calibration_replay when truth fields exist, otherwise runtime_guard.",
    )
    parser.add_argument("--split", default="validation")
    parser.add_argument("--stability-splits", default="train,validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--calibration-candidate-source-mode",
        choices=("minimal_set", "all_catalog"),
        default="all_catalog",
    )
    parser.add_argument(
        "--runtime-candidate-source-mode",
        choices=("minimal_set", "all_catalog"),
        default="all_catalog",
    )
    parser.add_argument(
        "--runtime-selection-policy",
        choices=(
            "nearest_centroid",
            "robust_family_fallback",
            "transfer_risk_fallback",
            "intrinsic_scale_gap_transfer_risk",
            "feature_vote_transfer_risk",
            "knn_transfer_risk",
            "centroid_knn_union_transfer_risk",
            "centroid_knn_veto_transfer_risk",
        ),
        default="transfer_risk_fallback",
    )
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--knn-neighbor-count", type=int, default=5)
    parser.add_argument("--intrinsic-scale-gap-source-scenario", default="intrinsic_scale_stress")
    parser.add_argument("--intrinsic-scale-gap-product-ratio-p10-threshold", type=float, default=1.0037)
    parser.add_argument(
        "--block-calibration-required-runtime-export",
        action="store_true",
        help=(
            "In runtime_guard mode, do not export a fallback profile when raw feature diagnostics indicate "
            "an intrinsic-scale calibration-required regime and the required source profile is unavailable."
        ),
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def csv_fieldnames(path: Path) -> set[str]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return set(reader.fieldnames or [])


def has_truth_fields(path: Path) -> bool:
    return bool(csv_fieldnames(path) & TRUTH_FIELDS)


def has_runtime_fields(path: Path) -> bool:
    return RAW_RUNTIME_FIELDS.issubset(csv_fieldnames(path))


def resolve_mode(requested: str, input_csv: Path) -> str:
    if requested != "auto":
        return requested
    return "calibration_replay" if has_truth_fields(input_csv) else "runtime_guard"


def write_summary_txt(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance active profile export by operating mode",
        "",
        f"selected_operating_mode: {summary.get('selected_operating_mode', '')}",
        f"selected: {summary.get('selected', '')}",
        f"output_profile: {summary.get('output_profile', '')}",
        f"input_csv: {summary.get('input_csv', '')}",
        f"calibration_required_detected: {summary.get('calibration_required_detected', '')}",
        f"calibration_required_blocked_export: {summary.get('calibration_required_blocked_export', '')}",
        f"calibration_required_block_reason: {summary.get('calibration_required_block_reason', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Metrics:",
        f"validation_range_rmse_corrected_m: {summary.get('validation_range_rmse_corrected_m', '')}",
        f"target_abs_mean_error_corrected_m: {summary.get('target_abs_mean_error_corrected_m', '')}",
        f"target_max_abs_error_corrected_m: {summary.get('target_max_abs_error_corrected_m', '')}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def select_runtime_guard_profile_without_truth(
    *,
    sweep_dir: Path,
    catalog_json: Path,
    input_csv: Path,
    output_profile: Path,
    output_dir: Path,
    minimal_set_json: Path | None,
    candidate_source_mode: str,
    selection_policy: str,
    transfer_matrix_csv: Path | None,
    transfer_low_risk_rmse_limit_m: float,
    transfer_low_risk_target_limit_m: float,
    transfer_risk_target_weight: float,
    knn_neighbor_count: int,
    intrinsic_scale_gap_source_scenario: str,
    intrinsic_scale_gap_product_ratio_p10_threshold: float,
    block_calibration_required_export: bool = False,
) -> dict[str, object]:
    catalog = load_catalog(catalog_json)
    source_scenarios = source_scenarios_for_mode(
        catalog=catalog,
        catalog_json=catalog_json,
        minimal_set_json=minimal_set_json,
        candidate_source_mode=candidate_source_mode,
    )
    matrix_path = transfer_matrix_csv or default_transfer_matrix_csv(sweep_dir)
    transfer_matrix = load_transfer_matrix(matrix_path)
    rows = read_csv(input_csv)
    library = build_feature_library(sweep_dir=sweep_dir, source_scenarios=source_scenarios)
    features = runtime_measurement_features(rows)
    selection = select_source_by_runtime_measurement(
        rows=rows,
        library=library,
        available_source_scenarios=source_scenarios,
        selection_policy=selection_policy,
        transfer_matrix=transfer_matrix,
        transfer_low_risk_rmse_limit_m=transfer_low_risk_rmse_limit_m,
        transfer_low_risk_target_limit_m=transfer_low_risk_target_limit_m,
        transfer_risk_target_weight=transfer_risk_target_weight,
        knn_neighbor_count=knn_neighbor_count,
        intrinsic_scale_gap_source_scenario=intrinsic_scale_gap_source_scenario,
        intrinsic_scale_gap_product_ratio_p10_threshold=intrinsic_scale_gap_product_ratio_p10_threshold,
    )
    selected = str(selection["selected_source_scenario"])
    source_profile = catalog_profile_path(catalog_json, catalog, selected)
    product_ratio_p10 = float(features.get("disparity_forward_product_ratio_p10_to_nominal", float("nan")))
    required_source_available = intrinsic_scale_gap_source_scenario in set(source_scenarios)
    calibration_required = (
        product_ratio_p10 >= intrinsic_scale_gap_product_ratio_p10_threshold
        and not required_source_available
    )
    block_export = bool(block_calibration_required_export and calibration_required)
    if not block_export:
        output_profile.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_profile, output_profile)
    summary = {
        "selected": not block_export,
        "selection_mode": "runtime_measurement_guard_selection_only",
        "scenario": selected,
        "preliminary_source_scenario": selection.get("preliminary_source_scenario", ""),
        "selection_policy": selection_policy,
        "fallback_reason": selection.get("fallback_reason", ""),
        "calibration_required_detected": calibration_required,
        "calibration_required_blocked_export": block_export,
        "calibration_required_block_reason": (
            "intrinsic_scale_gap_detected_required_profile_unavailable"
            if block_export
            else ""
        ),
        "required_source_scenario": intrinsic_scale_gap_source_scenario,
        "required_source_available": required_source_available,
        "feature_disparity_forward_product_ratio_p10_to_nominal": product_ratio_p10,
        "source_profile": str(source_profile),
        "output_profile": "" if block_export else str(output_profile),
        "candidate_source_mode": candidate_source_mode,
        "candidate_source_scenarios": ",".join(source_scenarios),
        "transfer_matrix_csv": str(matrix_path),
        "transfer_low_risk_rmse_limit_m": transfer_low_risk_rmse_limit_m,
        "transfer_low_risk_target_limit_m": transfer_low_risk_target_limit_m,
        "transfer_risk_target_weight": transfer_risk_target_weight,
        "knn_neighbor_count": knn_neighbor_count,
        "intrinsic_scale_gap_source_scenario": intrinsic_scale_gap_source_scenario,
        "intrinsic_scale_gap_product_ratio_p10_threshold": intrinsic_scale_gap_product_ratio_p10_threshold,
        "distance_rows_json": json.dumps(selection.get("distance_rows", []), ensure_ascii=False, allow_nan=False),
        "transfer_confusion_cluster": ",".join(str(item) for item in selection.get("transfer_confusion_cluster", [])),
        "transfer_score_rows_json": json.dumps(
            selection.get("transfer_score_rows", []), ensure_ascii=False, allow_nan=False
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "runtime guard selection uses raw RDF/disparity only; no truth replay is required. "
            "When block_calibration_required_export is enabled, raw feature diagnostics may prevent "
            "fallback profile export for calibration-required regimes."
        ),
        "runtime_input_contract": (
            "exported active profile consumes raw stereo-center RDF/disparity only; blocked exports produce no active profile"
        ),
    }
    write_json(output_dir / "runtime_guard_selection_only_summary.json", summary)
    return summary


def export_by_operating_mode(
    *,
    catalog_json: Path,
    input_csv: Path,
    output_profile: Path,
    output_dir: Path,
    sweep_dir: Path | None = None,
    minimal_set_json: Path | None = None,
    mode: str = "auto",
    split: str = "validation",
    stability_splits: Sequence[str] = ("train", "validation"),
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    calibration_candidate_source_mode: str = "all_catalog",
    runtime_candidate_source_mode: str = "all_catalog",
    runtime_selection_policy: str = "transfer_risk_fallback",
    transfer_matrix_csv: Path | None = None,
    transfer_low_risk_rmse_limit_m: float = 1.3,
    transfer_low_risk_target_limit_m: float = 2.5,
    transfer_risk_target_weight: float = 0.10,
    knn_neighbor_count: int = 5,
    intrinsic_scale_gap_source_scenario: str = "intrinsic_scale_stress",
    intrinsic_scale_gap_product_ratio_p10_threshold: float = 1.0037,
    block_calibration_required_runtime_export: bool = False,
    allow_unvalidated: bool = False,
) -> dict[str, object]:
    if not input_csv.is_file():
        raise ValueError(f"input CSV does not exist: {input_csv}")
    selected_mode = resolve_mode(mode, input_csv)
    if selected_mode == "runtime_guard" and not has_runtime_fields(input_csv):
        raise ValueError(f"runtime_guard mode requires fields: {','.join(sorted(RAW_RUNTIME_FIELDS))}")
    output_dir.mkdir(parents=True, exist_ok=True)

    if selected_mode == "calibration_replay":
        if not has_truth_fields(input_csv):
            raise ValueError("calibration_replay mode requires calibration-time truth fields")
        selection_summary = select_by_calibration_replay(
            catalog_json=catalog_json,
            input_csv=input_csv,
            output_profile=output_profile,
            output_dir=output_dir / "calibration_replay_export",
            minimal_set_json=minimal_set_json,
            split=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
            allow_unvalidated=allow_unvalidated,
            stability_splits=stability_splits,
            candidate_source_mode=calibration_candidate_source_mode,
        )
        summary: dict[str, object] = {
            "selected_operating_mode": selected_mode,
            "selection_summary": selection_summary,
            "selected_source_scenario": selection_summary.get("scenario", ""),
            "selected_method": selection_summary.get("method", ""),
            "selected": True,
            "calibration_required_detected": False,
            "calibration_required_blocked_export": False,
            "calibration_required_block_reason": "",
            "validation_range_rmse_corrected_m": selection_summary.get("validation_range_rmse_corrected_m", ""),
            "target_abs_mean_error_corrected_m": selection_summary.get("target_abs_mean_error_corrected_m", ""),
            "target_max_abs_error_corrected_m": selection_summary.get("target_max_abs_error_corrected_m", ""),
        }
    elif selected_mode == "runtime_guard":
        resolved_sweep_dir = sweep_dir or catalog_json.parent.parent
        selection_summary = select_runtime_guard_profile_without_truth(
            sweep_dir=resolved_sweep_dir,
            catalog_json=catalog_json,
            input_csv=input_csv,
            output_profile=output_profile,
            output_dir=output_dir / "runtime_guard_export",
            minimal_set_json=minimal_set_json,
            candidate_source_mode=runtime_candidate_source_mode,
            selection_policy=runtime_selection_policy,
            transfer_matrix_csv=transfer_matrix_csv,
            transfer_low_risk_rmse_limit_m=transfer_low_risk_rmse_limit_m,
            transfer_low_risk_target_limit_m=transfer_low_risk_target_limit_m,
            transfer_risk_target_weight=transfer_risk_target_weight,
            knn_neighbor_count=knn_neighbor_count,
            intrinsic_scale_gap_source_scenario=intrinsic_scale_gap_source_scenario,
            intrinsic_scale_gap_product_ratio_p10_threshold=intrinsic_scale_gap_product_ratio_p10_threshold,
            block_calibration_required_export=block_calibration_required_runtime_export,
        )
        summary = {
            "selected_operating_mode": selected_mode,
            "selection_summary": selection_summary,
            "selected_source_scenario": selection_summary.get("scenario", ""),
            "selected_method": "",
            "selected": selection_summary.get("selected", True),
            "calibration_required_detected": selection_summary.get("calibration_required_detected", False),
            "calibration_required_blocked_export": selection_summary.get("calibration_required_blocked_export", False),
            "calibration_required_block_reason": selection_summary.get("calibration_required_block_reason", ""),
            "validation_range_rmse_corrected_m": "",
            "target_abs_mean_error_corrected_m": "",
            "target_max_abs_error_corrected_m": "",
        }
    else:
        raise ValueError(f"unsupported operating mode: {selected_mode}")

    exported_output_profile = str(output_profile)
    runtime_input_contract = "exported active profile consumes raw stereo-center RDF/disparity only"
    if bool(summary.get("calibration_required_blocked_export", False)):
        exported_output_profile = ""
        runtime_input_contract = (
            "blocked export consumes no active profile; runtime guard used raw stereo-center RDF/disparity "
            "diagnostics only"
        )
    summary.update(
        {
            "requested_mode": mode,
            "catalog_json": str(catalog_json),
            "input_csv": str(input_csv),
            "output_profile": exported_output_profile,
            "output_dir": str(output_dir),
            "has_truth_fields": has_truth_fields(input_csv),
            "has_runtime_fields": has_runtime_fields(input_csv),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "selection_contract": (
                "calibration_replay may use calibration-time truth for profile selection; "
                "runtime_guard uses raw RDF/disparity only. Exported profiles never consume runtime truth."
            ),
            "runtime_input_contract": runtime_input_contract,
        }
    )
    write_json(output_dir / "operating_mode_profile_export_summary.json", summary)
    write_summary_txt(output_dir / "operating_mode_profile_export_summary.txt", summary)
    return summary


def main() -> int:
    args = parse_args()
    summary = export_by_operating_mode(
        catalog_json=args.catalog_json,
        input_csv=args.input_csv,
        output_profile=args.output_profile,
        output_dir=args.output_dir,
        sweep_dir=args.sweep_dir,
        minimal_set_json=args.minimal_set_json,
        mode=args.mode,
        split=args.split,
        stability_splits=parse_split_list(args.stability_splits),
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        calibration_candidate_source_mode=args.calibration_candidate_source_mode,
        runtime_candidate_source_mode=args.runtime_candidate_source_mode,
        runtime_selection_policy=args.runtime_selection_policy,
        transfer_matrix_csv=args.transfer_matrix_csv,
        transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
        transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
        transfer_risk_target_weight=args.transfer_risk_target_weight,
        knn_neighbor_count=args.knn_neighbor_count,
        intrinsic_scale_gap_source_scenario=args.intrinsic_scale_gap_source_scenario,
        intrinsic_scale_gap_product_ratio_p10_threshold=args.intrinsic_scale_gap_product_ratio_p10_threshold,
        block_calibration_required_runtime_export=args.block_calibration_required_runtime_export,
        allow_unvalidated=args.allow_unvalidated,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
