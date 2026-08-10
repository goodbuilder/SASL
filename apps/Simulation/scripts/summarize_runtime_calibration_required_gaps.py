#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize runtime guard rows that look like calibration-required intrinsic-scale regimes. "
            "This does not change profile selection; it records when runtime raw features indicate a "
            "regime whose matching source profile is unavailable."
        )
    )
    parser.add_argument("--runtime-audit-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--product-ratio-p10-threshold", type=float, default=1.0037)
    parser.add_argument("--required-source-scenario", default="intrinsic_scale_stress")
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def parse_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def selected_source_available(row: Mapping[str, str], required_source: str) -> bool:
    eligible = {item.strip() for item in str(row.get("eligible_source_scenarios", "")).split(",") if item.strip()}
    return required_source in eligible


def summarize_rows(
    rows: Sequence[Mapping[str, str]],
    *,
    product_ratio_p10_threshold: float,
    required_source_scenario: str,
    target_error_limit_m: float,
) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for row in rows:
        product_ratio_p10 = finite_float(row.get("feature_disparity_forward_product_ratio_p10_to_nominal"))
        detected = math.isfinite(product_ratio_p10) and product_ratio_p10 >= product_ratio_p10_threshold
        required_available = selected_source_available(row, required_source_scenario)
        selected = str(row.get("selected_source_scenario", ""))
        selected_required = selected == required_source_scenario
        validation_improvement = finite_float(row.get("validation_range_rmse_improvement_m"))
        target_max = finite_float(row.get("target_max_abs_error_corrected_m"))
        strict_pass = parse_bool(row.get("pass_runtime_measurement_guard_replay"))
        validation_only_pass = parse_bool(row.get("validation_only_pass_runtime_measurement_guard_replay"))
        calibration_required = detected and not required_available
        unsafe_correction = calibration_required and math.isfinite(validation_improvement) and validation_improvement <= 0.0
        target_within_limit = math.isfinite(target_max) and target_max <= target_error_limit_m
        output.append(
            {
                "target_scenario": row.get("target_scenario", ""),
                "target_seed": row.get("target_seed", ""),
                "selected_source_scenario": selected,
                "required_source_scenario": required_source_scenario,
                "required_source_available": required_available,
                "selected_required_source": selected_required,
                "product_ratio_p10_to_nominal": product_ratio_p10,
                "product_ratio_p10_threshold": product_ratio_p10_threshold,
                "calibration_required_detected": calibration_required,
                "unsafe_correction_when_required_unavailable": unsafe_correction,
                "strict_pass": strict_pass,
                "validation_only_pass": validation_only_pass,
                "validation_range_rmse_corrected_m": row.get("validation_range_rmse_corrected_m", ""),
                "validation_range_rmse_improvement_m": row.get("validation_range_rmse_improvement_m", ""),
                "target_max_abs_error_corrected_m": row.get("target_max_abs_error_corrected_m", ""),
                "target_within_limit": target_within_limit,
                "fallback_reason": row.get("fallback_reason", ""),
                "pass_reason": row.get("pass_reason", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    return output


def aggregate(rows: Sequence[Mapping[str, object]], *, audit_csv: Path) -> dict[str, object]:
    calibration_required = [row for row in rows if bool(row.get("calibration_required_detected"))]
    unsafe = [row for row in rows if bool(row.get("unsafe_correction_when_required_unavailable"))]
    target_ok = [row for row in calibration_required if bool(row.get("target_within_limit"))]
    strict_pass = [row for row in calibration_required if bool(row.get("strict_pass"))]
    validation_only_pass = [row for row in calibration_required if bool(row.get("validation_only_pass"))]
    scenarios = sorted({str(row.get("target_scenario", "")) for row in calibration_required})
    return {
        "mode": "runtime_calibration_required_gap_summary",
        "runtime_audit_csv": str(audit_csv),
        "audit_count": len(rows),
        "calibration_required_count": len(calibration_required),
        "calibration_required_scenarios": scenarios,
        "unsafe_correction_count": len(unsafe),
        "calibration_required_target_within_limit_count": len(target_ok),
        "calibration_required_strict_pass_count": len(strict_pass),
        "calibration_required_validation_only_pass_count": len(validation_only_pass),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Calibration-required detection uses runtime raw RDF/disparity-derived product-ratio features. "
            "Truth-derived replay metrics are used only to audit whether the selected profile was safe."
        ),
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime calibration-required gap summary",
        "",
        f"runtime_audit_csv: {summary.get('runtime_audit_csv', '')}",
        f"audit_count: {summary.get('audit_count', '')}",
        f"calibration_required_count: {summary.get('calibration_required_count', '')}",
        f"calibration_required_scenarios: {summary.get('calibration_required_scenarios', '')}",
        f"unsafe_correction_count: {summary.get('unsafe_correction_count', '')}",
        f"calibration_required_target_within_limit_count: {summary.get('calibration_required_target_within_limit_count', '')}",
        f"calibration_required_strict_pass_count: {summary.get('calibration_required_strict_pass_count', '')}",
        f"calibration_required_validation_only_pass_count: {summary.get('calibration_required_validation_only_pass_count', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Calibration-required rows:",
    ]
    for row in rows:
        if not bool(row.get("calibration_required_detected")):
            continue
        lines.append(
            "- {target}/{seed}: selected={selected}, unsafe={unsafe}, strict={strict}, "
            "validation_only={validation_only}, improvement={improvement}, target_max={target_max}, reason={reason}".format(
                target=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                selected=row.get("selected_source_scenario", ""),
                unsafe=row.get("unsafe_correction_when_required_unavailable", ""),
                strict=row.get("strict_pass", ""),
                validation_only=row.get("validation_only_pass", ""),
                improvement=row.get("validation_range_rmse_improvement_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                reason=row.get("pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_calibration_required_gaps(
    runtime_audit_csv: Path,
    output_dir: Path,
    *,
    product_ratio_p10_threshold: float,
    required_source_scenario: str,
    target_error_limit_m: float,
) -> dict[str, object]:
    rows = summarize_rows(
        read_csv(runtime_audit_csv),
        product_ratio_p10_threshold=product_ratio_p10_threshold,
        required_source_scenario=required_source_scenario,
        target_error_limit_m=target_error_limit_m,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "runtime_calibration_required_gap_rows.csv", rows)
    summary = aggregate(rows, audit_csv=runtime_audit_csv)
    write_json(output_dir / "runtime_calibration_required_gap_summary.json", summary)
    write_summary_txt(output_dir / "runtime_calibration_required_gap_summary.txt", summary, rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = summarize_calibration_required_gaps(
        args.runtime_audit_csv,
        args.output_dir,
        product_ratio_p10_threshold=args.product_ratio_p10_threshold,
        required_source_scenario=args.required_source_scenario,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
