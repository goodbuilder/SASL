#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Mapping


CORE_ARTIFACTS = [
    "method_comparison.csv",
    "validation_predictions.csv",
    "target_1852m_stats.csv",
    "pareto_frontier.csv",
    "pareto_frontier.json",
    "policy_recommendations.csv",
    "policy_recommendations.json",
    "method_parameters.json",
    "run_config.json",
    "summary.txt",
    "range_error_before_after.png",
    "residual_disparity_by_distance.png",
    "rdf_axis_rmse.png",
    "pareto_frontier.png",
    "metrics.json",
    "promotion_summary.txt",
]

RUNTIME_ARTIFACTS = [
    "sim3_runtime_profile.json",
    "runtime_profiles/manifest.json",
    "policy_profiles/manifest.json",
    "profile_replay/profile_replay_summary.csv",
    "profile_replay/profile_replay_predictions.csv",
    "profile_replay/profile_replay_range_error.png",
    "profile_replay/profile_replay_summary.txt",
    "promotion_check/promotion_check.json",
    "promotion_check/promotion_check.txt",
    "promotion_check/promotion_check_samples.csv",
]

CSV_PREFLIGHT_ARTIFACTS = [
    "preflight/preflight_summary.json",
    "preflight/preflight_rows.csv",
    "preflight/preflight_summary.txt",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit an estimated-to-truth calibration result directory for promotion readiness. "
            "Checks required artifacts, gate metrics, runtime profile promotion check, and coordinate contracts."
        )
    )
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    parser.add_argument("--require-csv-preflight", action="store_true")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def artifact_status(result_dir: Path, relative_paths: list[str]) -> dict[str, bool]:
    return {path: (result_dir / path).is_file() for path in relative_paths}


def all_true(status: Mapping[str, bool]) -> bool:
    return all(bool(value) for value in status.values())


def check_metrics(metrics: Mapping[str, Any]) -> dict[str, Any]:
    gate_result = metrics.get("gate_result") if isinstance(metrics.get("gate_result"), dict) else {}
    validation_range_raw = finite_float(metrics.get("validation_range_rmse_raw_m"))
    validation_range_corrected = finite_float(metrics.get("validation_range_rmse_corrected_m"))
    validation_position_raw = finite_float(metrics.get("validation_position_rmse_raw_m"))
    validation_position_corrected = finite_float(metrics.get("validation_position_rmse_corrected_m"))
    target_error = finite_float(metrics.get("target_1852m_abs_mean_error_m"))
    target_tolerance = finite_float(metrics.get("target_1852m_tolerance_m")) or 10.0
    target_count = int(metrics.get("target_1852m_sample_count") or 0)
    target_min = int(metrics.get("target_1852m_min_sample_count") or 3)
    promotion_check = metrics.get("promotion_check") if isinstance(metrics.get("promotion_check"), dict) else {}
    preflight = metrics.get("preflight") if isinstance(metrics.get("preflight"), dict) else {}
    return {
        "status_strict_pass": metrics.get("status") == "STRICT_PASS",
        "gate_strict_pass": gate_result.get("verdict") == "STRICT_PASS",
        "validation_range_improved": (
            validation_range_raw is not None
            and validation_range_corrected is not None
            and validation_range_corrected < validation_range_raw
        ),
        "validation_position_improved": (
            validation_position_raw is not None
            and validation_position_corrected is not None
            and validation_position_corrected < validation_position_raw
        ),
        "target_error_within_tolerance": target_error is not None and target_error <= target_tolerance,
        "target_sample_count_ok": target_count >= target_min,
        "runtime_truth_not_used": not bool(metrics.get("runtime_truth_used")),
        "coordinate_contracts_explicit": bool(metrics.get("coordinate_contracts_explicit")),
        "alias_contracts_explicit": bool(metrics.get("alias_contracts_explicit")),
        "runtime_profile_exportable": bool(metrics.get("runtime_profile_exportable")),
        "stereodistance_applicable": bool(metrics.get("stereodistance_applicable")),
        "promotion_check_passed": bool(promotion_check.get("pass_promotion_check")),
        "preflight_passed_or_not_required": not preflight or bool(preflight.get("pass_preflight")),
    }


def audit_result_dir(result_dir: Path, *, require_csv_preflight: bool = False) -> dict[str, Any]:
    core = artifact_status(result_dir, CORE_ARTIFACTS)
    runtime = artifact_status(result_dir, RUNTIME_ARTIFACTS)
    preflight = artifact_status(result_dir, CSV_PREFLIGHT_ARTIFACTS)
    metrics_path = result_dir / "metrics.json"
    metrics: dict[str, Any] = load_json(metrics_path) if metrics_path.is_file() else {}
    metrics_checks = check_metrics(metrics) if metrics else {}
    scenario = str(metrics.get("scenario", ""))
    needs_preflight = require_csv_preflight or scenario.startswith("recorded_calibration_csv")
    pass_audit = (
        all_true(core)
        and all_true(runtime)
        and (all_true(preflight) if needs_preflight else True)
        and all_true(metrics_checks)
    )
    missing = [
        path
        for group in (core, runtime, preflight if needs_preflight else {})
        for path, exists in group.items()
        if not exists
    ]
    failed_checks = [name for name, passed in metrics_checks.items() if not passed]
    return {
        "pass_audit": pass_audit,
        "result_dir": str(result_dir),
        "scenario": scenario,
        "requires_preflight": needs_preflight,
        "missing_artifacts": missing,
        "failed_metric_checks": failed_checks,
        "core_artifacts": core,
        "runtime_artifacts": runtime,
        "preflight_artifacts": preflight,
        "metrics_checks": metrics_checks,
        "method": metrics.get("method", ""),
        "gate_verdict": (metrics.get("gate_result") or {}).get("verdict", ""),
        "validation_range_rmse_raw_m": metrics.get("validation_range_rmse_raw_m"),
        "validation_range_rmse_corrected_m": metrics.get("validation_range_rmse_corrected_m"),
        "target_1852m_abs_mean_error_m": metrics.get("target_1852m_abs_mean_error_m"),
    }


def write_text(path: Path, audit: Mapping[str, Any]) -> None:
    lines = ["Estimated-to-Truth result audit", ""]
    lines.append(f"pass_audit: {audit['pass_audit']}")
    lines.append(f"result_dir: {audit['result_dir']}")
    lines.append(f"scenario: {audit['scenario']}")
    lines.append(f"gate_verdict: {audit['gate_verdict']}")
    lines.append(f"method: {audit['method']}")
    lines.append(f"validation_range_rmse_raw_m: {audit['validation_range_rmse_raw_m']}")
    lines.append(f"validation_range_rmse_corrected_m: {audit['validation_range_rmse_corrected_m']}")
    lines.append(f"target_1852m_abs_mean_error_m: {audit['target_1852m_abs_mean_error_m']}")
    lines.append("")
    lines.append("missing_artifacts:")
    for item in audit["missing_artifacts"]:
        lines.append(f"- {item}")
    lines.append("")
    lines.append("failed_metric_checks:")
    for item in audit["failed_metric_checks"]:
        lines.append(f"- {item}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    audit = audit_result_dir(args.result_dir, require_csv_preflight=args.require_csv_preflight)
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    output_json = args.output_json or args.result_dir / "result_audit.json"
    output_txt = args.output_txt or args.result_dir / "result_audit.txt"
    output_json.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_text(output_txt, audit)
    return 0 if audit["pass_audit"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
