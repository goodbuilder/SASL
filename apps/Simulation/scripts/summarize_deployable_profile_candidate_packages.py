#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize deployable StereoDistance candidate packages.")
    parser.add_argument("--package-dir", action="append", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def package_row(package_dir: Path) -> dict[str, object]:
    manifest = load_json(package_dir / "deployable_profile_candidate_package_manifest.json")
    package_check_path = package_dir / "deployable_profile_candidate_package_check.json"
    operator_check_path = package_dir / "operator_view_launch_check.json"
    portability_check_path = package_dir / "deployable_profile_candidate_package_portability.json"
    package_check = load_json(package_check_path) if package_check_path.is_file() else {}
    operator_check = load_json(operator_check_path) if operator_check_path.is_file() else {}
    portability_check = load_json(portability_check_path) if portability_check_path.is_file() else {}
    portability_required = portability_check_path.is_file()
    status = (
        "PASS"
        if manifest.get("status") == "PASS"
        and package_check.get("status") == "PASS"
        and operator_check.get("status") == "PASS"
        and (not portability_required or portability_check.get("status") == "PASS")
        else "FAIL"
    )
    return {
        "status": status,
        "recommended_policy": manifest.get("recommended_policy", ""),
        "method": manifest.get("method", ""),
        "package_dir": str(package_dir),
        "active_profile_sha256": manifest.get("active_profile_sha256", ""),
        "validation_range_rmse_corrected_mean_m": manifest.get("validation_range_rmse_corrected_mean_m", ""),
        "validation_range_rmse_corrected_max_m": manifest.get("validation_range_rmse_corrected_max_m", ""),
        "target_max_abs_error_corrected_max_m": manifest.get("target_max_abs_error_corrected_max_m", ""),
        "candidate_pass_count": manifest.get("candidate_pass_count", ""),
        "candidate_expected_run_count": manifest.get("candidate_expected_run_count", ""),
        "candidate_axis_pass_count": manifest.get("candidate_axis_pass_count", ""),
        "candidate_target_pass_count": manifest.get("candidate_target_pass_count", ""),
        "promotion_sample_apply_count": manifest.get("promotion_sample_apply_count", ""),
        "promotion_sample_invalid_count": manifest.get("promotion_sample_invalid_count", ""),
        "public_topic_payload_sample_count": manifest.get("public_topic_payload_sample_count", ""),
        "public_topic_payload_node_equivalence_status": manifest.get("public_topic_payload_node_equivalence_status", ""),
        "package_check_status": package_check.get("status", ""),
        "package_check_failure_count": package_check.get("failure_count", ""),
        "operator_view_launch_status": operator_check.get("status", ""),
        "operator_package_type": operator_check.get("package_type", ""),
        "operator_bbox_adjustment_arg_present": operator_check.get("bbox_adjustment_arg_present", ""),
        "operator_launch_contract_status": operator_check.get("launch_contract_status", ""),
        "portability_required": portability_required,
        "portability_status": portability_check.get("status", ""),
        "portability_operator_view_status": portability_check.get("copied_operator_view_status", ""),
        "portability_runtime_truth_used": portability_check.get("runtime_truth_used", ""),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
    }


def sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        0 if row.get("status") == "PASS" else 1,
        finite_float(row.get("validation_range_rmse_corrected_mean_m")),
        finite_float(row.get("validation_range_rmse_corrected_max_m")),
        finite_float(row.get("target_max_abs_error_corrected_max_m")),
        str(row.get("recommended_policy", "")),
    )


def summarize(package_dirs: Sequence[Path]) -> dict[str, object]:
    rows = sorted([package_row(path) for path in package_dirs], key=sort_key)
    selected = rows[0] if rows and rows[0].get("status") == "PASS" else {}
    return {
        "status": "PASS" if selected else "FAIL",
        "selected_policy": selected.get("recommended_policy", ""),
        "selected_method": selected.get("method", ""),
        "selected_package_dir": selected.get("package_dir", ""),
        "selected_active_profile_sha256": selected.get("active_profile_sha256", ""),
        "reason": (
            "lowest validation range RMSE among package/operator/portability-ready policy winners"
            if selected
            else "no package/operator/portability-ready candidate"
        ),
        "candidate_count": len(rows),
        "pass_count": sum(1 for row in rows if row.get("status") == "PASS"),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "candidates": rows,
    }


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Deployable StereoDistance candidate package summary",
        "",
        f"status: {summary.get('status', '')}",
        f"selected_policy: {summary.get('selected_policy', '')}",
        f"selected_method: {summary.get('selected_method', '')}",
        f"selected_package_dir: {summary.get('selected_package_dir', '')}",
        f"selected_active_profile_sha256: {summary.get('selected_active_profile_sha256', '')}",
        f"reason: {summary.get('reason', '')}",
        f"candidate_count: {summary.get('candidate_count', '')}",
        f"pass_count: {summary.get('pass_count', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        "",
        "Candidates:",
    ]
    for row in summary.get("candidates", []):
        if not isinstance(row, Mapping):
            continue
        lines.append(
            (
                "- {policy}: {status} method={method} rmse_mean={rmse_mean} "
                "rmse_max={rmse_max} target_max={target_max} package={package_status} "
                "operator={operator_status} portability={portability_status}"
            ).format(
                policy=row.get("recommended_policy", ""),
                status=row.get("status", ""),
                method=row.get("method", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_corrected_max_m", ""),
                package_status=row.get("package_check_status", ""),
                operator_status=row.get("operator_view_launch_status", ""),
                portability_status=row.get("portability_status", ""),
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_outputs(output_dir: Path, summary: Mapping[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    candidates = summary.get("candidates", [])
    write_csv(output_dir / "deployable_profile_candidate_package_summary.csv", candidates if isinstance(candidates, list) else [])
    write_json(output_dir / "deployable_profile_candidate_package_summary.json", summary)
    write_text(output_dir / "deployable_profile_candidate_package_summary.txt", summary)


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    summary = summarize(args.package_dir)
    write_outputs(args.output_dir, summary)
    return summary


def main() -> int:
    result = run()
    compact = {
        key: result.get(key)
        for key in (
            "status",
            "selected_policy",
            "selected_method",
            "selected_package_dir",
            "candidate_count",
            "pass_count",
            "runtime_truth_used",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
