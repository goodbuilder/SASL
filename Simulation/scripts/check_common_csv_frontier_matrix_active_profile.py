#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Mapping


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from compare_stereodistance_profiles_on_common_csv import parse_bool  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from stereodistance_profile_replay import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT, load_profile  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "active profile matrix gate reads offline common-CSV frontier evidence only; runtime apply consumes "
    "stereo-center RDF(right,down,forward) plus measured disparity and publishes FRD+range"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check whether an active StereoDistance runtime profile is still the method recommended by "
            "the common-CSV frontier matrix."
        )
    )
    parser.add_argument("--matrix-summary-json", type=Path, required=True)
    parser.add_argument("--active-profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def method_row(summary: Mapping[str, object], method: str) -> dict[str, object]:
    for row in summary.get("methods", []):
        if isinstance(row, Mapping) and str(row.get("method", "")) == method:
            return dict(row)
    return {}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def same_profile_path(value: object, active_profile_json: Path) -> bool:
    text = str(value)
    if not text:
        return False
    candidate = Path(text)
    if text == str(active_profile_json):
        return True
    try:
        return candidate.resolve() == active_profile_json.resolve()
    except OSError:
        return False


def active_profile_candidate_rows(matrix: Mapping[str, object], active_profile_json: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run in matrix.get("runs", []):
        if not isinstance(run, Mapping):
            continue
        output_dir = Path(str(run.get("output_dir", "")))
        comparison_csv = output_dir / "common_csv_profile_comparison.csv"
        if not comparison_csv.is_file():
            continue
        for row in read_csv(comparison_csv):
            if same_profile_path(row.get("profile_json", ""), active_profile_json):
                rows.append({**row, "_scenario": run.get("scenario", ""), "_seed": run.get("seed", "")})
                break
    return rows


def mean_or_blank(values: list[float]) -> float | str:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else ""


def max_or_blank(values: list[float]) -> float | str:
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else ""


def summarize_active_profile_candidate_rows(rows: list[dict[str, object]], expected_run_count: int) -> dict[str, object]:
    if not rows:
        return {
            "active_profile_candidate_rows_found": False,
            "active_profile_candidate_evaluated_run_count": 0,
            "active_profile_candidate_expected_run_count": expected_run_count,
            "active_profile_candidate_pass_count": "",
            "active_profile_candidate_pass_all_runs": "",
            "active_profile_candidate_axis_pass_count": "",
            "active_profile_candidate_target_pass_count": "",
            "active_profile_candidate_validation_range_rmse_corrected_mean_m": "",
            "active_profile_candidate_validation_range_rmse_corrected_max_m": "",
            "active_profile_candidate_target_max_abs_error_corrected_max_m": "",
            "active_profile_candidate_common_csv_score_mean": "",
        }
    rmse = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
    target = [finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in rows]
    scores = [finite_float(row.get("common_csv_score"), float("nan")) for row in rows]
    pass_count = sum(1 for row in rows if parse_bool(row.get("pass_common_replay")))
    axis_count = sum(1 for row in rows if parse_bool(row.get("rdf_axis_nonregression_pass")))
    target_count = sum(1 for row in rows if parse_bool(row.get("target_max_error_pass")))
    return {
        "active_profile_candidate_rows_found": True,
        "active_profile_candidate_evaluated_run_count": len(rows),
        "active_profile_candidate_expected_run_count": expected_run_count,
        "active_profile_candidate_pass_count": pass_count,
        "active_profile_candidate_pass_all_runs": expected_run_count > 0 and pass_count == expected_run_count,
        "active_profile_candidate_axis_pass_count": axis_count,
        "active_profile_candidate_target_pass_count": target_count,
        "active_profile_candidate_validation_range_rmse_corrected_mean_m": mean_or_blank(rmse),
        "active_profile_candidate_validation_range_rmse_corrected_max_m": max_or_blank(rmse),
        "active_profile_candidate_target_max_abs_error_corrected_max_m": max_or_blank(target),
        "active_profile_candidate_common_csv_score_mean": mean_or_blank(scores),
    }


def check_active_profile_against_matrix(
    *,
    matrix_summary_json: Path,
    active_profile_json: Path,
    output_dir: Path,
) -> dict[str, object]:
    matrix = read_json(matrix_summary_json)
    profile_payload = read_json(active_profile_json)
    load_profile(active_profile_json)
    active_method = str(profile_payload.get("method", ""))
    selected_method = str(matrix.get("selected_method", ""))
    active_method_row = method_row(matrix, active_method)
    selected_method_row = method_row(matrix, selected_method)
    matrix_status_pass = str(matrix.get("status", "")).upper() == "PASS"
    active_matches_matrix = bool(active_method) and active_method == selected_method
    active_pass_all_runs = parse_bool(active_method_row.get("pass_all_runs"))
    expected_run_count = int(finite_float(matrix.get("run_count"), 0.0))
    actual_active_rows = active_profile_candidate_rows(matrix, active_profile_json)
    active_candidate_summary = summarize_active_profile_candidate_rows(actual_active_rows, expected_run_count)
    actual_active_rows_found = parse_bool(active_candidate_summary.get("active_profile_candidate_rows_found"))
    actual_active_pass_all_runs = parse_bool(active_candidate_summary.get("active_profile_candidate_pass_all_runs"))
    active_runtime_truth_free = not runtime_truth_required(profile_payload)
    active_deployable_pass_all_runs = actual_active_pass_all_runs if actual_active_rows_found else active_pass_all_runs
    status = (
        "PASS"
        if matrix_status_pass
        and active_matches_matrix
        and active_deployable_pass_all_runs
        and active_runtime_truth_free
        else "FAIL"
    )
    result = {
        "status": status,
        "recommendation": "keep_active_profile" if status == "PASS" else "do_not_promote_without_review",
        "matrix_summary_json": str(matrix_summary_json),
        "active_profile_json": str(active_profile_json),
        "active_profile_sha256": sha256_file(active_profile_json),
        "matrix_status": matrix.get("status", ""),
        "matrix_selected_method": selected_method,
        "active_profile_method": active_method,
        "active_matches_matrix_selected_method": active_matches_matrix,
        "active_method_pass_all_runs": active_pass_all_runs,
        "active_method_evaluated_run_count": active_method_row.get("evaluated_run_count", ""),
        "active_method_expected_run_count": active_method_row.get("expected_run_count", ""),
        "active_method_pass_count": active_method_row.get("pass_count", ""),
        "active_method_selected_count": active_method_row.get("selected_count", ""),
        "active_method_validation_range_rmse_corrected_mean_m": active_method_row.get(
            "validation_range_rmse_corrected_mean_m", ""
        ),
        "active_method_validation_range_rmse_corrected_max_m": active_method_row.get(
            "validation_range_rmse_corrected_max_m", ""
        ),
        "active_method_target_max_abs_error_corrected_max_m": active_method_row.get(
            "target_max_abs_error_corrected_max_m", ""
        ),
        **active_candidate_summary,
        "selected_method_pass_all_runs": selected_method_row.get("pass_all_runs", ""),
        "selected_method_common_csv_score_mean": selected_method_row.get("common_csv_score_mean", ""),
        "active_profile_runtime_truth_free": active_runtime_truth_free,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "active_profile_matrix_gate.csv", [result])
    write_json(output_dir / "active_profile_matrix_gate.json", result)
    write_summary_txt(output_dir / "active_profile_matrix_gate.txt", result)
    return result


def runtime_truth_required(profile_payload: Mapping[str, object]) -> bool:
    runtime_inputs = {str(value).lower() for value in profile_payload.get("runtime_inputs", [])}
    forbidden = {str(value).lower() for value in profile_payload.get("forbidden_runtime_inputs", [])}
    forbidden_tokens = ("truth", "px4", "rtk", "ned")
    input_uses_truth = any(any(token in value for token in forbidden_tokens) for value in runtime_inputs)
    explicit_forbidden = any(any(token in value for token in forbidden_tokens) for value in forbidden)
    return input_uses_truth or not explicit_forbidden


def write_summary_txt(path: Path, result: Mapping[str, object]) -> None:
    lines = [
        "Active profile common-CSV frontier matrix gate",
        "",
        f"status: {result.get('status', '')}",
        f"recommendation: {result.get('recommendation', '')}",
        f"matrix_selected_method: {result.get('matrix_selected_method', '')}",
        f"active_profile_method: {result.get('active_profile_method', '')}",
        f"active_matches_matrix_selected_method: {result.get('active_matches_matrix_selected_method', '')}",
        f"active_method_pass_count: {result.get('active_method_pass_count', '')}/{result.get('active_method_expected_run_count', '')}",
        f"active_method_selected_count: {result.get('active_method_selected_count', '')}",
        f"active_method_rmse_mean_m: {result.get('active_method_validation_range_rmse_corrected_mean_m', '')}",
        f"active_method_rmse_max_m: {result.get('active_method_validation_range_rmse_corrected_max_m', '')}",
        f"active_method_target_max_abs_m: {result.get('active_method_target_max_abs_error_corrected_max_m', '')}",
        f"active_profile_candidate_rows_found: {result.get('active_profile_candidate_rows_found', '')}",
        f"active_profile_candidate_pass_count: {result.get('active_profile_candidate_pass_count', '')}/{result.get('active_profile_candidate_expected_run_count', '')}",
        f"active_profile_candidate_axis_pass_count: {result.get('active_profile_candidate_axis_pass_count', '')}",
        f"active_profile_candidate_target_pass_count: {result.get('active_profile_candidate_target_pass_count', '')}",
        f"active_profile_candidate_rmse_mean_m: {result.get('active_profile_candidate_validation_range_rmse_corrected_mean_m', '')}",
        f"active_profile_candidate_rmse_max_m: {result.get('active_profile_candidate_validation_range_rmse_corrected_max_m', '')}",
        f"active_profile_candidate_target_max_abs_m: {result.get('active_profile_candidate_target_max_abs_error_corrected_max_m', '')}",
        f"active_profile_runtime_truth_free: {result.get('active_profile_runtime_truth_free', '')}",
        f"active_profile_sha256: {result.get('active_profile_sha256', '')}",
        f"axis_contract: {result.get('axis_contract', '')}",
        f"public_topic_contract: {result.get('public_topic_contract', '')}",
        f"runtime_input_contract: {result.get('runtime_input_contract', '')}",
        "runtime_truth_used: False",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    result = check_active_profile_against_matrix(
        matrix_summary_json=args.matrix_summary_json,
        active_profile_json=args.active_profile_json,
        output_dir=args.output_dir,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
