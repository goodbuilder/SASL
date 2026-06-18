#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    finite_float,
    method_complexity_estimate,
    write_csv,
    write_json,
)
from stereodistance_profile_replay import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT, load_profile  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "deployable profile policy selection reads offline common-CSV matrix evidence only; selected runtime profiles "
    "consume stereo-center RDF(right,down,forward) plus measured disparity and publish FRD+range"
)
POLICIES = ("range_priority", "target_priority", "score_priority", "balanced")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select deployable StereoDistance runtime profiles from a common-CSV frontier matrix. "
            "PX4/RTK truth is calibration-time validation evidence only and is not a runtime input."
        )
    )
    parser.add_argument("--matrix-summary-json", type=Path, required=True)
    parser.add_argument(
        "--runtime-profiles-dir",
        type=Path,
        help="Directory containing deployable <method>.json profiles. Defaults to matrix runtime_profiles_dir.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--recommended-policy",
        choices=POLICIES,
        default="range_priority",
        help="Policy used as the final recommendation after all hard gates pass.",
    )
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def runtime_truth_required(profile_payload: Mapping[str, object]) -> bool:
    runtime_inputs = {str(value).lower() for value in profile_payload.get("runtime_inputs", [])}
    forbidden = {str(value).lower() for value in profile_payload.get("forbidden_runtime_inputs", [])}
    forbidden_tokens = ("truth", "px4", "rtk", "ned")
    input_uses_truth = any(any(token in value for token in forbidden_tokens) for value in runtime_inputs)
    explicit_forbidden = any(any(token in value for token in forbidden_tokens) for value in forbidden)
    return input_uses_truth or not explicit_forbidden


def profile_payload(path: Path) -> dict[str, object] | None:
    if not path.is_file():
        return None
    try:
        load_profile(path)
        return read_json(path)
    except Exception:
        return None


def policy_sort_key(policy: str, row: Mapping[str, object]) -> tuple[object, ...]:
    rmse_mean = finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf"))
    rmse_max = finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf"))
    target_max = finite_float(row.get("target_max_abs_error_corrected_max_m"), float("inf"))
    score = finite_float(row.get("common_csv_score_mean"), float("inf"))
    position_mean = finite_float(row.get("validation_position_rmse_corrected_mean_m"), float("inf"))
    complexity = finite_float(row.get("model_complexity_estimate"), float("inf"))
    method = str(row.get("method", ""))
    if policy == "range_priority":
        return (rmse_mean, rmse_max, target_max, position_mean, complexity, method)
    if policy == "target_priority":
        return (target_max, rmse_mean, rmse_max, position_mean, complexity, method)
    if policy == "score_priority":
        return (score, rmse_mean, rmse_max, target_max, complexity, method)
    if policy == "balanced":
        balanced_score = rmse_mean + 0.20 * rmse_max + 0.35 * target_max + 0.03 * complexity
        return (balanced_score, rmse_mean, rmse_max, target_max, position_mean, method)
    raise ValueError(f"unknown policy: {policy}")


def candidate_from_method_row(
    row: Mapping[str, object],
    *,
    runtime_profiles_dir: Path,
    expected_run_count: int,
    target_error_limit_m: float,
) -> dict[str, object]:
    method = str(row.get("method", "")).strip()
    profile_json = runtime_profiles_dir / f"{method}.json"
    payload = profile_payload(profile_json)
    profile_exists = payload is not None
    runtime_truth_free = bool(payload is not None and not runtime_truth_required(payload))
    pass_count = int(finite_float(row.get("pass_count"), 0.0))
    axis_pass_count = int(finite_float(row.get("axis_pass_count"), 0.0))
    target_pass_count = int(finite_float(row.get("target_pass_count"), 0.0))
    evaluated_count = int(finite_float(row.get("evaluated_run_count"), 0.0))
    target_max = finite_float(row.get("target_max_abs_error_corrected_max_m"), float("inf"))
    pass_all_runs = parse_bool(row.get("pass_all_runs"))
    hard_gate_pass = (
        bool(method)
        and profile_exists
        and runtime_truth_free
        and expected_run_count > 0
        and evaluated_count == expected_run_count
        and pass_count == expected_run_count
        and pass_all_runs
        and axis_pass_count == expected_run_count
        and target_pass_count == expected_run_count
        and math.isfinite(target_max)
        and target_max <= target_error_limit_m
    )
    result = {
        "method": method,
        "profile_json": str(profile_json),
        "profile_exists": profile_exists,
        "profile_sha256": sha256_file(profile_json) if profile_exists else "",
        "runtime_truth_free": runtime_truth_free,
        "hard_gate_pass": hard_gate_pass,
        "evaluated_run_count": evaluated_count,
        "expected_run_count": expected_run_count,
        "pass_count": pass_count,
        "pass_all_runs": pass_all_runs,
        "axis_pass_count": axis_pass_count,
        "target_pass_count": target_pass_count,
        "validation_range_rmse_corrected_mean_m": row.get("validation_range_rmse_corrected_mean_m", ""),
        "validation_range_rmse_corrected_max_m": row.get("validation_range_rmse_corrected_max_m", ""),
        "validation_position_rmse_corrected_mean_m": row.get("validation_position_rmse_corrected_mean_m", ""),
        "validation_position_rmse_corrected_max_m": row.get("validation_position_rmse_corrected_max_m", ""),
        "target_max_abs_error_corrected_max_m": row.get("target_max_abs_error_corrected_max_m", ""),
        "common_csv_score_mean": row.get("common_csv_score_mean", ""),
        "common_csv_score_max": row.get("common_csv_score_max", ""),
        "model_complexity_estimate": method_complexity_estimate(method),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    return result


def select_profiles(
    *,
    matrix_summary_json: Path,
    runtime_profiles_dir: Path | None,
    target_error_limit_m: float,
    recommended_policy: str,
) -> dict[str, object]:
    matrix = read_json(matrix_summary_json)
    resolved_profiles_dir = runtime_profiles_dir or Path(str(matrix.get("runtime_profiles_dir", "")))
    if not str(resolved_profiles_dir):
        raise ValueError("--runtime-profiles-dir is required when matrix summary has no runtime_profiles_dir")
    expected_run_count = int(finite_float(matrix.get("run_count"), 0.0))
    candidate_rows = [
        candidate_from_method_row(
            row,
            runtime_profiles_dir=resolved_profiles_dir,
            expected_run_count=expected_run_count,
            target_error_limit_m=target_error_limit_m,
        )
        for row in matrix.get("methods", [])
        if isinstance(row, Mapping)
    ]
    passing_candidates = [row for row in candidate_rows if parse_bool(row.get("hard_gate_pass"))]
    policy_rows: list[dict[str, object]] = []
    for policy in POLICIES:
        ordered = sorted(passing_candidates, key=lambda row, policy=policy: policy_sort_key(policy, row))
        selected = ordered[0] if ordered else {}
        policy_rows.append(
            {
                "policy": policy,
                "status": "PASS" if selected else "FAIL",
                "selected_method": selected.get("method", ""),
                "selected_profile_json": selected.get("profile_json", ""),
                "selected_profile_sha256": selected.get("profile_sha256", ""),
                "validation_range_rmse_corrected_mean_m": selected.get("validation_range_rmse_corrected_mean_m", ""),
                "validation_range_rmse_corrected_max_m": selected.get("validation_range_rmse_corrected_max_m", ""),
                "validation_position_rmse_corrected_mean_m": selected.get(
                    "validation_position_rmse_corrected_mean_m", ""
                ),
                "target_max_abs_error_corrected_max_m": selected.get(
                    "target_max_abs_error_corrected_max_m", ""
                ),
                "common_csv_score_mean": selected.get("common_csv_score_mean", ""),
                "model_complexity_estimate": selected.get("model_complexity_estimate", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )
    recommended = next(row for row in policy_rows if row["policy"] == recommended_policy)
    status = "PASS" if parse_bool(recommended.get("status")) else "FAIL"
    return {
        "status": status,
        "recommendation": "promote_after_acceptance_dashboard" if status == "PASS" else "continue_search",
        "recommended_policy": recommended_policy,
        "recommended_method": recommended.get("selected_method", ""),
        "recommended_profile_json": recommended.get("selected_profile_json", ""),
        "recommended_profile_sha256": recommended.get("selected_profile_sha256", ""),
        "reason": (
            "hard-gated deployable profile selected by validation range RMSE priority"
            if recommended_policy == "range_priority"
            else f"hard-gated deployable profile selected by {recommended_policy}"
        ),
        "matrix_summary_json": str(matrix_summary_json),
        "runtime_profiles_dir": str(resolved_profiles_dir),
        "matrix_status": matrix.get("status", ""),
        "matrix_selected_method": matrix.get("selected_method", ""),
        "run_count": expected_run_count,
        "method_count": len(candidate_rows),
        "hard_gate_pass_count": len(passing_candidates),
        "target_error_limit_m": target_error_limit_m,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "policies": policy_rows,
        "candidates": sorted(candidate_rows, key=lambda row: (not parse_bool(row.get("hard_gate_pass")), policy_sort_key("range_priority", row))),
    }


def write_summary_txt(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Deployable StereoDistance profile policy selection",
        "",
        f"status: {summary.get('status', '')}",
        f"recommendation: {summary.get('recommendation', '')}",
        f"recommended_policy: {summary.get('recommended_policy', '')}",
        f"recommended_method: {summary.get('recommended_method', '')}",
        f"recommended_profile_json: {summary.get('recommended_profile_json', '')}",
        f"recommended_profile_sha256: {summary.get('recommended_profile_sha256', '')}",
        f"reason: {summary.get('reason', '')}",
        f"matrix_selected_method: {summary.get('matrix_selected_method', '')}",
        f"run_count: {summary.get('run_count', '')}",
        f"method_count: {summary.get('method_count', '')}",
        f"hard_gate_pass_count: {summary.get('hard_gate_pass_count', '')}",
        f"target_error_limit_m: {summary.get('target_error_limit_m', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "runtime_truth_used: False",
        "",
        "Policy winners:",
    ]
    for row in summary.get("policies", []):
        if not isinstance(row, Mapping):
            continue
        lines.append(
            (
                "- {policy}: {status} method={method} rmse_mean={rmse_mean} rmse_max={rmse_max} "
                "target_max={target_max} score={score}"
            ).format(
                policy=row.get("policy", ""),
                status=row.get("status", ""),
                method=row.get("selected_method", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_corrected_max_m", ""),
                score=row.get("common_csv_score_mean", ""),
            )
        )
    lines.extend(["", "Top hard-gated candidates:"])
    top_candidates = [
        row for row in summary.get("candidates", []) if isinstance(row, Mapping) and parse_bool(row.get("hard_gate_pass"))
    ][:20]
    for row in top_candidates:
        lines.append(
            (
                "- method={method} rmse_mean={rmse_mean} rmse_max={rmse_max} target_max={target_max} "
                "axis_pass={axis}/{expected}"
            ).format(
                method=row.get("method", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                target_max=row.get("target_max_abs_error_corrected_max_m", ""),
                axis=row.get("axis_pass_count", ""),
                expected=row.get("expected_run_count", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_outputs(output_dir: Path, summary: Mapping[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    policies = summary.get("policies", [])
    candidates = summary.get("candidates", [])
    write_csv(output_dir / "deployable_profile_policy_selection.csv", policies if isinstance(policies, list) else [])
    write_csv(output_dir / "deployable_profile_candidates.csv", candidates if isinstance(candidates, list) else [])
    write_json(output_dir / "deployable_profile_policy_selection.json", summary)
    write_summary_txt(output_dir / "deployable_profile_policy_selection.txt", summary)


def parse_args_from_sequence(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-summary-json", type=Path, required=True)
    parser.add_argument("--runtime-profiles-dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--recommended-policy", choices=POLICIES, default="range_priority")
    return parser.parse_args(argv)


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args() if argv is None else parse_args_from_sequence(argv)
    summary = select_profiles(
        matrix_summary_json=args.matrix_summary_json,
        runtime_profiles_dir=args.runtime_profiles_dir,
        target_error_limit_m=args.target_error_limit_m,
        recommended_policy=args.recommended_policy,
    )
    write_outputs(args.output_dir, summary)
    return summary


def main() -> int:
    payload = run()
    compact = {
        key: payload.get(key)
        for key in (
            "status",
            "recommendation",
            "recommended_policy",
            "recommended_method",
            "recommended_profile_json",
            "matrix_selected_method",
            "run_count",
            "method_count",
            "hard_gate_pass_count",
            "axis_contract",
            "public_topic_contract",
            "runtime_input_contract",
            "runtime_truth_used",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
