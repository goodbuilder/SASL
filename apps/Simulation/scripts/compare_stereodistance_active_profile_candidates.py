#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "truth offline PX4 local NED; runtime input stereo-center RDF(right,down,forward) "
    "plus measured disparity; public output FRD+range"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare deployment-ready StereoDistance active-profile candidates and select the "
            "best runtime-truth-free, RDF-axis-clean candidate by validation evidence."
        )
    )
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        metavar="LABEL:DASHBOARD_DIR:ROBUSTNESS_SUMMARY_JSON",
        help=(
            "Candidate triple. DASHBOARD_DIR must contain active_runtime_profile.json, "
            "rtk_stereo_acceptance_dashboard_with_promotion_summary.json, and deployment_readiness.json."
        ),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def finite_or_blank(value: object) -> float | str:
    parsed = finite_float(value, float("nan"))
    return parsed if math.isfinite(parsed) else ""


def balanced_validation_target_score(row: Mapping[str, object]) -> float:
    rmse_mean = finite_float(row.get("robust_validation_range_rmse_mean_m"), float("inf"))
    rmse_max = finite_float(row.get("robust_validation_range_rmse_max_m"), float("inf"))
    target_max = finite_float(row.get("robust_target_abs_mean_error_max_m"), float("inf"))
    if not all(math.isfinite(value) for value in (rmse_mean, rmse_max, target_max)):
        return float("inf")
    return rmse_mean + 0.5 * rmse_max + target_max


def csv_sample_fingerprint(sample_csv: object, split: object) -> dict[str, object]:
    path_text = str(sample_csv or "").strip()
    split_text = str(split or "").strip()
    if not path_text:
        return {
            "promotion_sample_csv": "",
            "promotion_sample_split": split_text,
            "promotion_sample_fingerprint_sha256": "",
            "promotion_sample_fingerprint_count": "",
            "promotion_sample_truth_range_mean_m": "",
            "promotion_sample_raw_range_mean_m": "",
        }
    path = Path(path_text)
    if not path.exists():
        return {
            "promotion_sample_csv": path_text,
            "promotion_sample_split": split_text,
            "promotion_sample_fingerprint_sha256": "",
            "promotion_sample_fingerprint_count": "",
            "promotion_sample_truth_range_mean_m": "",
            "promotion_sample_raw_range_mean_m": "",
        }
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    selected = [row for row in rows if not split_text or str(row.get("split", "")) == split_text]
    selected.sort(key=lambda row: str(row.get("sample_id", "")))
    digest = hashlib.sha256()
    columns = [
        "sample_id",
        "split",
        "truth_right_m",
        "truth_down_m",
        "truth_forward_m",
        "truth_range_m",
        "raw_right_m",
        "raw_down_m",
        "raw_forward_m",
        "raw_range_m",
        "measured_disparity_px",
    ]
    truth_ranges: list[float] = []
    raw_ranges: list[float] = []
    for row in selected:
        digest.update("\x1e".join(str(row.get(column, "")) for column in columns).encode("utf-8"))
        digest.update(b"\n")
        truth_range = finite_float(row.get("truth_range_m"), float("nan"))
        raw_range = finite_float(row.get("raw_range_m"), float("nan"))
        if math.isfinite(truth_range):
            truth_ranges.append(truth_range)
        if math.isfinite(raw_range):
            raw_ranges.append(raw_range)
    return {
        "promotion_sample_csv": path_text,
        "promotion_sample_split": split_text,
        "promotion_sample_fingerprint_sha256": digest.hexdigest(),
        "promotion_sample_fingerprint_count": len(selected),
        "promotion_sample_truth_range_mean_m": (
            sum(truth_ranges) / len(truth_ranges) if truth_ranges else ""
        ),
        "promotion_sample_raw_range_mean_m": sum(raw_ranges) / len(raw_ranges) if raw_ranges else "",
    }


def parse_candidate_spec(spec: str) -> tuple[str, Path, Path]:
    parts = spec.split(":", 2)
    if len(parts) != 3 or not all(part.strip() for part in parts):
        raise ValueError("--candidate must be LABEL:DASHBOARD_DIR:ROBUSTNESS_SUMMARY_JSON")
    label, dashboard_dir, robustness_summary = parts
    return label.strip(), Path(dashboard_dir), Path(robustness_summary)


def candidate_row(label: str, dashboard_dir: Path, robustness_summary_json: Path) -> dict[str, object]:
    profile_json = dashboard_dir / "active_runtime_profile.json"
    dashboard_summary_json = dashboard_dir / "rtk_stereo_acceptance_dashboard_with_promotion_summary.json"
    readiness_json = dashboard_dir / "deployment_readiness.json"
    promotion_check_json = dashboard_dir / "active_profile_promotion_check" / "promotion_check.json"
    profile = read_json(profile_json)
    dashboard_summary = read_json(dashboard_summary_json)
    readiness = read_json(readiness_json)
    promotion_check = read_json(promotion_check_json) if promotion_check_json.exists() else {}
    robustness_summary = read_json(robustness_summary_json)
    robust_recommendation = robustness_summary.get("robust_recommendation", {})
    if not isinstance(robust_recommendation, Mapping):
        robust_recommendation = {}
    validation_summary = profile.get("validation_summary", {})
    if not isinstance(validation_summary, Mapping):
        validation_summary = {}
    dashboard_status = str(dashboard_summary.get("status", ""))
    readiness_status = str(readiness.get("status", ""))
    axis_status = str(dashboard_summary.get("final_rdf_axis_nonregression_status", ""))
    runtime_truth_used = parse_bool(dashboard_summary.get("runtime_truth_used"))
    robust_axis_regression = finite_float(
        robust_recommendation.get("axis_rmse_regression_sum_mean_m"), float("inf")
    )
    profile_method = str(profile.get("method", "") or "")
    robust_method = str(robust_recommendation.get("method", "") or "")
    profile_matches_robust = bool(profile_method and robust_method and profile_method == robust_method)
    deployment_ready = (
        dashboard_status == "PASS"
        and readiness_status == "PASS"
        and axis_status == "PASS"
        and not runtime_truth_used
        and robust_axis_regression <= 0.0
    )
    sample_fingerprint = csv_sample_fingerprint(
        promotion_check.get("sample_csv", ""),
        promotion_check.get("sample_split", ""),
    )
    row = {
        "candidate": label,
        "deployment_ready": deployment_ready,
        "dashboard_dir": str(dashboard_dir),
        "robustness_summary_json": str(robustness_summary_json),
        "dashboard_status": dashboard_status,
        "readiness_status": readiness_status,
        "method": profile_method,
        "profile_id": profile.get("profile_id", ""),
        "profile_mode": profile.get("mode", ""),
        "active_profile_sha256": readiness.get("active_profile_sha256", ""),
        "profile_validation_range_rmse_raw_m": finite_or_blank(
            validation_summary.get("validation_range_rmse_raw_m")
        ),
        "profile_validation_range_rmse_corrected_m": finite_or_blank(
            validation_summary.get("validation_range_rmse_corrected_m")
        ),
        "profile_target_1852m_abs_mean_error_m": finite_or_blank(
            validation_summary.get("target_1852m_abs_mean_error_m")
        ),
        "robust_recommendation_method": robust_method,
        "profile_matches_robust_recommendation": profile_matches_robust,
        "method_selection_scope": (
            "exported_runtime_profile"
            if profile_matches_robust
            else "exported_runtime_profile differs from robustness frontier recommendation"
        ),
        "robust_validation_range_rmse_mean_m": finite_or_blank(
            robust_recommendation.get("validation_range_rmse_mean_m")
        ),
        "robust_validation_range_rmse_max_m": finite_or_blank(
            robust_recommendation.get("validation_range_rmse_max_m")
        ),
        "robust_target_abs_mean_error_mean_m": finite_or_blank(
            robust_recommendation.get("target_abs_mean_error_mean_m")
        ),
        "robust_target_abs_mean_error_max_m": finite_or_blank(
            robust_recommendation.get("target_abs_mean_error_max_m")
        ),
        "robust_axis_rmse_regression_sum_mean_m": finite_or_blank(
            robust_recommendation.get("axis_rmse_regression_sum_mean_m")
        ),
        "final_rdf_axis_nonregression_status": axis_status,
        "runtime_truth_used": runtime_truth_used,
        "promotion_sample_apply_count": dashboard_summary.get("promotion_sample_apply_count", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
    }
    row.update(sample_fingerprint)
    row["balanced_validation_target_score"] = finite_or_blank(balanced_validation_target_score(row))
    return row


def candidate_sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        not parse_bool(row.get("deployment_ready")),
        finite_float(row.get("balanced_validation_target_score"), float("inf")),
        finite_float(row.get("robust_validation_range_rmse_mean_m"), float("inf")),
        finite_float(row.get("robust_target_abs_mean_error_max_m"), float("inf")),
        finite_float(row.get("profile_validation_range_rmse_corrected_m"), float("inf")),
        str(row.get("candidate", "")),
    )


def compare_candidates(
    *,
    candidate_specs: Sequence[str],
    output_dir: Path,
) -> dict[str, object]:
    rows = [candidate_row(*parse_candidate_spec(spec)) for spec in candidate_specs]
    passing = [row for row in rows if parse_bool(row.get("deployment_ready"))]
    selected = min(passing, key=candidate_sort_key) if passing else None
    sample_hashes = sorted(
        {
            str(row.get("promotion_sample_fingerprint_sha256", ""))
            for row in rows
            if str(row.get("promotion_sample_fingerprint_sha256", "")).strip()
        }
    )
    summary = {
        "status": "PASS" if selected else "FAIL",
        "selected_candidate": selected.get("candidate", "") if selected else "",
        "selected_method": selected.get("method", "") if selected else "",
        "selected_profile_sha256": selected.get("active_profile_sha256", "") if selected else "",
        "reason": (
            "lowest robust balanced validation/1852m target score among readiness PASS, RDF-axis-clean, runtime-truth-free candidates"
            if selected
            else "no deployment-ready candidate"
        ),
        "candidate_count": len(rows),
        "passing_candidate_count": len(passing),
        "promotion_sample_unique_fingerprint_count": len(sample_hashes),
        "promotion_samples_comparable": len(sample_hashes) <= 1,
        "promotion_sample_comparison_note": (
            "all candidates used the same promotion sample fingerprint"
            if len(sample_hashes) <= 1
            else "promotion replay samples differ; profile validation fields are evidence only, not primary selection score"
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "rows": rows,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "profile_candidate_comparison.csv", rows)
    write_json(output_dir / "profile_candidate_comparison.json", summary)
    write_summary_txt(output_dir / "profile_candidate_comparison.txt", summary, rows)
    return summary


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance active profile candidate comparison",
        "",
        f"status: {summary.get('status', '')}",
        f"selected_candidate: {summary.get('selected_candidate', '')}",
        f"selected_method: {summary.get('selected_method', '')}",
        f"selected_profile_sha256: {summary.get('selected_profile_sha256', '')}",
        f"reason: {summary.get('reason', '')}",
        f"promotion_samples_comparable: {summary.get('promotion_samples_comparable', '')}",
        f"promotion_sample_comparison_note: {summary.get('promotion_sample_comparison_note', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "score_contract: robust validation RMSE mean/max and 1852m target max drive selection; "
        "promotion sample fingerprints are evidence only",
        "",
        "Candidates:",
    ]
    for row in rows:
        lines.append(
            (
                "- {candidate}: method={method}, ready={ready}, dashboard={dashboard}, "
                "readiness={readiness}, robust_method={robust_method}, profile_matches_robust={matches}, "
                "robust_rmse_mean={rmse}, robust_target_max={target}, score={score}, "
                "axis={axis}, runtime_truth={truth}, sample_hash={sample_hash}"
            ).format(
                candidate=row.get("candidate", ""),
                method=row.get("method", ""),
                ready=row.get("deployment_ready", ""),
                dashboard=row.get("dashboard_status", ""),
                readiness=row.get("readiness_status", ""),
                robust_method=row.get("robust_recommendation_method", ""),
                matches=row.get("profile_matches_robust_recommendation", ""),
                rmse=row.get("robust_validation_range_rmse_mean_m", ""),
                target=row.get("robust_target_abs_mean_error_max_m", ""),
                score=row.get("balanced_validation_target_score", ""),
                axis=row.get("final_rdf_axis_nonregression_status", ""),
                truth=row.get("runtime_truth_used", ""),
                sample_hash=row.get("promotion_sample_fingerprint_sha256", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    summary = compare_candidates(candidate_specs=args.candidate, output_dir=args.output_dir)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
