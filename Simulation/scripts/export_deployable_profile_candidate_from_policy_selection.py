#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import export_stereodistance_public_topic_payload_from_profile as public_payload_exporter  # noqa: E402
import stereodistance_profile_promotion_check as promotion_check  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
RUNTIME_INPUT_CONTRACT = (
    "exported candidate consumes stereo-center RDF(right,down,forward) plus measured disparity; "
    "PX4/RTK/NED truth remains offline calibration/evaluation evidence only"
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export the policy-selected deployable StereoDistance profile as an active-profile candidate, then "
            "run promotion and public FRD+range payload smoke checks."
        )
    )
    parser.add_argument("--policy-selection-json", type=Path, required=True)
    parser.add_argument("--output-active-profile", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-csv", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--max-samples", type=int, default=80)
    return parser.parse_args(argv)


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def selected_policy_row(selection: Mapping[str, object]) -> dict[str, object]:
    recommended_policy = str(selection.get("recommended_policy", ""))
    for row in selection.get("policies", []):
        if isinstance(row, Mapping) and str(row.get("policy", "")) == recommended_policy:
            return dict(row)
    return {}


def policy_candidate_row(selection: Mapping[str, object], method: str) -> dict[str, object]:
    for row in selection.get("candidates", []):
        if isinstance(row, Mapping) and str(row.get("method", "")) == method:
            return dict(row)
    return {}


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Deployable StereoDistance active-profile candidate export",
        "",
        f"status: {summary.get('status', '')}",
        f"recommendation: {summary.get('recommendation', '')}",
        f"policy_selection_json: {summary.get('policy_selection_json', '')}",
        f"recommended_policy: {summary.get('recommended_policy', '')}",
        f"method: {summary.get('method', '')}",
        f"source_profile_json: {summary.get('source_profile_json', '')}",
        f"output_active_profile: {summary.get('output_active_profile', '')}",
        f"active_profile_sha256: {summary.get('active_profile_sha256', '')}",
        f"promotion_check_pass: {summary.get('promotion_check_pass', '')}",
        f"promotion_sample_apply_count: {summary.get('promotion_sample_apply_count', '')}",
        f"promotion_sample_invalid_count: {summary.get('promotion_sample_invalid_count', '')}",
        f"public_topic_payload_status: {summary.get('public_topic_payload_status', '')}",
        f"public_topic_payload_sample_count: {summary.get('public_topic_payload_sample_count', '')}",
        f"public_topic_payload_invalid_count: {summary.get('public_topic_payload_invalid_count', '')}",
        f"validation_range_rmse_corrected_mean_m: {summary.get('validation_range_rmse_corrected_mean_m', '')}",
        f"validation_range_rmse_corrected_max_m: {summary.get('validation_range_rmse_corrected_max_m', '')}",
        f"target_max_abs_error_corrected_max_m: {summary.get('target_max_abs_error_corrected_max_m', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', '')}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_candidate(
    *,
    policy_selection_json: Path,
    output_active_profile: Path,
    output_dir: Path,
    sample_csv: Path | None = None,
    split: str = "validation",
    max_samples: int = 80,
) -> dict[str, object]:
    selection = read_json(policy_selection_json)
    if selection.get("status") != "PASS":
        raise ValueError("policy selection did not pass")
    method = str(selection.get("recommended_method", ""))
    source_profile = Path(str(selection.get("recommended_profile_json", "")))
    if not source_profile.is_file():
        raise FileNotFoundError(f"recommended profile missing: {source_profile}")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_active_profile.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_profile, output_active_profile)
    active_sha = sha256_file(output_active_profile)
    expected_sha = str(selection.get("recommended_profile_sha256", ""))
    if expected_sha and active_sha != expected_sha:
        raise ValueError("copied active profile SHA-256 does not match policy selection")

    promotion = promotion_check.run_check(
        output_active_profile,
        output_dir / "promotion_check",
        sample_csv=sample_csv,
        split=split,
        max_samples=max_samples,
    )
    public_payload: dict[str, object] = {}
    if sample_csv is not None:
        public_payload = public_payload_exporter.export_payloads(
            profile_json=output_active_profile,
            sample_csv=sample_csv,
            output_dir=output_dir / "public_topic_payload",
            split=split,
            max_samples=max_samples,
        )

    policy_row = selected_policy_row(selection)
    candidate_row = policy_candidate_row(selection, method)
    promotion_pass = bool(promotion.get("pass_promotion_check", False))
    public_status = str(public_payload.get("status", ""))
    public_pass = not public_payload or public_status == "PASS"
    status = "PASS" if promotion_pass and public_pass else "FAIL"
    summary = {
        "mode": "deployable_profile_candidate_export",
        "status": status,
        "recommendation": "use_as_next_acceptance_dashboard_active_profile" if status == "PASS" else "do_not_promote",
        "policy_selection_json": str(policy_selection_json),
        "recommended_policy": selection.get("recommended_policy", ""),
        "method": method,
        "source_profile_json": str(source_profile),
        "output_active_profile": str(output_active_profile),
        "active_profile_sha256": active_sha,
        "policy_selection_sha256": expected_sha,
        "profile_sha256_matches_policy": active_sha == expected_sha if expected_sha else "",
        "matrix_summary_json": selection.get("matrix_summary_json", ""),
        "matrix_selected_method": selection.get("matrix_selected_method", ""),
        "hard_gate_pass_count": selection.get("hard_gate_pass_count", ""),
        "validation_range_rmse_corrected_mean_m": policy_row.get(
            "validation_range_rmse_corrected_mean_m",
            candidate_row.get("validation_range_rmse_corrected_mean_m", ""),
        ),
        "validation_range_rmse_corrected_max_m": policy_row.get(
            "validation_range_rmse_corrected_max_m",
            candidate_row.get("validation_range_rmse_corrected_max_m", ""),
        ),
        "target_max_abs_error_corrected_max_m": policy_row.get(
            "target_max_abs_error_corrected_max_m",
            candidate_row.get("target_max_abs_error_corrected_max_m", ""),
        ),
        "candidate_pass_count": candidate_row.get("pass_count", ""),
        "candidate_expected_run_count": candidate_row.get("expected_run_count", ""),
        "candidate_axis_pass_count": candidate_row.get("axis_pass_count", ""),
        "candidate_target_pass_count": candidate_row.get("target_pass_count", ""),
        "promotion_check_json": str(output_dir / "promotion_check" / "promotion_check.json"),
        "promotion_check_pass": promotion_pass,
        "promotion_sample_apply_count": promotion.get("sample_apply_count", 0),
        "promotion_sample_invalid_count": promotion.get("sample_invalid_count", 0),
        "public_topic_payload_json": str(output_dir / "public_topic_payload" / "public_corrected_position_payload_summary.json")
        if public_payload
        else "",
        "public_topic_payload_csv": str(output_dir / "public_topic_payload" / "public_corrected_position_payload.csv")
        if public_payload
        else "",
        "public_topic_payload_status": public_status,
        "public_topic_payload_sample_count": public_payload.get("sample_count", ""),
        "public_topic_payload_invalid_count": public_payload.get("invalid_count", ""),
        "public_topic_payload_node_equivalence_status": public_payload.get(
            "node_public_payload_equivalence_status",
            "",
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "selection_summary": selection,
        "promotion_summary": promotion,
        "public_topic_payload_summary": public_payload,
    }
    write_json(output_dir / "deployable_profile_candidate_export.json", summary)
    write_text(output_dir / "deployable_profile_candidate_export.txt", summary)
    return summary


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_candidate(
        policy_selection_json=args.policy_selection_json,
        output_active_profile=args.output_active_profile,
        output_dir=args.output_dir,
        sample_csv=args.sample_csv,
        split=args.split,
        max_samples=args.max_samples,
    )


def main() -> int:
    result = run()
    compact = {
        key: result.get(key)
        for key in (
            "status",
            "recommendation",
            "recommended_policy",
            "method",
            "output_active_profile",
            "active_profile_sha256",
            "promotion_check_pass",
            "public_topic_payload_status",
            "runtime_truth_used",
            "axis_contract",
            "public_topic_contract",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
