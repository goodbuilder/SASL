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
RUNTIME_INPUT_CONTRACT = (
    "method-search catalog uses PX4/RTK/NED truth only for offline calibration/evaluation; deployable runtime "
    "profiles consume stereo-center RDF(right,down,forward) plus measured disparity and publish FRD+range"
)

FAMILY_HINTS = {
    "rotation_or_pose_alignment": (("rotation",), ("rot",), ("kabsch",)),
    "scale_or_intrinsic_compensation": (("scale",), ("focal",), ("baseline",)),
    "sim3_similarity_alignment": (("sim3",), ("umeyama",)),
    "linear_disparity_or_inverse_depth": (("linear", "disparity"), ("inverse_depth",)),
    "nonlinear_or_binned_residual": (("quadratic",), ("poly",), ("binned",), ("lut",), ("residual",)),
    "combined_geometry_and_disparity": (("sim3", "disparity"), ("sim3", "residual"), ("pose", "residual")),
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit whether the RTK stereo correction-method search catalog is broad enough, truth-free at runtime, "
            "and ready for continued automatic exploration."
        )
    )
    parser.add_argument("--matrix-summary-json", type=Path, required=True)
    parser.add_argument("--policy-selection-json", type=Path, required=True)
    parser.add_argument("--bundle-manifest-json", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-method-count", type=int, default=12)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
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


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def finite_float(value: object, default: float = float("inf")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def method_name(row: Mapping[str, object]) -> str:
    return str(row.get("method", row.get("selected_method", row.get("recommended_method", "")))).strip()


def family_covered(methods: Sequence[str], alternatives: Sequence[Sequence[str]]) -> tuple[bool, list[str]]:
    matched = []
    lowered = [method.lower() for method in methods]
    for method in lowered:
        for hints in alternatives:
            if all(hint in method for hint in hints):
                matched.append(method)
                break
    return bool(matched), matched[:8]


def build_family_rows(methods: Sequence[str]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for family, hints in FAMILY_HINTS.items():
        covered, examples = family_covered(methods, hints)
        rows.append(
            {
                "family": family,
                "status": "PASS" if covered else "FAIL",
                "example_count": len(examples),
                "examples": ";".join(examples),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
            }
        )
    return rows


def candidate_status_rows(selection: Mapping[str, object], target_error_limit_m: float) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for candidate in selection.get("candidates", []):
        if not isinstance(candidate, Mapping):
            continue
        method = method_name(candidate)
        hard_gate = parse_bool(candidate.get("hard_gate_pass"))
        runtime_truth_free = parse_bool(candidate.get("runtime_truth_free"))
        profile_exists = parse_bool(candidate.get("profile_exists"))
        target_error = finite_float(candidate.get("target_max_abs_error_corrected_max_m"))
        expected = int(finite_float(candidate.get("expected_run_count"), 0.0))
        pass_count = int(finite_float(candidate.get("pass_count"), 0.0))
        axis_pass = int(finite_float(candidate.get("axis_pass_count"), 0.0))
        rows.append(
            {
                "method": method,
                "status": "PASS" if hard_gate else "FAIL",
                "profile_exists": profile_exists,
                "runtime_truth_free": runtime_truth_free,
                "hard_gate_pass": hard_gate,
                "pass_count": pass_count,
                "expected_run_count": expected,
                "axis_pass_count": axis_pass,
                "target_error_limit_pass": target_error <= target_error_limit_m,
                "target_max_abs_error_corrected_max_m": candidate.get("target_max_abs_error_corrected_max_m", ""),
                "validation_range_rmse_corrected_mean_m": candidate.get("validation_range_rmse_corrected_mean_m", ""),
                "validation_range_rmse_corrected_max_m": candidate.get("validation_range_rmse_corrected_max_m", ""),
                "profile_sha256": candidate.get("profile_sha256", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_truth_used": False,
            }
        )
    return rows


def next_actions(family_rows: Sequence[Mapping[str, object]], candidate_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    actions: list[dict[str, object]] = []
    for row in family_rows:
        if row.get("status") != "PASS":
            actions.append(
                {
                    "priority": "high",
                    "action": "add_method_family",
                    "target": row.get("family", ""),
                    "reason": "required search family is missing from the catalog",
                }
            )
    passing = [row for row in candidate_rows if row.get("status") == "PASS"]
    if not passing:
        actions.append(
            {
                "priority": "high",
                "action": "continue_global_search",
                "target": "all_families",
                "reason": "no deployable truth-free method passed the hard gate",
            }
        )
    else:
        best = min(
            passing,
            key=lambda row: (
                finite_float(row.get("validation_range_rmse_corrected_mean_m")),
                finite_float(row.get("target_max_abs_error_corrected_max_m")),
                finite_float(row.get("validation_range_rmse_corrected_max_m")),
                str(row.get("method", "")),
            ),
        )
        actions.append(
            {
                "priority": "medium",
                "action": "exploit_near_best_family",
                "target": best.get("method", ""),
                "reason": "hard-gated best candidate exists; continue local sweeps around its profile while keeping alternates",
            }
        )
    return actions


def audit_catalog(
    *,
    matrix_summary_json: Path,
    policy_selection_json: Path,
    bundle_manifest_json: Path | None,
    min_method_count: int,
    target_error_limit_m: float,
) -> dict[str, object]:
    matrix = load_json(matrix_summary_json)
    selection = load_json(policy_selection_json)
    bundle = load_json(bundle_manifest_json) if bundle_manifest_json is not None and bundle_manifest_json.is_file() else {}
    matrix_methods = [method_name(row) for row in matrix.get("methods", []) if isinstance(row, Mapping)]
    selection_methods = [method_name(row) for row in selection.get("candidates", []) if isinstance(row, Mapping)]
    methods = sorted({method for method in matrix_methods + selection_methods if method})
    family_rows = build_family_rows(methods)
    candidate_rows = candidate_status_rows(selection, target_error_limit_m)
    passing_candidates = [row for row in candidate_rows if row.get("status") == "PASS"]
    required_family_pass = all(row.get("status") == "PASS" for row in family_rows)
    method_count_pass = len(methods) >= min_method_count
    matrix_status_pass = matrix.get("status") == "PASS"
    selection_status_pass = selection.get("status") == "PASS"
    bundle_status_pass = not bundle or bundle.get("status") == "PASS"
    runtime_truth_used = bool(matrix.get("runtime_truth_used", True)) or bool(selection.get("runtime_truth_used", True)) or bool(bundle.get("runtime_truth_used", False))
    coordinate_contract_pass = (
        matrix.get("axis_contract") == RDF_AXIS_CONTRACT
        and selection.get("axis_contract") == RDF_AXIS_CONTRACT
        and (not bundle or bundle.get("axis_contract") == RDF_AXIS_CONTRACT)
        and selection.get("public_topic_contract") == PUBLIC_TOPIC_CONTRACT
    )
    status = "PASS" if (
        method_count_pass
        and required_family_pass
        and matrix_status_pass
        and selection_status_pass
        and bundle_status_pass
        and passing_candidates
        and not runtime_truth_used
        and coordinate_contract_pass
    ) else "FAIL"
    actions = next_actions(family_rows, candidate_rows)
    return {
        "mode": "rtk_stereo_method_search_catalog_audit",
        "status": status,
        "matrix_summary_json": str(matrix_summary_json),
        "policy_selection_json": str(policy_selection_json),
        "bundle_manifest_json": str(bundle_manifest_json or ""),
        "method_count": len(methods),
        "min_method_count": min_method_count,
        "method_count_pass": method_count_pass,
        "required_family_pass": required_family_pass,
        "hard_gate_pass_count": len(passing_candidates),
        "matrix_status": matrix.get("status", ""),
        "policy_selection_status": selection.get("status", ""),
        "bundle_status": bundle.get("status", "") if bundle else "",
        "selected_method": selection.get("recommended_method", bundle.get("selected_method", "")),
        "selected_policy": selection.get("recommended_policy", bundle.get("selected_policy", "")),
        "target_error_limit_m": target_error_limit_m,
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contract_pass": coordinate_contract_pass,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "families": family_rows,
        "candidates": candidate_rows,
        "next_actions": actions,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "RTK stereo method-search catalog audit",
        f"status: {payload.get('status', '')}",
        f"method_count: {payload.get('method_count', '')}/{payload.get('min_method_count', '')}",
        f"required_family_pass: {payload.get('required_family_pass', '')}",
        f"hard_gate_pass_count: {payload.get('hard_gate_pass_count', '')}",
        f"selected_policy: {payload.get('selected_policy', '')}",
        f"selected_method: {payload.get('selected_method', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"coordinate_contract_pass: {payload.get('coordinate_contract_pass', '')}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        "",
        "Families:",
    ]
    for row in payload.get("families", []):
        if isinstance(row, Mapping):
            lines.append(f"- {row.get('family', '')}: {row.get('status', '')} examples={row.get('examples', '')}")
    lines.append("")
    lines.append("Next actions:")
    for row in payload.get("next_actions", []):
        if isinstance(row, Mapping):
            lines.append(f"- {row.get('priority', '')}: {row.get('action', '')} target={row.get('target', '')} reason={row.get('reason', '')}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = audit_catalog(
        matrix_summary_json=args.matrix_summary_json,
        policy_selection_json=args.policy_selection_json,
        bundle_manifest_json=args.bundle_manifest_json,
        min_method_count=args.min_method_count,
        target_error_limit_m=args.target_error_limit_m,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.output_dir / "rtk_stereo_method_search_catalog_audit.json", payload)
    write_csv(args.output_dir / "rtk_stereo_method_search_family_coverage.csv", payload.get("families", []))
    write_csv(args.output_dir / "rtk_stereo_method_search_candidate_status.csv", payload.get("candidates", []))
    write_csv(args.output_dir / "rtk_stereo_method_search_next_actions.csv", payload.get("next_actions", []))
    write_text(args.output_dir / "rtk_stereo_method_search_catalog_audit.txt", payload)
    return payload


def main() -> int:
    payload = run()
    compact = {
        key: payload.get(key)
        for key in (
            "status",
            "method_count",
            "min_method_count",
            "required_family_pass",
            "hard_gate_pass_count",
            "selected_policy",
            "selected_method",
            "runtime_truth_used",
            "coordinate_contract_pass",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
