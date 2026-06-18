#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


REQUIRED_SAMPLE_COLUMNS = {
    "rdf_axis_contract",
    "truth_rdf_transform_contract",
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "left_u_px",
    "left_v_px",
    "measured_disparity_px",
}

REQUIRED_DELIVERABLE_ROLES = {
    "method_comparison_csv",
    "range_error_plot_png",
    "residual_disparity_plot_png",
    "rdf_axis_rmse_plot_png",
    "target_1852m_stats_csv",
    "summary_txt",
    "validation_predictions_csv",
}

METHOD_FAMILY_HINTS = {
    "global_rotation": (("global_rotation",),),
    "global_scale_rotation": (("global_scale_rotation",),),
    "sim3": (("sim3",),),
    "linear_disparity_or_inverse_depth": (("disparity",), ("inverse_depth",)),
    "quadratic_or_lut_residual": (("quadratic",), ("lut",), ("binned",)),
    "sim3_residual_combination": (("sim3", "residual"),),
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a requirement-by-requirement acceptance checklist for the RTK stereo calibration goal."
    )
    parser.add_argument("--suite-summary-json", type=Path, required=True)
    parser.add_argument("--deliverables-json", type=Path, required=True)
    parser.add_argument("--candidate-package-dir", type=Path, required=True)
    parser.add_argument("--readme", type=Path, default=Path("Simulation/README.md"))
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-txt", type=Path)
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


def read_csv_header(path: Path) -> set[str]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        return set(next(reader, []))


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite_float(value: object, default: float = float("inf")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed == parsed else default


def status_row(requirement_id: str, description: str, passed: bool, evidence: str, detail: object = "") -> dict[str, object]:
    return {
        "requirement_id": requirement_id,
        "status": "PASS" if passed else "FAIL",
        "description": description,
        "evidence": evidence,
        "detail": detail,
    }


def pipeline_file_path(deliverables: Mapping[str, object], role: str) -> Path:
    files = deliverables.get("pipeline_files", {})
    if not isinstance(files, Mapping):
        return Path("")
    row = files.get(role, {})
    if not isinstance(row, Mapping):
        return Path("")
    return Path(str(row.get("path", "")))


def method_family_coverage(method_rows: Sequence[Mapping[str, str]]) -> dict[str, bool]:
    names = [str(row.get("method", "")).lower() for row in method_rows]
    coverage: dict[str, bool] = {}
    for family, alternatives in METHOD_FAMILY_HINTS.items():
        coverage[family] = any(all(hint in name for hint in hints) for name in names for hints in alternatives)
    return coverage


def check_acceptance(
    *,
    suite_summary_json: Path,
    deliverables_json: Path,
    candidate_package_dir: Path,
    readme: Path,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    suite = load_json(suite_summary_json)
    deliverables = load_json(deliverables_json)
    manifest_path = candidate_package_dir / "deployable_profile_candidate_package_manifest.json"
    manifest = load_json(manifest_path) if manifest_path.is_file() else {}
    readme_text = readme.read_text(encoding="utf-8") if readme.is_file() else ""

    sample_csv = Path(str(suite.get("sample_csv", "")))
    method_csv = pipeline_file_path(deliverables, "method_comparison_csv")
    validation_csv = pipeline_file_path(deliverables, "validation_predictions_csv")
    method_rows = read_csv_rows(method_csv) if method_csv.is_file() else []
    sample_header = read_csv_header(sample_csv) if sample_csv.is_file() else set()
    validation_header = read_csv_header(validation_csv) if validation_csv.is_file() else set()
    coverage = method_family_coverage(method_rows)

    policies = suite.get("policies", [])
    policy_runs = suite.get("policy_runs", [])
    pipeline_files = deliverables.get("pipeline_files", {})
    all_pipeline_files_present = (
        isinstance(pipeline_files, Mapping)
        and all(
            role in pipeline_files
            and isinstance(pipeline_files[role], Mapping)
            and bool(pipeline_files[role].get("exists"))
            and int(pipeline_files[role].get("size_bytes", 0) or 0) > 0
            for role in REQUIRED_DELIVERABLE_ROLES
        )
    )
    selected_rmse = finite_float(deliverables.get("validation_range_rmse_m"))
    raw_rmse = finite_float(deliverables.get("raw_validation_range_rmse_m"))
    target_error = finite_float(deliverables.get("target_1852m_max_abs_error_m"))
    package_target_error = finite_float(manifest.get("target_max_abs_error_corrected_max_m"))

    rows = [
        status_row(
            "REQ-01-truth-source-offline",
            "Truth source data is represented as calibration/evaluation evidence only and must be converted to stereo-center RDF before evaluation.",
            "T_source_to_stereo_rdf" in readme_text
            and "truth_scope" in deliverables
            and not bool(suite.get("runtime_truth_used", True)),
            str(readme),
            deliverables.get("truth_scope", ""),
        ),
        status_row(
            "REQ-02-stereo-rdf-observation",
            "Stereo-center OpenCV optical RDF is the internal measurement frame.",
            suite.get("axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)"
            and deliverables.get("axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)"
            and "rdf_axis_contract" in sample_header,
            str(sample_csv),
            sorted(sample_header & {"rdf_axis_contract", "raw_right_m", "raw_down_m", "raw_forward_m"}),
        ),
        status_row(
            "REQ-03-bbox-disparity-inputs",
            "Stereo-center RDF truth, raw RDF, left synthetic/BBox center, measured disparity, and explicit truth transform contract are present.",
            REQUIRED_SAMPLE_COLUMNS.issubset(sample_header),
            str(sample_csv),
            sorted(REQUIRED_SAMPLE_COLUMNS - sample_header),
        ),
        status_row(
            "REQ-04-error-injection-synthetic-mode",
            "Synthetic mode and injected observation/camera/disparity stress are documented and represented in the run root.",
            "synthetic mode" in readme_text.lower()
            and "right_yaw_error_deg" in readme_text
            and "measured_disparity_px" in readme_text,
            str(readme),
            "synthetic mode, camera attitude/focal/baseline/disparity stress documentation",
        ),
        status_row(
            "REQ-05-method-family-coverage",
            "Multiple correction families were explored, including rotation/scale/Sim3/disparity/residual combinations.",
            all(coverage.values()) and len(method_rows) >= 6,
            str(method_csv),
            coverage,
        ),
        status_row(
            "REQ-06-train-validation-split",
            "Train/validation split evidence exists and validation predictions are recorded.",
            {"method", "split", "truth_right_m", "pred_right_m", "measured_disparity_px"}.issubset(validation_header)
            and "train" in readme_text.lower()
            and "validation" in readme_text.lower(),
            str(validation_csv),
            sorted(validation_header & {"method", "split", "truth_right_m", "pred_right_m", "measured_disparity_px"}),
        ),
        status_row(
            "REQ-07-validation-metrics-pass",
            "Validation range RMSE improves over raw and 1852m target error is within limit.",
            selected_rmse < raw_rmse and target_error <= target_error_limit_m and package_target_error <= target_error_limit_m,
            str(deliverables_json),
            {
                "selected_validation_range_rmse_m": selected_rmse,
                "raw_validation_range_rmse_m": raw_rmse,
                "pooled_target_1852m_max_abs_error_m": target_error,
                "common_csv_target_max_abs_error_m": package_target_error,
            },
        ),
        status_row(
            "REQ-08-required-artifacts",
            "Required CSV, PNG, and summary artifacts are present.",
            all_pipeline_files_present,
            str(deliverables_json),
            sorted(REQUIRED_DELIVERABLE_ROLES),
        ),
        status_row(
            "REQ-09-package-runtime-ready",
            "Selected profile package, operator dry-run, portability, install rehearsal, and public payload checks pass.",
            suite.get("status") == "PASS"
            and suite.get("selected_install_status") == "PASS"
            and suite.get("selected_deliverables_status") == "PASS"
            and manifest.get("public_topic_payload_status") == "PASS"
            and manifest.get("public_topic_payload_node_equivalence_status") == "PASS",
            str(suite_summary_json),
            {
                "suite_status": suite.get("status"),
                "selected_install_status": suite.get("selected_install_status"),
                "selected_deliverables_status": suite.get("selected_deliverables_status"),
                "public_topic_payload_status": manifest.get("public_topic_payload_status"),
            },
        ),
        status_row(
            "REQ-10-interface-separation",
            "Future real PX4, image, and YOLO/BBox topic replacement interfaces are documented separately from synthetic mode.",
            "/fmu/out/vehicle_local_position" in readme_text
            and "camera image" in readme_text.lower()
            and "YOLO BBox" in readme_text
            and "runtime input" in readme_text,
            str(readme),
            "real topic bridge documentation",
        ),
        status_row(
            "REQ-11-final-selection-policy",
            "Final recommendation is based on validation/package readiness rather than train RMSE alone.",
            suite.get("selected_policy") == "range_priority"
            and isinstance(policies, list)
            and len(policies) >= 4
            and isinstance(policy_runs, list)
            and all(isinstance(row, Mapping) and row.get("status") == "PASS" for row in policy_runs),
            str(suite_summary_json),
            {
                "selected_policy": suite.get("selected_policy"),
                "selected_method": suite.get("selected_method"),
                "policy_count": suite.get("policy_count"),
                "pass_count": suite.get("pass_count"),
            },
        ),
    ]

    status = "PASS" if all(row["status"] == "PASS" for row in rows) else "FAIL"
    return {
        "mode": "rtk_stereo_goal_acceptance_check",
        "status": status,
        "suite_summary_json": str(suite_summary_json),
        "deliverables_json": str(deliverables_json),
        "candidate_package_dir": str(candidate_package_dir),
        "readme": str(readme),
        "selected_policy": suite.get("selected_policy", ""),
        "selected_method": suite.get("selected_method", ""),
        "target_error_limit_m": target_error_limit_m,
        "pass_count": sum(1 for row in rows if row["status"] == "PASS"),
        "requirement_count": len(rows),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "requirements": rows,
        "failures": [row["requirement_id"] for row in rows if row["status"] != "PASS"],
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "RTK stereo goal acceptance checklist",
        f"status: {payload.get('status', '')}",
        f"selected_policy: {payload.get('selected_policy', '')}",
        f"selected_method: {payload.get('selected_method', '')}",
        f"pass_count: {payload.get('pass_count', '')}/{payload.get('requirement_count', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        "",
        "Requirements:",
    ]
    for row in payload.get("requirements", []):
        if isinstance(row, Mapping):
            lines.append(f"- {row.get('requirement_id', '')}: {row.get('status', '')} - {row.get('description', '')}")
            lines.append(f"  evidence: {row.get('evidence', '')}")
            detail = row.get("detail", "")
            if detail not in ("", None, [], {}):
                lines.append(f"  detail: {detail}")
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = check_acceptance(
        suite_summary_json=args.suite_summary_json,
        deliverables_json=args.deliverables_json,
        candidate_package_dir=args.candidate_package_dir,
        readme=args.readme,
        target_error_limit_m=args.target_error_limit_m,
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_csv is not None:
        requirements = payload.get("requirements", [])
        write_csv(args.output_csv, requirements if isinstance(requirements, list) else [])
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> int:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
