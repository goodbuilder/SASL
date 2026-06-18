#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


REQUIRED_PIPELINE_FILES = {
    "method_comparison_csv": "method_comparison.csv",
    "range_error_plot_png": "range_error_before_after.png",
    "residual_disparity_plot_png": "residual_disparity_by_distance.png",
    "rdf_axis_rmse_plot_png": "rdf_axis_rmse.png",
    "target_1852m_stats_csv": "target_1852m_stats.csv",
    "summary_txt": "summary.txt",
    "validation_predictions_csv": "validation_predictions.csv",
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit that the deployable StereoDistance profile candidate is backed by the required CSV, PNG, "
            "summary, package, operator, portability, install, and coordinate-contract evidence."
        )
    )
    parser.add_argument("--pipeline-dir", type=Path, required=True)
    parser.add_argument("--package-summary-json", type=Path, required=True)
    parser.add_argument("--candidate-package-dir", type=Path, required=True)
    parser.add_argument("--install-manifest-json", type=Path)
    parser.add_argument("--output-json", type=Path)
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


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def finite_float(value: object, default: float = float("inf")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    if parsed != parsed:
        return default
    return parsed


def file_status(path: Path) -> dict[str, object]:
    return {
        "path": str(path),
        "exists": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else 0,
    }


def selected_candidate(summary: Mapping[str, object], package_dir: Path) -> dict[str, object]:
    candidates = summary.get("candidates", [])
    if not isinstance(candidates, list):
        return {}
    package_dir_str = str(package_dir)
    for row in candidates:
        if isinstance(row, Mapping) and str(row.get("package_dir", "")) == package_dir_str:
            return dict(row)
    selected_dir = str(summary.get("selected_package_dir", ""))
    for row in candidates:
        if isinstance(row, Mapping) and str(row.get("package_dir", "")) == selected_dir:
            return dict(row)
    return dict(candidates[0]) if candidates and isinstance(candidates[0], Mapping) else {}


def check_deliverables(
    *,
    pipeline_dir: Path,
    package_summary_json: Path,
    candidate_package_dir: Path,
    install_manifest_json: Path | None = None,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    failures: list[str] = []
    pipeline_dir = pipeline_dir.resolve()
    candidate_package_dir = candidate_package_dir.resolve()
    package_summary_json = package_summary_json.resolve()
    install_manifest_json = install_manifest_json.resolve() if install_manifest_json is not None else None

    require(pipeline_dir.is_dir(), failures, "pipeline_dir:missing")
    require(package_summary_json.is_file(), failures, "package_summary_json:missing")
    require(candidate_package_dir.is_dir(), failures, "candidate_package_dir:missing")

    pipeline_files = {role: pipeline_dir / rel for role, rel in REQUIRED_PIPELINE_FILES.items()}
    pipeline_file_status = {role: file_status(path) for role, path in pipeline_files.items()}
    for role, status in pipeline_file_status.items():
        require(bool(status["exists"]), failures, f"pipeline_file:{role}:missing")
        require(int(status["size_bytes"]) > 0, failures, f"pipeline_file:{role}:empty")

    package_summary = load_json(package_summary_json) if package_summary_json.is_file() else {}
    require(package_summary.get("status") == "PASS", failures, "package_summary:status")
    require(not bool(package_summary.get("runtime_truth_used", True)), failures, "package_summary:runtime_truth_used")
    require("RDF(right,down,forward)" in str(package_summary.get("axis_contract", "")), failures, "package_summary:axis_contract")
    require("FRD_range(forward,right,down,range)" in str(package_summary.get("public_topic_contract", "")), failures, "package_summary:public_topic_contract")

    candidate = selected_candidate(package_summary, candidate_package_dir)
    selected_method = str(package_summary.get("selected_method", candidate.get("method", "")))
    require(bool(selected_method), failures, "selected_method:missing")
    require(candidate.get("status") == "PASS", failures, "candidate:status")
    require(candidate.get("package_check_status") == "PASS", failures, "candidate:package_check")
    require(candidate.get("operator_view_launch_status") == "PASS", failures, "candidate:operator_view")
    require(candidate.get("portability_status") in ("PASS", ""), failures, "candidate:portability")
    require(not bool(candidate.get("runtime_truth_used", True)), failures, "candidate:runtime_truth_used")
    target_max = finite_float(candidate.get("target_max_abs_error_corrected_max_m"))
    require(target_max <= target_error_limit_m, failures, "candidate:target_error_limit")

    manifest_path = candidate_package_dir / "deployable_profile_candidate_package_manifest.json"
    package_check_path = candidate_package_dir / "deployable_profile_candidate_package_check.json"
    operator_check_path = candidate_package_dir / "operator_view_launch_check.json"
    portability_path = candidate_package_dir / "deployable_profile_candidate_package_portability.json"
    manifest = load_json(manifest_path) if manifest_path.is_file() else {}
    package_check = load_json(package_check_path) if package_check_path.is_file() else {}
    operator_check = load_json(operator_check_path) if operator_check_path.is_file() else {}
    portability = load_json(portability_path) if portability_path.is_file() else {}

    require(manifest.get("status") == "PASS", failures, "candidate_manifest:status")
    require(manifest.get("method") == selected_method, failures, "candidate_manifest:method")
    require(not bool(manifest.get("runtime_truth_used", True)), failures, "candidate_manifest:runtime_truth_used")
    require("RDF(right,down,forward)" in str(manifest.get("axis_contract", "")), failures, "candidate_manifest:axis_contract")
    require("FRD_range(forward,right,down,range)" in str(manifest.get("public_topic_contract", "")), failures, "candidate_manifest:public_topic_contract")
    require(package_check.get("status") == "PASS", failures, "package_check:status")
    require(operator_check.get("status") == "PASS", failures, "operator_check:status")
    require(portability.get("status") == "PASS", failures, "portability:status")

    method_rows = read_csv_rows(pipeline_files["method_comparison_csv"]) if pipeline_files["method_comparison_csv"].is_file() else []
    selected_method_rows = [row for row in method_rows if row.get("method") == selected_method]
    raw_rows = [row for row in method_rows if row.get("method") == "raw"]
    require(bool(selected_method_rows), failures, "method_comparison:selected_method")
    require(bool(raw_rows), failures, "method_comparison:raw")
    selected_method_row = selected_method_rows[0] if selected_method_rows else {}
    raw_row = raw_rows[0] if raw_rows else {}
    selected_validation_rmse = finite_float(selected_method_row.get("validation_range_rmse_m"))
    raw_validation_rmse = finite_float(raw_row.get("validation_range_rmse_m"))
    require(selected_validation_rmse < raw_validation_rmse, failures, "method_comparison:range_rmse_improved")

    target_rows = read_csv_rows(pipeline_files["target_1852m_stats_csv"]) if pipeline_files["target_1852m_stats_csv"].is_file() else []
    selected_target_rows = [row for row in target_rows if row.get("method") == selected_method]
    require(bool(selected_target_rows), failures, "target_1852m:selected_method")
    selected_target_row = selected_target_rows[0] if selected_target_rows else {}
    require(finite_float(selected_target_row.get("max_abs_error_m")) <= target_error_limit_m, failures, "target_1852m:max_abs_error")
    require(selected_target_row.get("pass_abs_mean_within_10m") in ("yes", "true", "True", "PASS"), failures, "target_1852m:pass_abs_mean")

    predictions_rows = read_csv_rows(pipeline_files["validation_predictions_csv"]) if pipeline_files["validation_predictions_csv"].is_file() else []
    require(any(row.get("method") == selected_method and row.get("split") == "validation" for row in predictions_rows), failures, "validation_predictions:selected_validation_rows")
    require(
        all(row.get("rdf_axis_contract") == "stereo_center_opencv_optical:RDF(right,down,forward)" for row in predictions_rows[:10]),
        failures,
        "validation_predictions:axis_contract",
    )

    summary_text = pipeline_files["summary_txt"].read_text(encoding="utf-8") if pipeline_files["summary_txt"].is_file() else ""
    require("RDF contract: stereo_center_opencv_optical:RDF(right,down,forward)" in summary_text, failures, "summary_txt:rdf_contract")
    require("NED contract: px4_local_position_ned:north,east,down" in summary_text, failures, "summary_txt:ned_contract")

    install_manifest = load_json(install_manifest_json) if install_manifest_json is not None and install_manifest_json.is_file() else {}
    if install_manifest_json is not None:
        require(install_manifest.get("status") == "PASS", failures, "install_manifest:status")
        require(install_manifest.get("package_type") == "candidate_package", failures, "install_manifest:package_type")
        require(install_manifest.get("package_check_status") == "PASS", failures, "install_manifest:package_check")
        require(install_manifest.get("portability_check_status") == "PASS", failures, "install_manifest:portability")
        require(install_manifest.get("installed_profile_sha256") == manifest.get("active_profile_sha256"), failures, "install_manifest:sha256")
        require(not bool(install_manifest.get("runtime_truth_used", True)), failures, "install_manifest:runtime_truth_used")

    status = "PASS" if not failures else "FAIL"
    return {
        "mode": "deployable_profile_deliverables_check",
        "status": status,
        "pipeline_dir": str(pipeline_dir),
        "package_summary_json": str(package_summary_json),
        "candidate_package_dir": str(candidate_package_dir),
        "install_manifest_json": str(install_manifest_json) if install_manifest_json is not None else "",
        "selected_method": selected_method,
        "selected_policy": package_summary.get("selected_policy", ""),
        "validation_range_rmse_m": selected_validation_rmse,
        "raw_validation_range_rmse_m": raw_validation_rmse,
        "target_1852m_max_abs_error_m": finite_float(selected_target_row.get("max_abs_error_m")) if selected_target_row else target_max,
        "target_error_limit_m": target_error_limit_m,
        "pipeline_files": pipeline_file_status,
        "package_status": manifest.get("status", ""),
        "package_check_status": package_check.get("status", ""),
        "operator_view_launch_status": operator_check.get("status", ""),
        "portability_status": portability.get("status", ""),
        "install_status": install_manifest.get("status", "") if install_manifest else "",
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "truth_scope": "PX4/RTK/NED truth is calibration/evaluation evidence only; runtime input remains raw stereo-center RDF plus measured disparity.",
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Deployable profile deliverables check",
        f"status: {payload.get('status', '')}",
        f"selected_policy: {payload.get('selected_policy', '')}",
        f"selected_method: {payload.get('selected_method', '')}",
        f"validation_range_rmse_m: {payload.get('validation_range_rmse_m', '')}",
        f"raw_validation_range_rmse_m: {payload.get('raw_validation_range_rmse_m', '')}",
        f"target_1852m_max_abs_error_m: {payload.get('target_1852m_max_abs_error_m', '')}",
        f"target_error_limit_m: {payload.get('target_error_limit_m', '')}",
        f"package_status: {payload.get('package_status', '')}",
        f"package_check_status: {payload.get('package_check_status', '')}",
        f"operator_view_launch_status: {payload.get('operator_view_launch_status', '')}",
        f"portability_status: {payload.get('portability_status', '')}",
        f"install_status: {payload.get('install_status', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        f"truth_scope: {payload.get('truth_scope', '')}",
        f"failure_count: {payload.get('failure_count', '')}",
        "",
        "Pipeline files:",
    ]
    pipeline_files = payload.get("pipeline_files", {})
    if isinstance(pipeline_files, Mapping):
        for role, status in pipeline_files.items():
            if isinstance(status, Mapping):
                lines.append(f"- {role}: exists={status.get('exists', '')} size_bytes={status.get('size_bytes', '')} path={status.get('path', '')}")
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    result = check_deliverables(
        pipeline_dir=args.pipeline_dir,
        package_summary_json=args.package_summary_json,
        candidate_package_dir=args.candidate_package_dir,
        install_manifest_json=args.install_manifest_json,
        target_error_limit_m=args.target_error_limit_m,
    )
    if args.output_json is not None:
        write_json(args.output_json, result)
    if args.output_txt is not None:
        write_text(args.output_txt, result)
    return result


def main() -> int:
    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
