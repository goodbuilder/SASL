#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path
from typing import Mapping, Sequence


REQUIRED_PIPELINE_ROLES = {
    "method_comparison_csv",
    "range_error_plot_png",
    "residual_disparity_plot_png",
    "rdf_axis_rmse_plot_png",
    "target_1852m_stats_csv",
    "summary_txt",
    "validation_predictions_csv",
}

PACKAGE_FILES = {
    "active_runtime_profile_json": "active_runtime_profile.json",
    "candidate_package_manifest_json": "deployable_profile_candidate_package_manifest.json",
    "candidate_package_check_json": "deployable_profile_candidate_package_check.json",
    "candidate_package_check_txt": "deployable_profile_candidate_package_check.txt",
    "candidate_package_portability_json": "deployable_profile_candidate_package_portability.json",
    "candidate_package_portability_txt": "deployable_profile_candidate_package_portability.txt",
    "operator_view_launch_check_json": "operator_view_launch_check.json",
    "operator_view_launch_check_txt": "operator_view_launch_check.txt",
    "candidate_readme_txt": "DEPLOYABLE_PROFILE_CANDIDATE_README.txt",
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a portable final evidence bundle for the RTK stereo calibration acceptance result. "
            "The bundle keeps the selected runtime profile, package checks, acceptance checks, required "
            "CSV/PNG/TXT deliverables, file hashes, and coordinate contracts together."
        )
    )
    parser.add_argument("--goal-acceptance-json", type=Path, required=True)
    parser.add_argument("--suite-summary-json", type=Path, required=True)
    parser.add_argument("--deliverables-json", type=Path, required=True)
    parser.add_argument("--candidate-package-dir", type=Path, required=True)
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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sibling(path: Path, suffix: str) -> Path:
    return path.with_suffix(suffix)


def pipeline_source(deliverables: Mapping[str, object], role: str) -> Path:
    files = deliverables.get("pipeline_files", {})
    if not isinstance(files, Mapping):
        return Path("")
    row = files.get(role, {})
    if not isinstance(row, Mapping):
        return Path("")
    return Path(str(row.get("path", "")))


def copy_record(role: str, source: Path, destination: Path, failures: list[str]) -> dict[str, object]:
    record: dict[str, object] = {
        "role": role,
        "source": str(source),
        "path": str(destination),
        "exists": source.is_file(),
        "size_bytes": 0,
        "sha256": "",
    }
    if not source.is_file():
        failures.append(f"{role}:missing:{source}")
        return record
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    record["size_bytes"] = destination.stat().st_size
    record["sha256"] = sha256_file(destination)
    source_sha = sha256_file(source)
    if record["sha256"] != source_sha:
        failures.append(f"{role}:sha256_mismatch")
    return record


def export_bundle(
    *,
    goal_acceptance_json: Path,
    suite_summary_json: Path,
    deliverables_json: Path,
    candidate_package_dir: Path,
    output_dir: Path,
) -> dict[str, object]:
    goal_acceptance_json = goal_acceptance_json.resolve()
    suite_summary_json = suite_summary_json.resolve()
    deliverables_json = deliverables_json.resolve()
    candidate_package_dir = candidate_package_dir.resolve()
    output_dir = output_dir.resolve()

    goal = load_json(goal_acceptance_json)
    suite = load_json(suite_summary_json)
    deliverables = load_json(deliverables_json)
    package_manifest_path = candidate_package_dir / "deployable_profile_candidate_package_manifest.json"
    package_manifest = load_json(package_manifest_path) if package_manifest_path.is_file() else {}

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    failures: list[str] = []
    files: list[dict[str, object]] = []

    evidence_dir = output_dir / "evidence"
    package_dir = output_dir / "candidate_package"
    artifacts_dir = output_dir / "artifacts"

    for role, source in {
        "goal_acceptance_json": goal_acceptance_json,
        "goal_acceptance_csv": sibling(goal_acceptance_json, ".csv"),
        "goal_acceptance_txt": sibling(goal_acceptance_json, ".txt"),
        "suite_summary_json": suite_summary_json,
        "suite_summary_txt": sibling(suite_summary_json, ".txt"),
        "deliverables_check_json": deliverables_json,
        "deliverables_check_txt": sibling(deliverables_json, ".txt"),
    }.items():
        files.append(copy_record(role, source, evidence_dir / source.name, failures))

    for role, relative in PACKAGE_FILES.items():
        source = candidate_package_dir / relative
        files.append(copy_record(role, source, package_dir / relative, failures))

    evidence_source_dir = candidate_package_dir / "evidence"
    if evidence_source_dir.is_dir():
        for source in sorted(path for path in evidence_source_dir.iterdir() if path.is_file()):
            role = f"candidate_evidence_{source.stem}"
            files.append(copy_record(role, source, package_dir / "evidence" / source.name, failures))

    for role in sorted(REQUIRED_PIPELINE_ROLES):
        source = pipeline_source(deliverables, role)
        files.append(copy_record(role, source, artifacts_dir / source.name, failures))

    status_checks = {
        "goal_acceptance_status": goal.get("status", ""),
        "suite_status": suite.get("status", ""),
        "deliverables_status": deliverables.get("status", ""),
        "candidate_package_status": package_manifest.get("status", ""),
        "selected_install_status": suite.get("selected_install_status", ""),
        "selected_deliverables_status": suite.get("selected_deliverables_status", ""),
        "selected_goal_acceptance_status": suite.get("selected_goal_acceptance_status", ""),
    }
    for key, value in status_checks.items():
        if value != "PASS":
            failures.append(f"{key}:{value}")

    runtime_truth_used = any(
        bool(payload.get("runtime_truth_used", False))
        for payload in (goal, suite, deliverables, package_manifest)
        if isinstance(payload, Mapping)
    )
    if runtime_truth_used:
        failures.append("runtime_truth_used:true")

    manifest = {
        "mode": "rtk_stereo_goal_acceptance_bundle",
        "status": "PASS" if not failures else "FAIL",
        "output_dir": str(output_dir),
        "selected_policy": suite.get("selected_policy", goal.get("selected_policy", "")),
        "selected_method": suite.get("selected_method", goal.get("selected_method", "")),
        "profile_sha256": package_manifest.get("active_profile_sha256", ""),
        "target_error_limit_m": goal.get("target_error_limit_m", ""),
        "validation_range_rmse_m": deliverables.get("validation_range_rmse_m", ""),
        "raw_validation_range_rmse_m": deliverables.get("raw_validation_range_rmse_m", ""),
        "target_1852m_max_abs_error_m": deliverables.get("target_1852m_max_abs_error_m", ""),
        "pass_count": goal.get("pass_count", ""),
        "requirement_count": goal.get("requirement_count", ""),
        "runtime_truth_used": runtime_truth_used,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "truth_scope": "PX4/RTK/NED truth is calibration/evaluation evidence only; runtime input remains raw stereo-center RDF plus measured disparity.",
        "status_checks": status_checks,
        "file_count": len(files),
        "files": files,
        "failure_count": len(failures),
        "failures": failures,
    }

    write_json(output_dir / "rtk_stereo_goal_acceptance_bundle_manifest.json", manifest)
    write_csv(output_dir / "rtk_stereo_goal_acceptance_bundle_files.csv", files)
    write_text(output_dir / "rtk_stereo_goal_acceptance_bundle_summary.txt", manifest)
    return manifest


def write_text(path: Path, manifest: Mapping[str, object]) -> None:
    lines = [
        "RTK stereo goal acceptance bundle",
        f"status: {manifest.get('status', '')}",
        f"selected_policy: {manifest.get('selected_policy', '')}",
        f"selected_method: {manifest.get('selected_method', '')}",
        f"profile_sha256: {manifest.get('profile_sha256', '')}",
        f"validation_range_rmse_m: {manifest.get('validation_range_rmse_m', '')}",
        f"raw_validation_range_rmse_m: {manifest.get('raw_validation_range_rmse_m', '')}",
        f"target_1852m_max_abs_error_m: {manifest.get('target_1852m_max_abs_error_m', '')}",
        f"pass_count: {manifest.get('pass_count', '')}/{manifest.get('requirement_count', '')}",
        f"runtime_truth_used: {manifest.get('runtime_truth_used', '')}",
        f"axis_contract: {manifest.get('axis_contract', '')}",
        f"public_topic_contract: {manifest.get('public_topic_contract', '')}",
        f"truth_scope: {manifest.get('truth_scope', '')}",
        f"file_count: {manifest.get('file_count', '')}",
        f"failure_count: {manifest.get('failure_count', '')}",
        "",
        "Status checks:",
    ]
    status_checks = manifest.get("status_checks", {})
    if isinstance(status_checks, Mapping):
        for key, value in status_checks.items():
            lines.append(f"- {key}: {value}")
    failures = manifest.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_bundle(
        goal_acceptance_json=args.goal_acceptance_json,
        suite_summary_json=args.suite_summary_json,
        deliverables_json=args.deliverables_json,
        candidate_package_dir=args.candidate_package_dir,
        output_dir=args.output_dir,
    )


def main() -> int:
    manifest = run()
    print(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if manifest.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
