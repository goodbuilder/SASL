#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path
from typing import Mapping, Sequence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a lightweight package for a policy-selected StereoDistance active-profile candidate. "
            "This package is an evidence bundle for the next acceptance-dashboard active profile."
        )
    )
    parser.add_argument("--candidate-readiness-json", type=Path, required=True)
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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_path(anchor: Path, value: object) -> Path:
    path = Path(str(value))
    if path.is_absolute() or path.exists():
        return path
    return anchor.parent / path


def record_file(path: Path, package_root: Path, role: str) -> dict[str, object]:
    return {
        "role": role,
        "path": str(path.relative_to(package_root)),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def copy_file(src: Path, dst: Path, package_root: Path, role: str) -> dict[str, object]:
    if not src.is_file():
        raise FileNotFoundError(f"{role} missing: {src}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return record_file(dst, package_root, role)


def write_package_readme(path: Path, manifest: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance Deployable Profile Candidate Package",
        "",
        "This package contains the policy-selected active profile candidate and the evidence used to promote it",
        "to the next acceptance-dashboard run.",
        "",
        "Deployable file:",
        "- active_runtime_profile.json",
        "",
        "Runtime input contract:",
        "- stereo-center RDF: right_m, down_m, forward_m",
        "- measured disparity in pixels",
        "- optional left image center: left_u_px, left_v_px",
        "- do not provide PX4/RTK/NED truth at runtime",
        "",
        "Public topic contract:",
        "- /stereo/corrected_position Float32MultiArray data=[forward_m,right_m,down_m,range_m]",
        "",
        f"method: {manifest.get('method', '')}",
        f"recommended_policy: {manifest.get('recommended_policy', '')}",
        f"active_profile_sha256: {manifest.get('active_profile_sha256', '')}",
        "",
        "Coordinate contract:",
        str(manifest.get("coordinate_contract", "")),
        "",
        "Evidence files:",
        "- evidence/deployable_profile_candidate_readiness.json",
        "- evidence/deployable_profile_candidate_export.json",
        "- evidence/deployable_profile_policy_selection.json",
        "- evidence/promotion_check.json",
        "- evidence/public_corrected_position_payload_summary.json",
        "- evidence/public_corrected_position_payload.csv",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_package(*, candidate_readiness_json: Path, output_dir: Path) -> dict[str, object]:
    readiness = load_json(candidate_readiness_json)
    if readiness.get("status") != "PASS":
        raise ValueError("candidate readiness did not pass")
    candidate_export = resolve_path(candidate_readiness_json, readiness.get("candidate_export_json", ""))
    candidate = load_json(candidate_export)
    active_profile = resolve_path(candidate_export, candidate.get("output_active_profile", ""))
    promotion_check = resolve_path(candidate_export, candidate.get("promotion_check_json", ""))
    public_payload_json = resolve_path(candidate_export, candidate.get("public_topic_payload_json", ""))
    public_payload_csv = resolve_path(candidate_export, candidate.get("public_topic_payload_csv", ""))
    policy_selection = resolve_path(candidate_export, candidate.get("policy_selection_json", ""))

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []
    records.append(copy_file(active_profile, output_dir / "active_runtime_profile.json", output_dir, "active_profile"))
    records.append(
        copy_file(candidate_readiness_json, output_dir / "evidence" / "deployable_profile_candidate_readiness.json", output_dir, "candidate_readiness")
    )
    records.append(
        copy_file(candidate_export, output_dir / "evidence" / "deployable_profile_candidate_export.json", output_dir, "candidate_export")
    )
    records.append(
        copy_file(policy_selection, output_dir / "evidence" / "deployable_profile_policy_selection.json", output_dir, "policy_selection")
    )
    records.append(copy_file(promotion_check, output_dir / "evidence" / "promotion_check.json", output_dir, "promotion_check"))
    records.append(
        copy_file(
            public_payload_json,
            output_dir / "evidence" / "public_corrected_position_payload_summary.json",
            output_dir,
            "public_topic_payload_summary",
        )
    )
    records.append(
        copy_file(
            public_payload_csv,
            output_dir / "evidence" / "public_corrected_position_payload.csv",
            output_dir,
            "public_topic_payload_csv",
        )
    )
    active_sha = sha256_file(output_dir / "active_runtime_profile.json")
    manifest = {
        "mode": "deployable_profile_candidate_package",
        "status": "PASS",
        "active_profile": "active_runtime_profile.json",
        "active_profile_sha256": active_sha,
        "method": readiness.get("method", ""),
        "recommended_policy": readiness.get("recommended_policy", ""),
        "candidate_readiness_status": readiness.get("status", ""),
        "candidate_pass_count": readiness.get("candidate_pass_count", ""),
        "candidate_expected_run_count": readiness.get("candidate_expected_run_count", ""),
        "candidate_axis_pass_count": readiness.get("candidate_axis_pass_count", ""),
        "candidate_target_pass_count": readiness.get("candidate_target_pass_count", ""),
        "validation_range_rmse_corrected_mean_m": readiness.get("validation_range_rmse_corrected_mean_m", ""),
        "validation_range_rmse_corrected_max_m": readiness.get("validation_range_rmse_corrected_max_m", ""),
        "target_max_abs_error_corrected_max_m": readiness.get("target_max_abs_error_corrected_max_m", ""),
        "promotion_check_pass": readiness.get("promotion_check_pass", ""),
        "promotion_sample_apply_count": readiness.get("promotion_sample_apply_count", ""),
        "promotion_sample_invalid_count": readiness.get("promotion_sample_invalid_count", ""),
        "public_topic_payload_status": readiness.get("public_topic_payload_status", ""),
        "public_topic_payload_sample_count": readiness.get("public_topic_payload_sample_count", ""),
        "public_topic_payload_node_equivalence_status": readiness.get("public_topic_payload_node_equivalence_status", ""),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "coordinate_contract": (
            "runtime active profile consumes stereo-center RDF(right,down,forward) plus measured disparity; "
            "PX4/RTK/NED truth remains calibration/evaluation evidence only; public StereoDistance output uses FRD+range"
        ),
        "file_count": 0,
        "files": [],
    }
    readme = output_dir / "DEPLOYABLE_PROFILE_CANDIDATE_README.txt"
    write_package_readme(readme, manifest)
    records.append(record_file(readme, output_dir, "package_readme"))
    manifest["file_count"] = len(records)
    manifest["files"] = records
    write_json(output_dir / "deployable_profile_candidate_package_manifest.json", manifest)
    return manifest


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_package(candidate_readiness_json=args.candidate_readiness_json, output_dir=args.output_dir)


def main() -> int:
    result = run()
    compact = {
        key: result.get(key)
        for key in (
            "status",
            "method",
            "recommended_policy",
            "active_profile_sha256",
            "candidate_pass_count",
            "candidate_expected_run_count",
            "runtime_truth_used",
            "file_count",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
