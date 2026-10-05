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
        description="Export a deployable StereoDistance active profile package from a validated deployment manifest."
    )
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--manifest-validation-json", type=Path, required=True)
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


def copy_with_record(src: Path, dst: Path, records: list[dict[str, object]], role: str, package_root: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    records.append(
        {
            "role": role,
            "path": str(dst.relative_to(package_root)),
            "size_bytes": dst.stat().st_size,
            "sha256": sha256_file(dst),
        }
    )


def record_file(path: Path, records: list[dict[str, object]], role: str, package_root: Path) -> None:
    records.append(
        {
            "role": role,
            "path": str(path.relative_to(package_root)),
            "size_bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
    )


def write_deployment_readme(path: Path, manifest: Mapping[str, object]) -> None:
    public_payload_enabled = bool(str(manifest.get("public_topic_payload_status", "")).strip())
    runtime_frontier_enabled = bool(str(manifest.get("runtime_deployment_frontier_status", "")).strip())
    lines = [
        "StereoDistance Active Profile Deployment",
        "",
        "Deployable file:",
        "- active_runtime_profile.json",
        "",
        "Use this file as the StereoDistance bbox-center/estimated-to-truth runtime correction profile.",
        "Do not provide PX4/RTK/NED truth, validation errors, or calibration target values at runtime.",
        "",
        "Operator-view launch:",
        "- From the Simulation directory, run:",
        "  ./run_operator_view_with_calibration_profile.sh --package-dir <this deployment_package>",
        "- Equivalent direct launch argument:",
        "  ./run_operator_view.sh --bbox-adjustment <this deployment_package>/active_runtime_profile.json",
        "",
        f"method: {manifest.get('method', '')}",
        f"profile_mode: {manifest.get('profile_mode', '')}",
        f"active_profile_sha256: {manifest.get('active_profile_sha256', '')}",
        "",
        "Runtime input contract:",
        "- stereo-center RDF: right_m, down_m, forward_m",
        "- measured disparity in pixels",
        "- optional left image center: left_u_px, left_v_px",
        "",
        "Coordinate contract:",
        str(manifest.get("coordinate_contract", "")),
        "",
        "Verification files:",
        "- deployment_package_manifest.json",
        "- deployment_package_check.json",
        "- active_profile_deployment_manifest.json",
        "- active_profile_deployment_manifest_validation.json",
        "- evidence/promotion_check.json",
        "- evidence/rtk_stereo_acceptance_matrix_summary.json",
    ]
    if public_payload_enabled:
        lines.extend(
            [
                "- evidence/public_corrected_position_payload_summary.json",
                "- evidence/public_corrected_position_payload.csv",
                "",
                "Public topic contract:",
                "- /stereo/corrected_position std_msgs/Float32MultiArray data=[forward_m,right_m,down_m,range_m]",
                "- public payload evidence must match the real StereoDistanceNode publish path within Float32 tolerance",
            ]
        )
    if runtime_frontier_enabled:
        lines.extend(
            [
                "",
                "Runtime selector/frontier evidence:",
                "- evidence/runtime_profile_label_selector.json",
                "- evidence/package_runtime_profile_label_selector.json",
                "- evidence/runtime_deployment_frontier/runtime_deployment_frontier.json",
                "- evidence/runtime_deployment_frontier/runtime_deployment_frontier_labels.csv",
                "- evidence/runtime_deployment_frontier/runtime_deployment_frontier_profiles.csv",
                "- package_runtime_profile_label_selector.json is package-local and points to ../active_runtime_profile.json",
                "- selector inputs are deployment/scenario labels only, not PX4/RTK/NED truth",
            ]
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_text(path: Path, manifest: Mapping[str, object], records: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance active profile deployment package",
        f"status: {manifest.get('status', '')}",
        f"active_profile: {manifest.get('active_profile', '')}",
        f"active_profile_sha256: {manifest.get('active_profile_sha256', '')}",
        f"method: {manifest.get('method', '')}",
        f"profile_mode: {manifest.get('profile_mode', '')}",
        f"runtime_truth_used: {manifest.get('runtime_truth_used', False)}",
        f"coordinate_contract: {manifest.get('coordinate_contract', '')}",
        "",
        "Files:",
    ]
    for record in records:
        lines.append(
            f"- {record.get('role', '')}: {record.get('path', '')} "
            f"sha256={record.get('sha256', '')} size={record.get('size_bytes', '')}"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_package(
    *,
    manifest_json: Path,
    manifest_validation_json: Path,
    output_dir: Path,
) -> dict[str, object]:
    manifest = load_json(manifest_json)
    validation = load_json(manifest_validation_json)
    if manifest.get("status") != "PASS":
        raise ValueError("deployment manifest did not pass")
    if validation.get("status") != "PASS":
        raise ValueError("deployment manifest validation did not pass")

    active_profile = resolve_path(manifest_json, manifest.get("active_profile", ""))
    promotion_check = resolve_path(manifest_json, manifest.get("promotion_check_json", ""))
    final_acceptance = resolve_path(manifest_json, manifest.get("final_acceptance_summary", ""))
    for path, role in (
        (active_profile, "active_profile"),
        (promotion_check, "promotion_check"),
        (final_acceptance, "final_acceptance_summary"),
        (manifest_json, "deployment_manifest"),
        (manifest_validation_json, "deployment_manifest_validation"),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"{role} missing: {path}")

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []
    copy_with_record(active_profile, output_dir / "active_runtime_profile.json", records, "active_profile", output_dir)
    copy_with_record(promotion_check, output_dir / "evidence" / "promotion_check.json", records, "promotion_check", output_dir)
    copy_with_record(
        final_acceptance,
        output_dir / "evidence" / "rtk_stereo_acceptance_matrix_summary.json",
        records,
        "final_acceptance_summary",
        output_dir,
    )
    public_payload_json_value = str(manifest.get("public_topic_payload_json", "") or "")
    public_payload_csv_value = str(manifest.get("public_topic_payload_csv", "") or "")
    public_payload_txt_value = str(manifest.get("public_topic_payload_txt", "") or "")
    if public_payload_json_value or public_payload_csv_value:
        public_payload_json = resolve_path(manifest_json, public_payload_json_value)
        public_payload_csv = resolve_path(manifest_json, public_payload_csv_value)
        public_payload_txt = resolve_path(manifest_json, public_payload_txt_value) if public_payload_txt_value else None
        for path, role in (
            (public_payload_json, "public_topic_payload_summary"),
            (public_payload_csv, "public_topic_payload_csv"),
        ):
            if not path.is_file():
                raise FileNotFoundError(f"{role} missing: {path}")
        copy_with_record(
            public_payload_json,
            output_dir / "evidence" / "public_corrected_position_payload_summary.json",
            records,
            "public_topic_payload_summary",
            output_dir,
        )
        if public_payload_txt is not None and public_payload_txt.is_file():
            copy_with_record(
                public_payload_txt,
                output_dir / "evidence" / "public_corrected_position_payload_summary.txt",
                records,
                "public_topic_payload_summary_txt",
                output_dir,
            )
        copy_with_record(
            public_payload_csv,
            output_dir / "evidence" / "public_corrected_position_payload.csv",
            records,
            "public_topic_payload_csv",
            output_dir,
        )
    copy_with_record(manifest_json, output_dir / "active_profile_deployment_manifest.json", records, "deployment_manifest", output_dir)
    copy_with_record(
        manifest_validation_json,
        output_dir / "active_profile_deployment_manifest_validation.json",
        records,
        "deployment_manifest_validation",
        output_dir,
    )
    readme_path = output_dir / "DEPLOYMENT_README.txt"
    write_deployment_readme(readme_path, manifest)
    record_file(readme_path, records, "deployment_readme", output_dir)

    package_manifest = {
        "mode": "stereodistance_active_profile_deployment_package",
        "status": "PASS",
        "active_profile": "active_runtime_profile.json",
        "active_profile_sha256": manifest.get("active_profile_sha256", ""),
        "profile_id": manifest.get("profile_id", ""),
        "method": manifest.get("method", ""),
        "profile_mode": manifest.get("profile_mode", ""),
        "runtime_inputs": manifest.get("runtime_inputs", []),
        "forbidden_runtime_inputs": manifest.get("forbidden_runtime_inputs", []),
        "runtime_truth_used": False,
        "coordinate_contract": manifest.get("coordinate_contract", ""),
        "final_rdf_axis_nonregression_status": manifest.get("final_rdf_axis_nonregression_status", ""),
        "public_topic_payload_status": manifest.get("public_topic_payload_status", ""),
        "public_topic_payload_sample_count": manifest.get("public_topic_payload_sample_count", 0),
        "public_topic_payload_node_equivalence_status": manifest.get(
            "public_topic_payload_node_equivalence_status",
            "",
        ),
        "source_manifest_json": str(manifest_json),
        "source_manifest_validation_json": str(manifest_validation_json),
        "file_count": len(records),
        "files": records,
    }
    write_json(output_dir / "deployment_package_manifest.json", package_manifest)
    write_text(output_dir / "deployment_package_manifest.txt", package_manifest, records)
    return package_manifest


def append_runtime_deployment_evidence(
    *,
    package_dir: Path,
    selector_json: Path,
    frontier_dir: Path,
) -> dict[str, object]:
    manifest_path = package_dir / "deployment_package_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"deployment package manifest missing: {manifest_path}")
    if not selector_json.is_file():
        raise FileNotFoundError(f"runtime selector missing: {selector_json}")
    frontier_json = frontier_dir / "runtime_deployment_frontier.json"
    frontier_txt = frontier_dir / "runtime_deployment_frontier.txt"
    frontier_labels_csv = frontier_dir / "runtime_deployment_frontier_labels.csv"
    frontier_profiles_csv = frontier_dir / "runtime_deployment_frontier_profiles.csv"
    for path in (frontier_json, frontier_txt, frontier_labels_csv, frontier_profiles_csv):
        if not path.is_file():
            raise FileNotFoundError(f"runtime deployment frontier evidence missing: {path}")

    manifest = load_json(manifest_path)
    frontier_payload = load_json(frontier_json)
    frontier_labels = frontier_payload.get("deployment_labels", [])
    active_profile_path = (package_dir / str(manifest.get("active_profile", "active_runtime_profile.json"))).resolve()
    supported_labels: list[str] = []
    if isinstance(frontier_labels, list):
        for row in frontier_labels:
            if not isinstance(row, Mapping):
                continue
            selected_profile = Path(str(row.get("selected_profile", ""))).resolve()
            deployment_label = str(row.get("deployment_label", "")).strip()
            if selected_profile == active_profile_path and deployment_label and deployment_label != "__default__":
                supported_labels.append(deployment_label)
    package_selector = {
        "mode": "stereodistance_runtime_profile_label_selector",
        "baseline_selector": "packaged_active",
        "label_profile_map": {label: "packaged_active" for label in sorted(set(supported_labels))},
        "profile_paths": {"packaged_active": "../active_runtime_profile.json"},
        "package_paths": {"packaged_active": ".."},
        "validated_evidence": {
            "package_frontier": "runtime_deployment_frontier/runtime_deployment_frontier.txt",
            "package_manifest": "../deployment_package_manifest.json",
        },
        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
        "forbidden_runtime_fields": [
            "truth_right_m",
            "truth_down_m",
            "truth_forward_m",
            "truth_range_m",
            "truth_north_m",
            "truth_east_m",
            "truth_down_ned_m",
            "validation_range_rmse_corrected_m",
            "target_max_abs_error_corrected_m",
        ],
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "selection_contract": (
            "Package-local selector for this deployment package only. It maps supported deployment/scenario "
            "labels to ../active_runtime_profile.json and does not inspect PX4/RTK/NED truth or validation errors."
        ),
        "supported_deployment_labels": sorted(set(supported_labels)),
        "runtime_truth_used": False,
    }
    package_selector_path = package_dir / "evidence" / "package_runtime_profile_label_selector.json"
    write_json(package_selector_path, package_selector)
    records = [dict(row) for row in manifest.get("files", []) if isinstance(row, Mapping)]
    records = [
        row
        for row in records
        if str(row.get("role", ""))
        not in {
            "runtime_profile_label_selector",
            "package_runtime_profile_label_selector",
            "runtime_deployment_frontier_json",
            "runtime_deployment_frontier_txt",
            "runtime_deployment_frontier_labels_csv",
            "runtime_deployment_frontier_profiles_csv",
        }
    ]
    copy_with_record(
        selector_json,
        package_dir / "evidence" / "runtime_profile_label_selector.json",
        records,
        "runtime_profile_label_selector",
        package_dir,
    )
    record_file(
        package_selector_path,
        records,
        "package_runtime_profile_label_selector",
        package_dir,
    )
    copy_with_record(
        frontier_json,
        package_dir / "evidence" / "runtime_deployment_frontier" / "runtime_deployment_frontier.json",
        records,
        "runtime_deployment_frontier_json",
        package_dir,
    )
    copy_with_record(
        frontier_txt,
        package_dir / "evidence" / "runtime_deployment_frontier" / "runtime_deployment_frontier.txt",
        records,
        "runtime_deployment_frontier_txt",
        package_dir,
    )
    copy_with_record(
        frontier_labels_csv,
        package_dir / "evidence" / "runtime_deployment_frontier" / "runtime_deployment_frontier_labels.csv",
        records,
        "runtime_deployment_frontier_labels_csv",
        package_dir,
    )
    copy_with_record(
        frontier_profiles_csv,
        package_dir / "evidence" / "runtime_deployment_frontier" / "runtime_deployment_frontier_profiles.csv",
        records,
        "runtime_deployment_frontier_profiles_csv",
        package_dir,
    )
    manifest["runtime_deployment_frontier_status"] = "PASS"
    manifest["runtime_truth_used"] = False
    manifest["file_count"] = len(records)
    manifest["files"] = records
    write_json(manifest_path, manifest)
    write_text(package_dir / "deployment_package_manifest.txt", manifest, records)

    readme_path = package_dir / "DEPLOYMENT_README.txt"
    if readme_path.is_file():
        source_manifest = load_json(package_dir / "active_profile_deployment_manifest.json")
        source_manifest["runtime_deployment_frontier_status"] = "PASS"
        write_deployment_readme(readme_path, source_manifest)
        for row in records:
            if row.get("role") == "deployment_readme":
                row["size_bytes"] = readme_path.stat().st_size
                row["sha256"] = sha256_file(readme_path)
        manifest["files"] = records
        write_json(manifest_path, manifest)
        write_text(package_dir / "deployment_package_manifest.txt", manifest, records)
    return manifest


def append_package_portability_evidence(
    *,
    package_dir: Path,
    portability_json: Path,
    portability_txt: Path | None = None,
) -> dict[str, object]:
    manifest_path = package_dir / "deployment_package_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"deployment package manifest missing: {manifest_path}")
    if not portability_json.is_file():
        raise FileNotFoundError(f"package portability evidence missing: {portability_json}")
    portability = load_json(portability_json)
    if portability.get("status") != "PASS":
        raise ValueError("package portability evidence did not pass")

    manifest = load_json(manifest_path)
    records = [dict(row) for row in manifest.get("files", []) if isinstance(row, Mapping)]
    records = [
        row
        for row in records
        if str(row.get("role", ""))
        not in {
            "package_portability_check",
            "package_portability_check_txt",
        }
    ]
    record_file(portability_json, records, "package_portability_check", package_dir)
    if portability_txt is not None and portability_txt.is_file():
        record_file(portability_txt, records, "package_portability_check_txt", package_dir)

    manifest["package_portability_status"] = "PASS"
    manifest["package_portability_selected_label"] = portability.get("selected_label", "")
    manifest["package_portability_copied_package_check_status"] = portability.get("copied_package_check_status", "")
    manifest["runtime_truth_used"] = False
    manifest["file_count"] = len(records)
    manifest["files"] = records

    readme_path = package_dir / "DEPLOYMENT_README.txt"
    if readme_path.is_file():
        readme_text = readme_path.read_text(encoding="utf-8")
        if "evidence/package_portability_check.json" not in readme_text:
            readme_text = (
                readme_text.rstrip()
                + "\n"
                + "\n"
                + "Package portability evidence:\n"
                + "- evidence/package_portability_check.json\n"
                + "- verifies package-local selector relocation without PX4/RTK/NED truth runtime input\n"
            )
            readme_path.write_text(readme_text, encoding="utf-8")
        for row in records:
            if row.get("role") == "deployment_readme":
                row["size_bytes"] = readme_path.stat().st_size
                row["sha256"] = sha256_file(readme_path)
        manifest["files"] = records

    write_json(manifest_path, manifest)
    write_text(package_dir / "deployment_package_manifest.txt", manifest, records)
    return manifest


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_package(
        manifest_json=args.manifest_json,
        manifest_validation_json=args.manifest_validation_json,
        output_dir=args.output_dir,
    )


def main() -> None:
    manifest = run()
    print(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
