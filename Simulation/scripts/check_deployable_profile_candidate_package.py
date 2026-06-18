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


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import stereodistance_profile_promotion_check as promotion_check  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check a lightweight deployable profile candidate package.")
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
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


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def role_paths(package_dir: Path, files: object) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    if not isinstance(files, list):
        return paths
    for row in files:
        if not isinstance(row, Mapping):
            continue
        role = str(row.get("role", ""))
        rel = Path(str(row.get("path", "")))
        if role and not rel.is_absolute() and ".." not in rel.parts:
            paths[role] = package_dir / rel
    return paths


def has_runtime_truth(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def int_value(value: object) -> int:
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return 0


def payload_csv_valid(path: Path, expected_count: int) -> bool:
    if not path.is_file():
        return False
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != expected_count:
        return False
    for row in rows:
        if row.get("data_axis_order") != "forward_m,right_m,down_m,range_m":
            return False
        if row.get("runtime_truth_used") != "False":
            return False
        if row.get("node_public_payload_equivalent") != "True":
            return False
        try:
            values = json.loads(row.get("data", "[]"))
        except json.JSONDecodeError:
            return False
        if not (isinstance(values, list) and len(values) == 4 and all(math.isfinite(float(v)) for v in values)):
            return False
    return True


def runtime_apply_smoke(profile_payload: Mapping[str, object]) -> tuple[bool, str]:
    try:
        profile = promotion_check.load_runtime_profile(profile_payload)
        result = profile.apply(
            {
                "right_m": 4.0,
                "down_m": 1.5,
                "forward_m": 800.0,
                "range_m": math.sqrt(4.0 * 4.0 + 1.5 * 1.5 + 800.0 * 800.0),
                "disparity_px": 25.0,
                "raw_disparity_px": 25.0,
                "left_u_px": 640.0,
                "left_v_px": 360.0,
            }
        )
    except Exception as exc:
        return False, f"exception:{exc}"
    corrected = result.corrected
    if not isinstance(corrected, Mapping):
        return False, "corrected_not_mapping"
    try:
        ok = all(math.isfinite(float(corrected[key])) for key in ("right_m", "down_m", "forward_m", "range_m", "disparity_px"))
    except (KeyError, TypeError, ValueError):
        ok = False
    return ok, "PASS" if ok else "nonfinite_or_missing"


def check_package(package_dir: Path) -> dict[str, object]:
    failures: list[str] = []
    manifest_path = package_dir / "deployable_profile_candidate_package_manifest.json"
    require(manifest_path.is_file(), failures, "manifest:missing")
    if not manifest_path.is_file():
        return {
            "status": "FAIL",
            "package_dir": str(package_dir),
            "runtime_truth_used": False,
            "failure_count": len(failures),
            "failures": failures,
        }
    manifest = load_json(manifest_path)
    require(manifest.get("mode") == "deployable_profile_candidate_package", failures, "manifest:mode")
    require(manifest.get("status") == "PASS", failures, "manifest:status")
    require(not bool(manifest.get("runtime_truth_used", True)), failures, "manifest:runtime_truth_used")
    require("RDF(right,down,forward)" in str(manifest.get("axis_contract", "")), failures, "manifest:axis_contract")
    require("FRD_range(forward,right,down,range)" in str(manifest.get("public_topic_contract", "")), failures, "manifest:public_topic_contract")
    require("truth remains calibration/evaluation evidence only" in str(manifest.get("coordinate_contract", "")), failures, "manifest:truth_scope")
    files = manifest.get("files", [])
    require(isinstance(files, list), failures, "manifest:files")
    if isinstance(files, list):
        require(int_value(manifest.get("file_count")) == len(files), failures, "manifest:file_count")
        for row in files:
            if not isinstance(row, Mapping):
                failures.append("files:row_not_mapping")
                continue
            rel = Path(str(row.get("path", "")))
            role = str(row.get("role", ""))
            if rel.is_absolute() or ".." in rel.parts:
                failures.append(f"files:{role}:unsafe_path")
                continue
            path = package_dir / rel
            require(path.is_file(), failures, f"files:{role}:missing")
            if path.is_file():
                require(sha256_file(path) == str(row.get("sha256", "")), failures, f"files:{role}:sha256")
                require(path.stat().st_size == int_value(row.get("size_bytes")), failures, f"files:{role}:size")
    paths = role_paths(package_dir, files)
    for role in (
        "active_profile",
        "candidate_readiness",
        "candidate_export",
        "policy_selection",
        "promotion_check",
        "public_topic_payload_summary",
        "public_topic_payload_csv",
        "package_readme",
    ):
        require(role in paths, failures, f"files:{role}:not_recorded")

    active_profile = paths.get("active_profile", package_dir / "active_runtime_profile.json")
    require(active_profile.is_file(), failures, "active_profile:missing")
    active_sha = sha256_file(active_profile) if active_profile.is_file() else ""
    require(active_sha == str(manifest.get("active_profile_sha256", "")), failures, "active_profile:sha256")
    runtime_apply_status = ""
    if active_profile.is_file():
        profile_payload = load_json(active_profile)
        require(profile_payload.get("method") == manifest.get("method"), failures, "active_profile:method")
        require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, "active_profile:axis_contract")
        require(not has_runtime_truth(profile_payload.get("runtime_inputs", [])), failures, "active_profile:runtime_inputs_truth")
        ok, runtime_apply_status = runtime_apply_smoke(profile_payload)
        require(ok, failures, f"active_profile:runtime_apply:{runtime_apply_status}")
        node_ok, node_log = promotion_check.node_loader_check(active_profile)
        require(node_ok, failures, f"active_profile:node_loader:{node_log}")

    readiness = load_json(paths["candidate_readiness"]) if "candidate_readiness" in paths and paths["candidate_readiness"].is_file() else {}
    require(readiness.get("status") == "PASS", failures, "readiness:status")
    require(readiness.get("method") == manifest.get("method"), failures, "readiness:method")
    require(readiness.get("active_profile_sha256") == active_sha, failures, "readiness:sha256")
    require(int_value(readiness.get("candidate_pass_count")) == int_value(readiness.get("candidate_expected_run_count")), failures, "readiness:pass_count")
    require(int_value(readiness.get("candidate_axis_pass_count")) == int_value(readiness.get("candidate_expected_run_count")), failures, "readiness:axis_pass_count")
    require(int_value(readiness.get("candidate_target_pass_count")) == int_value(readiness.get("candidate_expected_run_count")), failures, "readiness:target_pass_count")
    require(not bool(readiness.get("runtime_truth_used", True)), failures, "readiness:runtime_truth_used")

    promotion = load_json(paths["promotion_check"]) if "promotion_check" in paths and paths["promotion_check"].is_file() else {}
    require(bool(promotion.get("pass_promotion_check", False)), failures, "promotion:pass")
    require(int_value(promotion.get("sample_apply_count")) > 0, failures, "promotion:sample_apply_count")
    require(int_value(promotion.get("sample_invalid_count")) == 0, failures, "promotion:sample_invalid_count")
    require(not bool(promotion.get("runtime_truth_used", True)), failures, "promotion:runtime_truth_used")

    public_payload = (
        load_json(paths["public_topic_payload_summary"])
        if "public_topic_payload_summary" in paths and paths["public_topic_payload_summary"].is_file()
        else {}
    )
    public_count = int_value(public_payload.get("sample_count"))
    require(public_payload.get("status") == "PASS", failures, "public_payload:status")
    require(public_count > 0, failures, "public_payload:sample_count")
    require(public_payload.get("node_public_payload_equivalence_status") == "PASS", failures, "public_payload:node_equivalence")
    require(not bool(public_payload.get("runtime_truth_used", True)), failures, "public_payload:runtime_truth_used")
    if "public_topic_payload_csv" in paths:
        require(payload_csv_valid(paths["public_topic_payload_csv"], public_count), failures, "public_payload_csv:valid")

    if "package_readme" in paths and paths["package_readme"].is_file():
        readme = paths["package_readme"].read_text(encoding="utf-8")
        require("active_runtime_profile.json" in readme, failures, "readme:active_profile")
        require("right_m, down_m, forward_m" in readme, failures, "readme:rdf_inputs")
        require("data=[forward_m,right_m,down_m,range_m]" in readme, failures, "readme:frd_output")
        require("do not provide PX4/RTK/NED truth" in readme, failures, "readme:truth_scope")

    passed = not failures
    return {
        "mode": "deployable_profile_candidate_package_check",
        "status": "PASS" if passed else "FAIL",
        "package_dir": str(package_dir),
        "manifest": str(manifest_path),
        "method": manifest.get("method", ""),
        "recommended_policy": manifest.get("recommended_policy", ""),
        "active_profile": str(active_profile),
        "active_profile_sha256": active_sha,
        "runtime_apply_status": runtime_apply_status,
        "candidate_pass_count": readiness.get("candidate_pass_count", manifest.get("candidate_pass_count", "")),
        "candidate_expected_run_count": readiness.get("candidate_expected_run_count", manifest.get("candidate_expected_run_count", "")),
        "candidate_axis_pass_count": readiness.get("candidate_axis_pass_count", manifest.get("candidate_axis_pass_count", "")),
        "candidate_target_pass_count": readiness.get("candidate_target_pass_count", manifest.get("candidate_target_pass_count", "")),
        "promotion_sample_apply_count": promotion.get("sample_apply_count", manifest.get("promotion_sample_apply_count", "")),
        "promotion_sample_invalid_count": promotion.get("sample_invalid_count", manifest.get("promotion_sample_invalid_count", "")),
        "public_topic_payload_sample_count": public_payload.get("sample_count", manifest.get("public_topic_payload_sample_count", "")),
        "public_topic_payload_node_equivalence_status": public_payload.get(
            "node_public_payload_equivalence_status",
            manifest.get("public_topic_payload_node_equivalence_status", ""),
        ),
        "file_count": int_value(manifest.get("file_count")),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Deployable StereoDistance profile candidate package check",
        f"status: {payload.get('status', '')}",
        f"package_dir: {payload.get('package_dir', '')}",
        f"method: {payload.get('method', '')}",
        f"recommended_policy: {payload.get('recommended_policy', '')}",
        f"active_profile: {payload.get('active_profile', '')}",
        f"active_profile_sha256: {payload.get('active_profile_sha256', '')}",
        f"runtime_apply_status: {payload.get('runtime_apply_status', '')}",
        f"candidate_pass_count: {payload.get('candidate_pass_count', '')}/{payload.get('candidate_expected_run_count', '')}",
        f"candidate_axis_pass_count: {payload.get('candidate_axis_pass_count', '')}",
        f"candidate_target_pass_count: {payload.get('candidate_target_pass_count', '')}",
        f"promotion_sample_apply_count: {payload.get('promotion_sample_apply_count', '')}",
        f"promotion_sample_invalid_count: {payload.get('promotion_sample_invalid_count', '')}",
        f"public_topic_payload_sample_count: {payload.get('public_topic_payload_sample_count', '')}",
        f"public_topic_payload_node_equivalence_status: {payload.get('public_topic_payload_node_equivalence_status', '')}",
        f"file_count: {payload.get('file_count', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        f"failure_count: {payload.get('failure_count', '')}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list):
        for failure in failures:
            lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    result = check_package(args.package_dir)
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
