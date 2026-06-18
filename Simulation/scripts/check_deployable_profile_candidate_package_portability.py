#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_deployable_profile_candidate_package as package_checker  # noqa: E402
import check_stereodistance_operator_view_launch as operator_checker  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Copy a deployable profile candidate package to a temporary location and verify that package-local "
            "active_runtime_profile.json, package check, and operator-view dry-run remain valid."
        )
    )
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def check_portability(*, package_dir: Path) -> dict[str, object]:
    package_dir = package_dir.resolve()
    failures: list[str] = []
    require(package_dir.is_dir(), failures, "package_dir:missing")
    if not package_dir.is_dir():
        return {
            "mode": "deployable_profile_candidate_package_portability_check",
            "status": "FAIL",
            "package_dir": str(package_dir),
            "runtime_truth_used": False,
            "failure_count": len(failures),
            "failures": failures,
        }

    with tempfile.TemporaryDirectory(prefix="deployable_profile_candidate_package_") as tmpdir:
        copied_package = Path(tmpdir) / "candidate_package"
        shutil.copytree(package_dir, copied_package)
        copied_manifest = copied_package / "deployable_profile_candidate_package_manifest.json"
        copied_active_profile = copied_package / "active_runtime_profile.json"
        require(copied_manifest.is_file(), failures, "copied_package:manifest")
        require(copied_active_profile.is_file(), failures, "copied_package:active_profile")

        copied_package_check = package_checker.check_package(copied_package)
        require(copied_package_check.get("status") == "PASS", failures, "copied_package_check:status")
        require(not bool(copied_package_check.get("runtime_truth_used", True)), failures, "copied_package_check:runtime_truth_used")

        operator_check = operator_checker.check_operator_view_launch(package_dir=copied_package)
        require(operator_check.get("status") == "PASS", failures, "copied_operator_view:status")
        require(operator_check.get("package_type") == "candidate_package", failures, "copied_operator_view:package_type")
        require(operator_check.get("package_check_status") == "PASS", failures, "copied_operator_view:package_check")
        require(not bool(operator_check.get("runtime_truth_used", True)), failures, "copied_operator_view:runtime_truth_used")
        require(
            Path(str(operator_check.get("resolved_runtime_profile", ""))) == copied_active_profile.resolve(),
            failures,
            "copied_operator_view:resolved_profile",
        )

        result = {
            "mode": "deployable_profile_candidate_package_portability_check",
            "status": "PASS" if not failures else "FAIL",
            "package_dir": str(package_dir),
            "copied_package_dir": str(copied_package),
            "copied_package_dir_ephemeral": True,
            "copied_package_dir_note": (
                "The copied package path is a temporary directory used only during this smoke check; "
                "the durable evidence is the PASS status, checksum-bound package files, and package-local active profile resolution."
            ),
            "copied_active_profile": str(copied_active_profile),
            "copied_package_check_status": copied_package_check.get("status", ""),
            "copied_package_file_count": copied_package_check.get("file_count", 0),
            "copied_operator_view_status": operator_check.get("status", ""),
            "copied_operator_view_package_type": operator_check.get("package_type", ""),
            "copied_operator_view_resolved_profile": operator_check.get("resolved_runtime_profile", ""),
            "copied_operator_view_sha256": operator_check.get("resolved_runtime_profile_sha256", ""),
            "runtime_truth_used": False,
            "coordinate_contract": (
                "portable candidate package resolves package-local active_runtime_profile.json; runtime profile consumes "
                "stereo-center RDF(right,down,forward) plus measured disparity; public output remains FRD+range"
            ),
            "failure_count": len(failures),
            "failures": failures,
        }

    return result


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Deployable profile candidate package portability check",
        f"status: {payload.get('status', '')}",
        f"package_dir: {payload.get('package_dir', '')}",
        f"copied_package_dir_ephemeral: {payload.get('copied_package_dir_ephemeral', '')}",
        f"copied_package_dir_note: {payload.get('copied_package_dir_note', '')}",
        f"copied_active_profile: {payload.get('copied_active_profile', '')}",
        f"copied_package_check_status: {payload.get('copied_package_check_status', '')}",
        f"copied_package_file_count: {payload.get('copied_package_file_count', '')}",
        f"copied_operator_view_status: {payload.get('copied_operator_view_status', '')}",
        f"copied_operator_view_package_type: {payload.get('copied_operator_view_package_type', '')}",
        f"copied_operator_view_resolved_profile: {payload.get('copied_operator_view_resolved_profile', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', '')}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = check_portability(package_dir=args.package_dir)
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> int:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
