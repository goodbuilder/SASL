#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_stereodistance_active_profile_deployment_package as package_checker  # noqa: E402
import check_deployable_profile_candidate_package as candidate_package_checker  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install a validated StereoDistance active runtime profile from a deployment package."
    )
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    parser.add_argument("--force", action="store_true")
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


def write_text(path: Path, payload: dict[str, object]) -> None:
    lines = [
        "StereoDistance active profile package install",
        f"status: {payload.get('status', '')}",
        f"package_type: {payload.get('package_type', '')}",
        f"package_dir: {payload.get('package_dir', '')}",
        f"source_active_profile: {payload.get('source_active_profile', '')}",
        f"installed_profile: {payload.get('installed_profile', '')}",
        f"installed_profile_sha256: {payload.get('installed_profile_sha256', '')}",
        f"expected_profile_sha256: {payload.get('expected_profile_sha256', '')}",
        f"profile_id: {payload.get('profile_id', '')}",
        f"method: {payload.get('method', '')}",
        f"profile_mode: {payload.get('profile_mode', '')}",
        f"package_check_status: {payload.get('package_check_status', '')}",
        f"portability_check_status: {payload.get('portability_check_status', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def detect_package(package_dir: Path) -> tuple[str, Path, dict[str, object], dict[str, object]]:
    deployment_manifest = package_dir / "deployment_package_manifest.json"
    candidate_manifest = package_dir / "deployable_profile_candidate_package_manifest.json"
    if deployment_manifest.is_file():
        manifest = load_json(deployment_manifest)
        package_check = package_checker.check_package(package_dir)
        return "deployment_package", deployment_manifest, manifest, package_check
    if candidate_manifest.is_file():
        manifest = load_json(candidate_manifest)
        package_check = candidate_package_checker.check_package(package_dir)
        return "candidate_package", candidate_manifest, manifest, package_check
    return "unknown", package_dir / "deployment_package_manifest.json", {}, {
        "status": "FAIL",
        "failure_count": 1,
        "failures": ["package_manifest:missing"],
    }


def load_optional_portability(package_dir: Path, package_type: str) -> dict[str, object]:
    if package_type == "candidate_package":
        path = package_dir / "deployable_profile_candidate_package_portability.json"
    elif package_type == "deployment_package":
        path = package_dir / "evidence" / "package_portability_check.json"
    else:
        return {}
    return load_json(path) if path.is_file() else {}


def install_package(*, package_dir: Path, output_profile: Path, force: bool = False) -> dict[str, object]:
    failures: list[str] = []
    package_type, manifest_path, manifest, package_check = detect_package(package_dir)
    if package_check.get("status") != "PASS":
        failures.append("package_check:FAIL")

    portability_check = load_optional_portability(package_dir, package_type)
    if portability_check and portability_check.get("status") != "PASS":
        failures.append("portability_check:FAIL")

    source_active_profile = package_dir / str(manifest.get("active_profile", "active_runtime_profile.json"))
    expected_sha = str(manifest.get("active_profile_sha256", ""))
    if not source_active_profile.is_file():
        failures.append("active_profile:missing")

    if output_profile.exists() and not force:
        failures.append("output_profile:exists_without_force")

    installed_sha = ""
    installed_profile_payload: dict[str, object] = {}
    if not failures:
        output_profile.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_active_profile, output_profile)
        installed_sha = sha256_file(output_profile)
        if installed_sha != expected_sha:
            failures.append("installed_profile:sha256")
        installed_profile_payload = load_json(output_profile)

    status = "PASS" if not failures else "FAIL"
    coordinate_contract = (
        "installed profile consumes stereo-center RDF(right,down,forward) plus measured disparity; "
        "PX4/RTK/NED truth remains calibration/evaluation only; public StereoDistance alias is FRD+range"
    )
    return {
        "mode": "stereodistance_active_profile_package_install",
        "status": status,
        "package_type": package_type,
        "package_dir": str(package_dir),
        "package_manifest": str(manifest_path),
        "source_active_profile": str(source_active_profile),
        "installed_profile": str(output_profile),
        "installed_profile_sha256": installed_sha,
        "expected_profile_sha256": expected_sha,
        "profile_id": installed_profile_payload.get("profile_id", manifest.get("profile_id", "")),
        "method": installed_profile_payload.get("method", manifest.get("method", "")),
        "profile_mode": installed_profile_payload.get("mode", manifest.get("profile_mode", "")),
        "package_check_status": package_check.get("status", "FAIL"),
        "package_check_failure_count": package_check.get("failure_count", 0),
        "portability_check_status": portability_check.get("status", "not_present"),
        "runtime_truth_used": False,
        "coordinate_contract": coordinate_contract,
        "failure_count": len(failures),
        "failures": failures,
    }


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = install_package(package_dir=args.package_dir, output_profile=args.output_profile, force=args.force)
    if args.output_json:
        write_json(args.output_json, payload)
    if args.output_txt:
        write_text(args.output_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
