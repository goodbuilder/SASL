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

import check_stereodistance_active_profile_deployment_package as package_check  # noqa: E402
import select_stereodistance_runtime_profile_by_label as label_select  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Copy a StereoDistance deployment package to a temporary location and verify that its "
            "package-local selector still resolves the internal active runtime profile."
        )
    )
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--deployment-label", default="")
    parser.add_argument("--scenario-label", default="")
    parser.add_argument("--scenario-name", default="")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def drop_portability_records_for_regeneration(package_dir: Path) -> None:
    manifest_path = package_dir / "deployment_package_manifest.json"
    if not manifest_path.is_file():
        return
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        return
    rows = [dict(row) for row in manifest.get("files", []) if isinstance(row, Mapping)]
    rows = [
        row
        for row in rows
        if str(row.get("role", ""))
        not in {
            "package_portability_check",
            "package_portability_check_txt",
        }
    ]
    manifest["files"] = rows
    manifest["file_count"] = len(rows)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def check_portability(
    *,
    package_dir: Path,
    deployment_label: str = "",
    scenario_label: str = "",
    scenario_name: str = "",
) -> dict[str, object]:
    package_dir = package_dir.resolve()
    failures: list[str] = []
    require(package_dir.is_dir(), failures, "package_dir:missing")
    if not package_dir.is_dir():
        return {
            "mode": "stereodistance_deployment_package_portability_check",
            "status": "FAIL",
            "package_dir": str(package_dir),
            "runtime_truth_used": False,
            "failure_count": len(failures),
            "failures": failures,
        }

    with tempfile.TemporaryDirectory(prefix="stereodistance_package_portability_") as tmpdir:
        copied_package = Path(tmpdir) / "deployment_package"
        shutil.copytree(package_dir, copied_package)
        drop_portability_records_for_regeneration(copied_package)

        copied_selector = copied_package / "evidence" / "package_runtime_profile_label_selector.json"
        require(copied_selector.is_file(), failures, "copied_package_selector:missing")

        selection: dict[str, object] = {}
        if copied_selector.is_file():
            selection = label_select.select_runtime_profile_by_label(
                selector_json=copied_selector,
                deployment_label=deployment_label,
                scenario_label=scenario_label,
                scenario_name=scenario_name,
            )
            require(selection.get("status") == "PASS", failures, "copied_selector:status")
            require(selection.get("selected_label") == "packaged_active", failures, "copied_selector:selected_label")
            require(not bool(selection.get("runtime_truth_used", True)), failures, "copied_selector:runtime_truth_used")
            selected_profile = Path(str(selection.get("selected_profile", "")))
            selected_package = Path(str(selection.get("selected_package_dir", "")))
            require(selected_profile == (copied_package / "active_runtime_profile.json").resolve(), failures, "copied_selector:profile_path")
            require(selected_package == copied_package.resolve(), failures, "copied_selector:package_path")

        copied_package_check = package_check.check_package(copied_package, validate_portability_evidence=False)
        require(copied_package_check.get("status") == "PASS", failures, "copied_package_check:status")
        require(not bool(copied_package_check.get("runtime_truth_used", True)), failures, "copied_package_check:runtime_truth_used")

        result = {
            "mode": "stereodistance_deployment_package_portability_check",
            "status": "PASS" if not failures else "FAIL",
            "package_dir": str(package_dir),
            "copied_package_dir": str(copied_package),
            "copied_package_dir_ephemeral": True,
            "copied_package_dir_note": (
                "The copied package path is a temporary directory used only during this smoke check; "
                "the durable evidence is the PASS status, selected packaged_active label, and checksum-bound package files."
            ),
            "deployment_label": deployment_label,
            "scenario_label": scenario_label,
            "scenario_name": scenario_name,
            "selector_json": str(copied_selector),
            "selector_status": selection.get("status", "") if isinstance(selection, Mapping) else "",
            "selected_label": selection.get("selected_label", "") if isinstance(selection, Mapping) else "",
            "selected_profile": selection.get("selected_profile", "") if isinstance(selection, Mapping) else "",
            "selected_package_dir": selection.get("selected_package_dir", "") if isinstance(selection, Mapping) else "",
            "copied_package_check_status": copied_package_check.get("status", ""),
            "copied_package_file_count": copied_package_check.get("file_count", 0),
            "runtime_truth_used": False,
            "coordinate_contract": (
                "portable package selector consumes deployment/scenario labels only; selected runtime profile "
                "consumes stereo-center RDF(right,down,forward) plus measured disparity; public output remains FRD+range"
            ),
            "failure_count": len(failures),
            "failures": failures,
        }

    return result


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance deployment package portability check",
        f"status: {payload.get('status', '')}",
        f"package_dir: {payload.get('package_dir', '')}",
        f"deployment_label: {payload.get('deployment_label', '')}",
        f"scenario_label: {payload.get('scenario_label', '')}",
        f"scenario_name: {payload.get('scenario_name', '')}",
        f"selected_label: {payload.get('selected_label', '')}",
        f"selected_profile: {payload.get('selected_profile', '')}",
        f"selected_package_dir: {payload.get('selected_package_dir', '')}",
        f"copied_package_dir_ephemeral: {payload.get('copied_package_dir_ephemeral', False)}",
        f"copied_package_dir_note: {payload.get('copied_package_dir_note', '')}",
        f"copied_package_check_status: {payload.get('copied_package_check_status', '')}",
        f"copied_package_file_count: {payload.get('copied_package_file_count', 0)}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
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
    payload = check_portability(
        package_dir=args.package_dir,
        deployment_label=args.deployment_label,
        scenario_label=args.scenario_label,
        scenario_name=args.scenario_name,
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
