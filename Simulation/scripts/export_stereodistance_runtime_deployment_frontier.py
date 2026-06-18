#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import select_stereodistance_runtime_profile_by_label as label_selector  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a truth-free StereoDistance runtime deployment frontier from a label selector manifest. "
            "The output is an operator/deployment view of which validated profile/package each label resolves to."
        )
    )
    parser.add_argument("--selector-json", type=Path, required=True)
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


def resolve_manifest_path(selector_json: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else (selector_json.parent / path).resolve()


def has_runtime_truth(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned", "validation", "replay"))


def read_optional_json(path: Path) -> dict[str, object]:
    return load_json(path) if path.is_file() else {}


def collect_profile_rows(selector_json: Path, manifest: Mapping[str, object]) -> list[dict[str, object]]:
    profile_paths = manifest.get("profile_paths", {})
    package_paths = manifest.get("package_paths", {})
    if not isinstance(profile_paths, Mapping) or not isinstance(package_paths, Mapping):
        return []

    profile_labels = sorted({str(label) for label in profile_paths.keys()} | {str(label) for label in package_paths.keys()})
    rows: list[dict[str, object]] = []
    for profile_label in profile_labels:
        profile_path = resolve_manifest_path(selector_json, profile_paths.get(profile_label, ""))
        package_dir = resolve_manifest_path(selector_json, package_paths.get(profile_label, ""))
        profile_payload = read_optional_json(profile_path)
        readiness_json = package_dir.parent / "deployment_readiness.json"
        readiness_payload = read_optional_json(readiness_json)
        operator_selector_json = package_dir.parent / "operator_view_launch_check_selector.json"
        operator_selector_payload = read_optional_json(operator_selector_json)
        runtime_inputs = profile_payload.get("runtime_inputs", [])
        axis_contract = str(profile_payload.get("axis_contract", ""))
        rows.append(
            {
                "profile_label": profile_label,
                "profile_json": str(profile_path),
                "package_dir": str(package_dir),
                "profile_exists": profile_path.is_file(),
                "package_exists": package_dir.is_dir(),
                "profile_sha256": sha256_file(profile_path) if profile_path.is_file() else "",
                "profile_id": profile_payload.get("profile_id", ""),
                "method": profile_payload.get("method", ""),
                "profile_mode": profile_payload.get("mode", ""),
                "axis_contract": axis_contract,
                "runtime_inputs": ";".join(str(value) for value in runtime_inputs) if isinstance(runtime_inputs, list) else "",
                "runtime_truth_used": False,
                "runtime_inputs_truth_free": not has_runtime_truth(runtime_inputs),
                "rdf_contract_declared": "RDF(right,down,forward)" in axis_contract,
                "readiness_json": str(readiness_json) if readiness_json.is_file() else "",
                "readiness_status": readiness_payload.get("status", ""),
                "readiness_failure_count": readiness_payload.get("failure_count", ""),
                "operator_selector_json": str(operator_selector_json) if operator_selector_json.is_file() else "",
                "operator_selector_status": operator_selector_payload.get("status", ""),
                "operator_selector_selected_label": operator_selector_payload.get("selected_label", ""),
            }
        )
    return rows


def collect_label_rows(selector_json: Path, manifest: Mapping[str, object]) -> list[dict[str, object]]:
    label_map = manifest.get("label_profile_map", {})
    if not isinstance(label_map, Mapping):
        label_map = {}
    deployment_labels = sorted(str(label) for label in label_map.keys())
    if "" not in deployment_labels:
        deployment_labels.insert(0, "")

    rows: list[dict[str, object]] = []
    for deployment_label in deployment_labels:
        selection = label_selector.select_runtime_profile_by_label(
            selector_json=selector_json,
            deployment_label=deployment_label,
        )
        rows.append(
            {
                "deployment_label": deployment_label or "__default__",
                "status": selection.get("status", ""),
                "selected_profile_label": selection.get("selected_label", ""),
                "label_selection_applied": selection.get("label_selection_applied", False),
                "selected_profile": selection.get("selected_profile", ""),
                "selected_package_dir": selection.get("selected_package_dir", ""),
                "profile_id": selection.get("profile_id", ""),
                "method": selection.get("method", ""),
                "profile_mode": selection.get("profile_mode", ""),
                "runtime_truth_used": selection.get("runtime_truth_used", True),
                "failure_count": selection.get("failure_count", 0),
                "failures": ";".join(str(item) for item in selection.get("failures", []))
                if isinstance(selection.get("failures", []), list)
                else "",
            }
        )
    return rows


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_text(
    path: Path,
    *,
    summary: Mapping[str, object],
    profile_rows: Sequence[Mapping[str, object]],
    label_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance runtime deployment frontier",
        f"status: {summary.get('status', '')}",
        f"selector_json: {summary.get('selector_json', '')}",
        f"default_profile_label: {summary.get('default_profile_label', '')}",
        f"profile_count: {summary.get('profile_count', 0)}",
        f"deployment_label_count: {summary.get('deployment_label_count', 0)}",
        f"truth_free_runtime_selection: {summary.get('truth_free_runtime_selection', False)}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"failure_count: {summary.get('failure_count', 0)}",
        "",
        "Profiles:",
    ]
    for row in profile_rows:
        lines.append(
            "- {profile_label}: status={readiness_status} method={method} sha256={sha}".format(
                profile_label=row.get("profile_label", ""),
                readiness_status=row.get("readiness_status", ""),
                method=row.get("method", ""),
                sha=row.get("profile_sha256", ""),
            )
        )
    lines.extend(["", "Deployment labels:"])
    for row in label_rows:
        lines.append(
            "- {deployment_label} -> {selected_profile_label} ({status})".format(
                deployment_label=row.get("deployment_label", ""),
                selected_profile_label=row.get("selected_profile_label", ""),
                status=row.get("status", ""),
            )
        )
    failures = summary.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.extend(["", "Failures:"])
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_frontier(*, selector_json: Path, output_dir: Path) -> dict[str, object]:
    selector_json = selector_json.resolve()
    manifest = load_json(selector_json)
    profile_rows = collect_profile_rows(selector_json, manifest)
    label_rows = collect_label_rows(selector_json, manifest)
    failures: list[str] = []

    if manifest.get("mode") != "stereodistance_runtime_profile_label_selector":
        failures.append("selector:mode")
    if "RDF(right,down,forward)" not in str(manifest.get("axis_contract", "")):
        failures.append("selector:rdf_contract")
    if "FRD" not in str(manifest.get("public_topic_contract", "")):
        failures.append("selector:public_contract")
    if has_runtime_truth(manifest.get("selection_input_fields", [])):
        failures.append("selector:selection_inputs_truth")

    for row in profile_rows:
        label = row.get("profile_label", "")
        if not bool(row.get("profile_exists", False)):
            failures.append(f"profile:{label}:missing")
        if not bool(row.get("package_exists", False)):
            failures.append(f"package:{label}:missing")
        if not bool(row.get("runtime_inputs_truth_free", False)):
            failures.append(f"profile:{label}:runtime_inputs_truth")
        if not bool(row.get("rdf_contract_declared", False)):
            failures.append(f"profile:{label}:rdf_contract")
        if row.get("readiness_status", "") not in ("PASS", ""):
            failures.append(f"profile:{label}:readiness")

    for row in label_rows:
        label = row.get("deployment_label", "")
        if row.get("status") != "PASS":
            failures.append(f"label:{label}:selection")
        if bool(row.get("runtime_truth_used", True)):
            failures.append(f"label:{label}:runtime_truth")

    summary = {
        "mode": "stereodistance_runtime_deployment_frontier",
        "status": "PASS" if not failures else "FAIL",
        "selector_json": str(selector_json),
        "default_profile_label": manifest.get("baseline_selector", ""),
        "profile_count": len(profile_rows),
        "deployment_label_count": len(label_rows),
        "profile_labels": [row.get("profile_label", "") for row in profile_rows],
        "deployment_labels": [row.get("deployment_label", "") for row in label_rows],
        "truth_free_runtime_selection": True,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_truth_used": False,
        "failure_count": len(failures),
        "failures": failures,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "runtime_deployment_frontier_profiles.csv", profile_rows)
    write_csv(output_dir / "runtime_deployment_frontier_labels.csv", label_rows)
    write_json(output_dir / "runtime_deployment_frontier.json", {
        "summary": summary,
        "profiles": profile_rows,
        "deployment_labels": label_rows,
    })
    write_text(
        output_dir / "runtime_deployment_frontier.txt",
        summary=summary,
        profile_rows=profile_rows,
        label_rows=label_rows,
    )
    return summary


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_frontier(selector_json=args.selector_json, output_dir=args.output_dir)


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
