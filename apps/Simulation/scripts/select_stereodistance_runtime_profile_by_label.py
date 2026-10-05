#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    PackagedRuntimeDeploymentSelector,
)


FORBIDDEN_RUNTIME_TOKENS = ("truth", "gt_", "px4", "rtk", "ned", "validation", "replay")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Resolve a StereoDistance runtime profile/package from an explicit deployment/scenario label. "
            "The selector is truth-free: labels are operator/deployment metadata, not live validation errors."
        )
    )
    parser.add_argument("--selector-json", type=Path, required=True)
    parser.add_argument("--deployment-label", default="")
    parser.add_argument("--scenario-label", default="")
    parser.add_argument("--scenario-name", default="")
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


def resolve_manifest_path(selector_json: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else (selector_json.parent / path).resolve()


def has_forbidden_runtime_field(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in FORBIDDEN_RUNTIME_TOKENS)


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def select_runtime_profile_by_label(
    *,
    selector_json: Path,
    deployment_label: str = "",
    scenario_label: str = "",
    scenario_name: str = "",
) -> dict[str, object]:
    selector_json = selector_json.resolve()
    manifest = load_json(selector_json)
    failures: list[str] = []

    require(manifest.get("mode") == "stereodistance_runtime_profile_label_selector", failures, "selector:mode")
    require("RDF(right,down,forward)" in str(manifest.get("axis_contract", "")), failures, "selector:rdf_contract")
    require("FRD_range" in str(manifest.get("public_topic_contract", "")).replace("+", "_"), failures, "selector:public_contract")
    require(not has_forbidden_runtime_field(manifest.get("selection_input_fields", [])), failures, "selector:selection_inputs_truth")

    features = {
        "deployment_label": deployment_label,
        "scenario_label": scenario_label,
        "scenario_name": scenario_name,
    }
    selector = PackagedRuntimeDeploymentSelector(manifest)
    selection = selector.select(features)
    selected_label = str(selection.get("selected_label", ""))

    profile_paths = manifest.get("profile_paths", {})
    package_paths = manifest.get("package_paths", {})
    evidence_paths = manifest.get("validated_evidence", {})
    require(isinstance(profile_paths, Mapping), failures, "selector:profile_paths")
    require(isinstance(package_paths, Mapping), failures, "selector:package_paths")
    require(isinstance(evidence_paths, Mapping), failures, "selector:validated_evidence")

    selected_profile = ""
    selected_package = ""
    if isinstance(profile_paths, Mapping):
        selected_profile = str(profile_paths.get(selected_label, ""))
    if isinstance(package_paths, Mapping):
        selected_package = str(package_paths.get(selected_label, ""))
    require(bool(selected_profile), failures, "selection:profile_path_missing")
    require(bool(selected_package), failures, "selection:package_path_missing")

    resolved_profile = resolve_manifest_path(selector_json, selected_profile) if selected_profile else Path("")
    resolved_package = resolve_manifest_path(selector_json, selected_package) if selected_package else Path("")
    require(resolved_profile.is_file(), failures, "selection:profile_file_missing")
    require(resolved_package.is_dir(), failures, "selection:package_dir_missing")

    profile_payload: dict[str, object] = {}
    if resolved_profile.is_file():
        profile_payload = load_json(resolved_profile)
        require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, "profile:rdf_contract")
        require(not has_forbidden_runtime_field(profile_payload.get("runtime_inputs", [])), failures, "profile:runtime_inputs_truth")

    evidence_status: dict[str, bool] = {}
    if isinstance(evidence_paths, Mapping):
        for key, value in evidence_paths.items():
            evidence_status[str(key)] = resolve_manifest_path(selector_json, value).exists()
        require(all(evidence_status.values()), failures, "selector:evidence_missing")

    return {
        "mode": "stereodistance_runtime_profile_label_selection",
        "status": "PASS" if not failures else "FAIL",
        "selector_json": str(selector_json),
        "deployment_label": deployment_label,
        "scenario_label": scenario_label,
        "scenario_name": scenario_name,
        "selected_label": selected_label,
        "label_selection_applied": bool(selection.get("label_selection_applied", False)),
        "override_applied": bool(selection.get("override_applied", False)),
        "selected_profile": str(resolved_profile) if selected_profile else "",
        "selected_package_dir": str(resolved_package) if selected_package else "",
        "profile_id": profile_payload.get("profile_id", ""),
        "method": profile_payload.get("method", ""),
        "profile_mode": profile_payload.get("mode", ""),
        "evidence_status": evidence_status,
        "runtime_truth_used": False,
        "selection_input_fields": manifest.get("selection_input_fields", []),
        "forbidden_runtime_fields": manifest.get("forbidden_runtime_fields", []),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": manifest.get("selection_contract", ""),
        "coordinate_contract": (
            "selection uses explicit deployment/scenario labels only; selected runtime profile consumes "
            "stereo-center RDF(right,down,forward) plus measured disparity; public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance runtime profile label selection",
        f"status: {payload.get('status', '')}",
        f"deployment_label: {payload.get('deployment_label', '')}",
        f"scenario_label: {payload.get('scenario_label', '')}",
        f"scenario_name: {payload.get('scenario_name', '')}",
        f"selected_label: {payload.get('selected_label', '')}",
        f"label_selection_applied: {payload.get('label_selection_applied', False)}",
        f"selected_profile: {payload.get('selected_profile', '')}",
        f"selected_package_dir: {payload.get('selected_package_dir', '')}",
        f"profile_id: {payload.get('profile_id', '')}",
        f"method: {payload.get('method', '')}",
        f"profile_mode: {payload.get('profile_mode', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        f"selection_contract: {payload.get('selection_contract', '')}",
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
    payload = select_runtime_profile_by_label(
        selector_json=args.selector_json,
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
