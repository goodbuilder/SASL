#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Mapping, Sequence


FORBIDDEN_RUNTIME_TOKENS = ("truth", "gt_", "px4", "rtk", "ned", "validation", "replay")
RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Resolve a runtime residual disparity profile from an explicit deployment/scenario label. "
            "This selector is truth-free: labels are operator/deployment metadata, not live validation errors."
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


def selected_label_from_manifest(
    manifest: Mapping[str, object],
    *,
    deployment_label: str,
    scenario_label: str,
    scenario_name: str,
) -> tuple[str, bool]:
    label_map = manifest.get("label_profile_map", {})
    if not isinstance(label_map, Mapping):
        label_map = {}
    for value in (deployment_label, scenario_label, scenario_name):
        key = str(value)
        if key and key in label_map:
            return str(label_map[key]), True
    return str(manifest.get("default_profile_label", "")), False


def select_residual_profile_by_label(
    *,
    selector_json: Path,
    deployment_label: str = "",
    scenario_label: str = "",
    scenario_name: str = "",
) -> dict[str, object]:
    selector_json = selector_json.resolve()
    manifest = load_json(selector_json)
    failures: list[str] = []

    require(
        manifest.get("mode") == "stereodistance_disparity_residual_profile_label_selector",
        failures,
        "selector:mode",
    )
    require("RDF(right,down,forward)" in str(manifest.get("axis_contract", "")), failures, "selector:rdf_contract")
    require("FRD" in str(manifest.get("public_topic_contract", "")), failures, "selector:public_contract")
    require(not has_forbidden_runtime_field(manifest.get("selection_input_fields", [])), failures, "selector:selection_inputs_truth")

    profile_paths = manifest.get("profile_paths", {})
    evidence_paths = manifest.get("validated_evidence", {})
    require(isinstance(profile_paths, Mapping), failures, "selector:profile_paths")
    require(isinstance(evidence_paths, Mapping), failures, "selector:validated_evidence")
    selected_label, label_selection_applied = selected_label_from_manifest(
        manifest,
        deployment_label=deployment_label,
        scenario_label=scenario_label,
        scenario_name=scenario_name,
    )
    require(bool(selected_label), failures, "selection:label_missing")
    selected_profile_value = profile_paths.get(selected_label, "") if isinstance(profile_paths, Mapping) else ""
    require(bool(selected_profile_value), failures, "selection:profile_path_missing")
    selected_profile = resolve_manifest_path(selector_json, selected_profile_value) if selected_profile_value else Path("")
    require(selected_profile.is_file(), failures, "selection:profile_file_missing")

    profile_payload: dict[str, object] = {}
    if selected_profile.is_file():
        profile_payload = load_json(selected_profile)
        require(
            str(profile_payload.get("profile_type", "")) in {
                "DisparityResidualRuntimeProfile",
                "disparity_residual_runtime_profile",
            },
            failures,
            "profile:type",
        )
        require(profile_payload.get("runtime_truth_used") is False, failures, "profile:runtime_truth_used")
        require(profile_payload.get("coordinate_contracts_explicit") is True, failures, "profile:coordinate_contract")
        require(not has_forbidden_runtime_field(profile_payload.get("runtime_inputs", [])), failures, "profile:runtime_inputs_truth")
        require("RDF(right,down,forward)" in str(profile_payload.get("rdf_axis_contract", "")), failures, "profile:rdf_contract")

    evidence_status: dict[str, bool] = {}
    if isinstance(evidence_paths, Mapping):
        for key, value in evidence_paths.items():
            evidence_status[str(key)] = resolve_manifest_path(selector_json, value).exists()
        require(all(evidence_status.values()), failures, "selector:evidence_missing")

    return {
        "mode": "stereodistance_disparity_residual_profile_label_selection",
        "status": "PASS" if not failures else "FAIL",
        "selector_json": str(selector_json),
        "deployment_label": deployment_label,
        "scenario_label": scenario_label,
        "scenario_name": scenario_name,
        "selected_label": selected_label,
        "label_selection_applied": label_selection_applied,
        "selected_profile": str(selected_profile) if selected_profile_value else "",
        "profile_id": profile_payload.get("profile_id", ""),
        "method": profile_payload.get("method", ""),
        "feature_space": profile_payload.get("feature_space", ""),
        "correction_space": profile_payload.get("correction_space", ""),
        "evidence_status": evidence_status,
        "runtime_truth_used": False,
        "selection_input_fields": manifest.get("selection_input_fields", []),
        "forbidden_runtime_fields": manifest.get("forbidden_runtime_fields", []),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": manifest.get("selection_contract", ""),
        "coordinate_contract": (
            "selection uses explicit deployment/scenario labels only; selected residual profile consumes "
            "measured disparity and corrected stereo-center RDF(right,down,forward); public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance disparity residual profile label selection",
        f"status: {payload.get('status', '')}",
        f"deployment_label: {payload.get('deployment_label', '')}",
        f"scenario_label: {payload.get('scenario_label', '')}",
        f"scenario_name: {payload.get('scenario_name', '')}",
        f"selected_label: {payload.get('selected_label', '')}",
        f"label_selection_applied: {payload.get('label_selection_applied', False)}",
        f"selected_profile: {payload.get('selected_profile', '')}",
        f"profile_id: {payload.get('profile_id', '')}",
        f"method: {payload.get('method', '')}",
        f"feature_space: {payload.get('feature_space', '')}",
        f"correction_space: {payload.get('correction_space', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
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
    payload = select_residual_profile_by_label(
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
