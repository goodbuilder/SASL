#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
FORBIDDEN_RUNTIME_FIELDS = [
    "px4_local_position",
    "rtk_truth",
    "calibration_truth",
    "validation_error",
    "target_window_label",
    "true_disparity",
    "true_image_feature",
]


def parse_mapping(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("profile mapping must be LABEL=PATH")
    label, path = value.split("=", 1)
    label = label.strip()
    if not label:
        raise argparse.ArgumentTypeError("profile label must not be empty")
    return label, Path(path)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a truth-free residual disparity profile label catalog.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--default-label", required=True)
    parser.add_argument("--profile", type=parse_mapping, action="append", required=True, help="LABEL=profile.json")
    parser.add_argument("--deployment-map", type=str, action="append", default=[], help="DEPLOYMENT_LABEL=PROFILE_LABEL")
    parser.add_argument("--evidence", type=parse_mapping, action="append", default=[], help="NAME=evidence_path")
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def relpath(path: Path, base: Path) -> str:
    try:
        return str(path.resolve().relative_to(base.resolve()))
    except ValueError:
        return str(path.resolve())


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def export_catalog(
    *,
    output_dir: Path,
    default_label: str,
    profiles: Sequence[tuple[str, Path]],
    deployment_maps: Sequence[str],
    evidence: Sequence[tuple[str, Path]],
) -> dict[str, object]:
    failures: list[str] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    profile_dir = output_dir / "profiles"
    evidence_dir = output_dir / "evidence"
    profile_dir.mkdir(parents=True, exist_ok=True)
    evidence_dir.mkdir(parents=True, exist_ok=True)
    profile_paths: dict[str, str] = {}
    profile_rows: list[dict[str, object]] = []
    label_profile_map: dict[str, str] = {}
    for item in deployment_maps:
        if "=" not in item:
            failures.append(f"deployment_map:invalid:{item}")
            continue
        deployment_label, profile_label = item.split("=", 1)
        label_profile_map[deployment_label.strip()] = profile_label.strip()
    for label, source_path in profiles:
        if not source_path.is_file():
            failures.append(f"profile:{label}:missing")
            continue
        payload = load_json(source_path)
        if payload.get("profile_type") != "DisparityResidualRuntimeProfile":
            failures.append(f"profile:{label}:type")
        if payload.get("runtime_truth_used") is not False:
            failures.append(f"profile:{label}:runtime_truth_used")
        runtime_inputs = payload.get("runtime_inputs", [])
        joined_inputs = " ".join(str(value).lower() for value in runtime_inputs) if isinstance(runtime_inputs, list) else ""
        if any(token in joined_inputs for token in ("truth", "gt_", "px4", "rtk", "ned", "validation", "replay")):
            failures.append(f"profile:{label}:runtime_inputs_truth")
        target_path = profile_dir / f"{label}.json"
        shutil.copyfile(source_path, target_path)
        profile_paths[label] = relpath(target_path, output_dir)
        profile_rows.append(
            {
                "profile_label": label,
                "profile_json": profile_paths[label],
                "profile_id": payload.get("profile_id", ""),
                "method": payload.get("method", ""),
                "feature_space": payload.get("feature_space", ""),
                "correction_space": payload.get("correction_space", ""),
                "runtime_truth_used": False,
                "rdf_axis_contract": payload.get("rdf_axis_contract", ""),
                "public_topic_contract": payload.get("public_topic_contract", ""),
            }
        )
    if default_label not in profile_paths:
        failures.append("default_label:missing_profile")
    for deployment_label, profile_label in label_profile_map.items():
        if profile_label not in profile_paths:
            failures.append(f"deployment_map:{deployment_label}:unknown_profile")
    evidence_paths: dict[str, str] = {}
    for name, source_path in evidence:
        if not source_path.exists():
            failures.append(f"evidence:{name}:missing")
            continue
        target_path = evidence_dir / source_path.name
        if source_path.is_file():
            shutil.copyfile(source_path, target_path)
        evidence_paths[name] = relpath(target_path, output_dir)
    manifest = {
        "mode": "stereodistance_disparity_residual_profile_label_selector",
        "status": "PASS" if not failures else "FAIL",
        "default_profile_label": default_label,
        "profile_paths": profile_paths,
        "label_profile_map": label_profile_map,
        "validated_evidence": evidence_paths,
        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
        "forbidden_runtime_fields": FORBIDDEN_RUNTIME_FIELDS,
        "selection_contract": (
            "Residual profile selection may use explicit operator/deployment labels only. It must not use "
            "PX4/RTK truth, validation errors, target-window labels, true disparity, or true image feature."
        ),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "failure_count": len(failures),
        "failures": failures,
    }
    manifest_path = output_dir / "disparity_residual_profile_label_selector.json"
    write_json(manifest_path, manifest)
    write_csv(output_dir / "disparity_residual_profile_catalog.csv", profile_rows)
    (output_dir / "disparity_residual_profile_catalog.txt").write_text(
        "\n".join(
            [
                "Disparity residual profile catalog",
                f"status: {manifest['status']}",
                f"default_profile_label: {default_label}",
                f"profile_count: {len(profile_rows)}",
                f"deployment_label_count: {len(label_profile_map)}",
                "runtime_truth_used: False",
                "Coordinate contract: selection uses labels only; selected profile consumes measured disparity and stereo-center RDF right/down/forward; public output remains FRD+range.",
                f"failure_count: {len(failures)}",
                *[f"- {failure}" for failure in failures],
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return {
        "status": manifest["status"],
        "selector_json": str(manifest_path),
        "profile_count": len(profile_rows),
        "deployment_label_count": len(label_profile_map),
        "runtime_truth_used": False,
        "failure_count": len(failures),
        "failures": failures,
    }


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_catalog(
        output_dir=args.output_dir,
        default_label=args.default_label,
        profiles=args.profile,
        deployment_maps=args.deployment_map,
        evidence=args.evidence,
    )


def main() -> None:
    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if result["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
