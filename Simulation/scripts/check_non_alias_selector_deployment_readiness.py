#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
FORBIDDEN_ALIAS_TEXT = "direct NED-to-RDF axis alias is forbidden"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check non-alias robustness selector deployment readiness: coordinate contract, "
            "scenario catalog coverage, selector package, frontier, and operator launch dry-runs."
        )
    )
    parser.add_argument("--robustness-dir", type=Path, required=True)
    parser.add_argument("--selector-package-dir", type=Path, required=True)
    parser.add_argument("--readme", type=Path, default=Path("Simulation/README.md"))
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def load_optional_json(path: Path) -> dict[str, object]:
    return load_json(path) if path.is_file() else {}


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def boolish(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def status_row(requirement_id: str, description: str, passed: bool, evidence: str, detail: object = "") -> dict[str, object]:
    return {
        "requirement_id": requirement_id,
        "status": "PASS" if passed else "FAIL",
        "description": description,
        "evidence": evidence,
        "detail": detail,
    }


def nested_summary(payload: Mapping[str, object], key: str) -> dict[str, object]:
    value = payload.get(key, {})
    return dict(value) if isinstance(value, Mapping) else {}


def check_readiness(
    *,
    robustness_dir: Path,
    selector_package_dir: Path,
    readme: Path,
) -> dict[str, object]:
    robustness_dir = robustness_dir.resolve()
    selector_package_dir = selector_package_dir.resolve()
    readme = readme.resolve()

    robustness_summary_path = robustness_dir / "robustness_summary.json"
    scenario_catalog_path = robustness_dir / "scenario_profile_catalog.json"
    coordinate_csv_path = robustness_dir / "scenario_coordinate_contracts.csv"
    selector_summary_path = selector_package_dir / "selector_package_export_summary.json"
    selector_json_path = selector_package_dir / "runtime_profile_label_selector.json"
    frontier_json_path = selector_package_dir / "runtime_deployment_frontier" / "runtime_deployment_frontier.json"
    operator_checks_json_path = selector_package_dir / "operator_view_launch_checks" / "operator_view_launch_checks.json"

    robustness = load_optional_json(robustness_summary_path)
    catalog = load_optional_json(scenario_catalog_path)
    selector_summary = load_optional_json(selector_summary_path)
    selector_manifest = load_optional_json(selector_json_path)
    frontier_payload = load_optional_json(frontier_json_path)
    operator_payload = load_optional_json(operator_checks_json_path)
    coordinate_rows = read_csv_rows(coordinate_csv_path)
    readme_text = readme.read_text(encoding="utf-8") if readme.is_file() else ""

    coordinate_summary = nested_summary(robustness, "coordinate_contract_summary")
    frontier_summary = nested_summary(frontier_payload, "summary")
    operator_summary = nested_summary(operator_payload, "summary")
    operator_rows = operator_payload.get("rows", []) if isinstance(operator_payload, Mapping) else []
    catalog_rows = catalog.get("rows", []) if isinstance(catalog, Mapping) else []
    scenario_count = int(robustness.get("scenario_count", 0) or catalog.get("catalog_profile_count", 0) or 0)

    all_coordinate_rows_forbid_alias = bool(coordinate_rows) and all(
        boolish(row.get("all_loaded_contracts_forbid_direct_axis_alias"))
        and FORBIDDEN_ALIAS_TEXT in str(row.get("truth_rdf_transform_contract", ""))
        and row.get("axis_contract") == RDF_AXIS_CONTRACT
        for row in coordinate_rows
    )
    all_catalog_profiles_ready = (
        bool(catalog_rows)
        and len(catalog_rows) == scenario_count
        and all(
            boolish(row.get("profile_exists"))
            and boolish(row.get("pass_cross_seed_replay"))
            and row.get("axis_contract") == RDF_AXIS_CONTRACT
            and "truth" not in str(row.get("runtime_input_contract", "")).lower().replace("no runtime truth", "")
            for row in catalog_rows
            if isinstance(row, Mapping)
        )
    )
    label_map = selector_manifest.get("label_profile_map", {})
    profile_paths = selector_manifest.get("profile_paths", {})
    package_paths = selector_manifest.get("package_paths", {})
    selector_runtime_fields = selector_manifest.get("selection_input_fields", [])
    selector_input_truth_free = isinstance(selector_runtime_fields, list) and not any(
        token in " ".join(str(item).lower() for item in selector_runtime_fields)
        for token in ("truth", "px4", "rtk", "ned", "validation", "replay")
    )
    operator_rows_truth_free = isinstance(operator_rows, list) and all(
        isinstance(row, Mapping)
        and row.get("status") == "PASS"
        and boolish(row.get("bbox_adjustment_arg_present"))
        and not boolish(row.get("runtime_truth_used"))
        for row in operator_rows
    )

    requirements = [
        status_row(
            "REQ-01-non-alias-coordinate-contract",
            "Every scenario/seed config forbids direct NED-to-RDF axis alias and declares explicit stereo-center RDF truth contract.",
            boolish(coordinate_summary.get("all_scenarios_forbid_direct_axis_alias"))
            and int(coordinate_summary.get("direct_axis_alias_forbidden_count", 0) or 0)
            == int(coordinate_summary.get("loaded_contract_count", -1) or -1)
            and all_coordinate_rows_forbid_alias,
            str(coordinate_csv_path),
            coordinate_summary,
        ),
        status_row(
            "REQ-02-scenario-catalog-coverage",
            "Scenario profile catalog covers every robustness scenario and every profile passed cross-seed replay.",
            boolish(catalog.get("pass_scenario_profile_catalog"))
            and int(catalog.get("catalog_profile_count", 0) or 0) == scenario_count
            and int(catalog.get("profile_exists_count", 0) or 0) == scenario_count
            and int(catalog.get("cross_seed_pass_count", 0) or 0) == scenario_count
            and all_catalog_profiles_ready,
            str(scenario_catalog_path),
            {
                "scenario_count": scenario_count,
                "catalog_profile_count": catalog.get("catalog_profile_count", 0),
                "profile_exists_count": catalog.get("profile_exists_count", 0),
                "cross_seed_pass_count": catalog.get("cross_seed_pass_count", 0),
            },
        ),
        status_row(
            "REQ-03-selector-package",
            "Selector package exists, maps labels to all scenario profiles, and uses only explicit deployment/scenario metadata.",
            selector_summary.get("status") == "PASS"
            and selector_json_path.is_file()
            and int(selector_summary.get("profile_count", 0) or 0) == scenario_count
            and isinstance(label_map, Mapping)
            and isinstance(profile_paths, Mapping)
            and isinstance(package_paths, Mapping)
            and len(profile_paths) == scenario_count
            and len(package_paths) == scenario_count
            and selector_input_truth_free
            and selector_manifest.get("axis_contract") == RDF_AXIS_CONTRACT,
            str(selector_summary_path),
            {
                "profile_count": selector_summary.get("profile_count", 0),
                "deployment_label_count": selector_summary.get("deployment_label_count", 0),
                "selection_input_fields": selector_runtime_fields,
            },
        ),
        status_row(
            "REQ-04-frontier",
            "Runtime deployment frontier validates every selector label without runtime truth.",
            frontier_summary.get("status") == "PASS"
            and bool(frontier_summary.get("truth_free_runtime_selection"))
            and not bool(frontier_summary.get("runtime_truth_used", True))
            and int(frontier_summary.get("profile_count", 0) or 0) == scenario_count
            and int(frontier_summary.get("deployment_label_count", 0) or 0) >= int(selector_summary.get("deployment_label_count", 0) or 0)
            and frontier_summary.get("axis_contract") == RDF_AXIS_CONTRACT,
            str(frontier_json_path),
            frontier_summary,
        ),
        status_row(
            "REQ-05-operator-launch",
            "Operator-view dry-run resolves default and deployment labels to --bbox-adjustment profiles without runtime truth.",
            operator_summary.get("status") == "PASS"
            and int(operator_summary.get("check_count", 0) or 0) > 0
            and int(operator_summary.get("pass_count", 0) or 0) == int(operator_summary.get("check_count", -1) or -1)
            and int(operator_summary.get("bbox_adjustment_arg_count", 0) or 0)
            == int(operator_summary.get("check_count", -1) or -1)
            and int(operator_summary.get("runtime_truth_used_count", 1)) == 0
            and operator_rows_truth_free,
            str(operator_checks_json_path),
            operator_summary,
        ),
        status_row(
            "REQ-06-runtime-truth-free",
            "No selector/frontier/operator deployment stage uses PX4/RTK/NED truth, validation metrics, replay metrics, or target-window labels at runtime.",
            not bool(selector_summary.get("runtime_truth_used", True))
            and not bool(frontier_summary.get("runtime_truth_used", True))
            and not bool(operator_summary.get("runtime_truth_used", True))
            and "PX4/RTK/NED truth, validation error, target-window label, replay metric" in readme_text,
            str(readme),
            {
                "selector_runtime_truth_used": selector_summary.get("runtime_truth_used", True),
                "frontier_runtime_truth_used": frontier_summary.get("runtime_truth_used", True),
                "operator_runtime_truth_used": operator_summary.get("runtime_truth_used", True),
            },
        ),
        status_row(
            "REQ-07-coordinate-documentation",
            "README documents forbidden camera-NED alignment and separates stereo-center RDF from public FRD+range.",
            "forward=north" in readme_text
            and "right=east" in readme_text
            and "down=down" in readme_text
            and "가정하면 안 된다" in readme_text
            and "stereo-center RDF(right,down,forward)" in readme_text
            and "FRD+range(forward,right,down,range)" in readme_text,
            str(readme),
            "non-alias synthetic and runtime coordinate contract documentation",
        ),
    ]
    passed = all(row["status"] == "PASS" for row in requirements)
    return {
        "mode": "non_alias_selector_deployment_readiness",
        "status": "PASS" if passed else "FAIL",
        "robustness_dir": str(robustness_dir),
        "selector_package_dir": str(selector_package_dir),
        "readme": str(readme),
        "scenario_count": scenario_count,
        "profile_count": selector_summary.get("profile_count", 0),
        "deployment_label_count": selector_summary.get("deployment_label_count", 0),
        "operator_launch_check_count": operator_summary.get("check_count", 0),
        "operator_launch_pass_count": operator_summary.get("pass_count", 0),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "pass_count": sum(1 for row in requirements if row["status"] == "PASS"),
        "requirement_count": len(requirements),
        "requirements": requirements,
        "failures": [row["requirement_id"] for row in requirements if row["status"] != "PASS"],
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Non-alias selector deployment readiness",
        f"status: {payload.get('status', '')}",
        f"scenario_count: {payload.get('scenario_count', 0)}",
        f"profile_count: {payload.get('profile_count', 0)}",
        f"deployment_label_count: {payload.get('deployment_label_count', 0)}",
        f"operator_launch: {payload.get('operator_launch_pass_count', 0)}/{payload.get('operator_launch_check_count', 0)}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        "",
        "Requirements:",
    ]
    for row in payload.get("requirements", []):
        if isinstance(row, Mapping):
            lines.append(f"- {row.get('requirement_id', '')}: {row.get('status', '')} - {row.get('description', '')}")
            lines.append(f"  evidence: {row.get('evidence', '')}")
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.extend(["", "Failures:"])
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = check_readiness(
        robustness_dir=args.robustness_dir,
        selector_package_dir=args.selector_package_dir,
        readme=args.readme,
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_csv is not None:
        rows = payload.get("requirements", [])
        write_csv(args.output_csv, rows if isinstance(rows, list) else [])
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> int:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
