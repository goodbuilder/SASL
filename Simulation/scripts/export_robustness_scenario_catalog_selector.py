#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
RUNTIME_DEPLOYMENT_FRONTIER_EXPORT_SCRIPT = SCRIPT_DIR / "export_stereodistance_runtime_deployment_frontier.py"
OPERATOR_VIEW_LAUNCH_CHECK_SCRIPT = SCRIPT_DIR / "check_stereodistance_operator_view_launch.py"
NON_ALIAS_SELECTOR_READINESS_CHECK_SCRIPT = SCRIPT_DIR / "check_non_alias_selector_deployment_readiness.py"

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
SELECTION_CONTRACT = (
    "Runtime profile selection may use only explicit deployment/scenario labels "
    "from operator configuration or deployment metadata. It must not inspect "
    "PX4/RTK/NED truth, validation errors, target-window labels, or replay metrics."
)
FORBIDDEN_RUNTIME_FIELDS = [
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "px4_north_m",
    "px4_east_m",
    "px4_down_m",
    "rtk_truth_m",
    "ned_truth_m",
    "validation_range_rmse_m",
    "target_window_error_m",
    "replay_pass",
]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a truth-free StereoDistance runtime label selector package "
            "from a robustness sweep scenario profile catalog."
        )
    )
    parser.add_argument("--robustness-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--baseline-scenario",
        default="nominal",
        help="Scenario profile used when no deployment/scenario label matches.",
    )
    parser.add_argument(
        "--with-deployment-frontier",
        action="store_true",
        help="Also export runtime_deployment_frontier/ for the selector manifest.",
    )
    parser.add_argument(
        "--with-operator-launch-checks",
        action="store_true",
        help="Also run operator-view launch dry-runs for default and selector deployment labels.",
    )
    parser.add_argument(
        "--with-readiness-check",
        action="store_true",
        help="Also run the non-alias selector deployment readiness checklist.",
    )
    parser.add_argument(
        "--readme",
        type=Path,
        default=ROOT_DIR / "README.md",
        help="README used by --with-readiness-check for coordinate/runtime contract documentation evidence.",
    )
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_label(value: object) -> str:
    text = str(value).strip().lower()
    cleaned = []
    for char in text:
        cleaned.append(char if char.isalnum() else "_")
    label = "".join(cleaned).strip("_")
    return label or "unlabeled"


def command_tail(text: str, limit: int = 4000) -> str:
    return text[-limit:] if len(text) > limit else text


def relative_to_output(path: Path, output_dir: Path) -> str:
    return str(path.resolve().relative_to(output_dir.resolve()))


def selector_deployment_labels(selector_json: Path) -> list[str]:
    manifest = load_json(selector_json)
    label_map = manifest.get("label_profile_map", {})
    labels = sorted(str(label) for label in label_map.keys()) if isinstance(label_map, Mapping) else []
    baseline_selector = str(manifest.get("baseline_selector", ""))
    if "" not in labels:
        labels.insert(0, "")
    baseline_label = ""
    if baseline_selector and isinstance(label_map, Mapping):
        for label, profile_label in label_map.items():
            if str(profile_label) == baseline_selector:
                baseline_label = str(label)
                break
    if baseline_label and baseline_label in labels:
        labels.remove(baseline_label)
        labels.insert(1 if labels and labels[0] == "" else 0, baseline_label)
    return list(dict.fromkeys(labels))


def read_optional_json(path: Path) -> dict[str, object]:
    return load_json(path) if path.is_file() else {}


def export_deployment_frontier(selector_json: Path, output_dir: Path) -> dict[str, object]:
    frontier_dir = output_dir / "runtime_deployment_frontier"
    command = [
        sys.executable,
        str(RUNTIME_DEPLOYMENT_FRONTIER_EXPORT_SCRIPT),
        "--selector-json",
        str(selector_json),
        "--output-dir",
        str(frontier_dir),
    ]
    result = subprocess.run(command, cwd=ROOT_DIR, check=False, text=True, capture_output=True)
    payload = read_optional_json(frontier_dir / "runtime_deployment_frontier.json")
    summary = payload.get("summary", {}) if isinstance(payload, Mapping) else {}
    parsed_summary = dict(summary) if isinstance(summary, Mapping) else {}
    parsed_summary.update(
        {
            "returncode": result.returncode,
            "command": " ".join(command),
            "stdout_tail": command_tail(result.stdout),
            "stderr_tail": command_tail(result.stderr),
        }
    )
    return parsed_summary


def run_operator_launch_checks(selector_json: Path, output_dir: Path) -> dict[str, object]:
    checks_dir = output_dir / "operator_view_launch_checks"
    checks_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for index, label in enumerate(selector_deployment_labels(selector_json)):
        artifact_label = safe_label(label or "default")
        output_json = checks_dir / f"operator_view_launch_check_{index:02d}_{artifact_label}.json"
        output_txt = checks_dir / f"operator_view_launch_check_{index:02d}_{artifact_label}.txt"
        command = [
            sys.executable,
            str(OPERATOR_VIEW_LAUNCH_CHECK_SCRIPT),
            "--selector-json",
            str(selector_json),
            "--output-json",
            str(output_json),
            "--output-txt",
            str(output_txt),
        ]
        if label:
            command.extend(["--deployment-label", label])
        result = subprocess.run(command, cwd=ROOT_DIR, check=False, text=True, capture_output=True)
        payload = read_optional_json(output_json)
        rows.append(
            {
                "deployment_label": label or "__default__",
                "status": payload.get("status", "FAIL" if result.returncode else "PASS"),
                "returncode": result.returncode,
                "selected_label": payload.get("selected_label", ""),
                "profile_json": payload.get("profile_json", ""),
                "method": payload.get("method", ""),
                "bbox_adjustment_arg_present": payload.get("bbox_adjustment_arg_present", False),
                "runtime_truth_used": payload.get("runtime_truth_used", True),
                "failure_count": payload.get("failure_count", 0),
                "failures": ";".join(str(item) for item in payload.get("failures", []))
                if isinstance(payload.get("failures", []), list)
                else "",
                "output_json": str(output_json),
                "output_txt": str(output_txt),
                "stdout_tail": command_tail(result.stdout, 1200),
                "stderr_tail": command_tail(result.stderr, 1200),
            }
        )
    pass_count = sum(1 for row in rows if row.get("status") == "PASS")
    truth_used_count = sum(1 for row in rows if bool(row.get("runtime_truth_used", True)))
    bbox_arg_count = sum(1 for row in rows if bool(row.get("bbox_adjustment_arg_present", False)))
    summary = {
        "mode": "runtime_selector_operator_launch_checks",
        "status": "PASS" if rows and pass_count == len(rows) and truth_used_count == 0 and bbox_arg_count == len(rows) else "FAIL",
        "selector_json": str(selector_json),
        "check_count": len(rows),
        "pass_count": pass_count,
        "bbox_adjustment_arg_count": bbox_arg_count,
        "runtime_truth_used_count": truth_used_count,
        "runtime_truth_used": truth_used_count > 0,
        "rows_csv": str(checks_dir / "operator_view_launch_checks.csv"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    write_csv(checks_dir / "operator_view_launch_checks.csv", rows)
    write_json(checks_dir / "operator_view_launch_checks.json", {"summary": summary, "rows": rows})
    lines = [
        "Runtime selector operator-view launch checks",
        f"status: {summary['status']}",
        f"selector_json: {selector_json}",
        f"check_count: {summary['check_count']}",
        f"pass_count: {summary['pass_count']}",
        f"bbox_adjustment_arg_count: {summary['bbox_adjustment_arg_count']}",
        f"runtime_truth_used_count: {summary['runtime_truth_used_count']}",
        "",
        "Checks:",
    ]
    for row in rows:
        lines.append(
            "- {label}: status={status}, selected={selected}, method={method}, bbox_arg={bbox}".format(
                label=row.get("deployment_label", ""),
                status=row.get("status", ""),
                selected=row.get("selected_label", ""),
                method=row.get("method", ""),
                bbox=row.get("bbox_adjustment_arg_present", False),
            )
        )
    (checks_dir / "operator_view_launch_checks.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def run_readiness_check(
    *,
    robustness_dir: Path,
    selector_package_dir: Path,
    readme: Path,
) -> dict[str, object]:
    output_json = selector_package_dir / "non_alias_selector_deployment_readiness.json"
    output_csv = selector_package_dir / "non_alias_selector_deployment_readiness.csv"
    output_txt = selector_package_dir / "non_alias_selector_deployment_readiness.txt"
    command = [
        sys.executable,
        str(NON_ALIAS_SELECTOR_READINESS_CHECK_SCRIPT),
        "--robustness-dir",
        str(robustness_dir),
        "--selector-package-dir",
        str(selector_package_dir),
        "--readme",
        str(readme),
        "--output-json",
        str(output_json),
        "--output-csv",
        str(output_csv),
        "--output-txt",
        str(output_txt),
    ]
    result = subprocess.run(command, cwd=ROOT_DIR, check=False, text=True, capture_output=True)
    summary = read_optional_json(output_json)
    summary.update(
        {
            "returncode": result.returncode,
            "command": " ".join(command),
            "stdout_tail": command_tail(result.stdout),
            "stderr_tail": command_tail(result.stderr),
            "output_json": str(output_json),
            "output_csv": str(output_csv),
            "output_txt": str(output_txt),
        }
    )
    return summary


def catalog_rows(robustness_dir: Path) -> list[dict[str, object]]:
    catalog = load_json(robustness_dir / "scenario_profile_catalog.json")
    rows = catalog.get("rows", [])
    if not isinstance(rows, list) or not rows:
        raise ValueError("scenario_profile_catalog.json has no rows")
    parsed = [dict(row) for row in rows if isinstance(row, Mapping)]
    if not parsed:
        raise ValueError("scenario_profile_catalog.json rows are invalid")
    return parsed


def copy_profile_package(
    *,
    robustness_dir: Path,
    output_dir: Path,
    scenario: str,
    profile_relpath: object,
) -> dict[str, object]:
    source_profile = (robustness_dir / str(profile_relpath)).resolve()
    if not source_profile.is_file():
        raise FileNotFoundError(f"missing catalog profile for {scenario}: {source_profile}")
    profile_payload = load_json(source_profile)
    if "RDF(right,down,forward)" not in str(profile_payload.get("axis_contract", "")):
        raise ValueError(f"profile missing RDF axis contract: {source_profile}")
    runtime_inputs = profile_payload.get("runtime_inputs", [])
    if not isinstance(runtime_inputs, list) or any(
        token in " ".join(str(item).lower() for item in runtime_inputs)
        for token in ("truth", "px4", "rtk", "ned", "validation", "replay")
    ):
        raise ValueError(f"profile runtime inputs are not truth-free: {source_profile}")

    profile_label = f"scenario_{safe_label(scenario)}"
    package_dir = output_dir / "packages" / profile_label
    package_dir.mkdir(parents=True, exist_ok=True)
    output_profile = package_dir / "active_runtime_profile.json"
    shutil.copy2(source_profile, output_profile)
    return {
        "scenario": scenario,
        "profile_label": profile_label,
        "source_profile_json": str(source_profile),
        "profile_json": relative_to_output(output_profile, output_dir),
        "package_dir": relative_to_output(package_dir, output_dir),
        "profile_sha256": sha256_file(output_profile),
        "profile_id": profile_payload.get("profile_id", ""),
        "method": profile_payload.get("method", ""),
        "profile_mode": profile_payload.get("mode", ""),
        "axis_contract": profile_payload.get("axis_contract", ""),
        "runtime_inputs": ";".join(str(item) for item in runtime_inputs),
        "runtime_truth_used": False,
    }


def export_selector_package(
    *,
    robustness_dir: Path,
    output_dir: Path,
    baseline_scenario: str = "nominal",
    with_deployment_frontier: bool = False,
    with_operator_launch_checks: bool = False,
    with_readiness_check: bool = False,
    readme: Path | None = None,
) -> dict[str, object]:
    robustness_dir = robustness_dir.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = catalog_rows(robustness_dir)
    profile_records: dict[str, dict[str, object]] = {}
    label_profile_map: dict[str, str] = {}
    label_rows: list[dict[str, object]] = []

    for row in rows:
        scenario = str(row.get("scenario", "")).strip()
        if not scenario:
            raise ValueError("catalog row missing scenario")
        record = copy_profile_package(
            robustness_dir=robustness_dir,
            output_dir=output_dir,
            scenario=scenario,
            profile_relpath=row.get("catalog_profile_path", ""),
        )
        profile_label = str(record["profile_label"])
        profile_records[profile_label] = record
        labels = list(dict.fromkeys([scenario, safe_label(scenario), f"scenario:{scenario}", f"scenario_{safe_label(scenario)}"]))
        for label in labels:
            label_profile_map[label] = profile_label
            label_rows.append(
                {
                    "deployment_label": label,
                    "scenario": scenario,
                    "profile_label": profile_label,
                    "method": record.get("method", ""),
                    "runtime_truth_used": False,
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                }
            )

    baseline_key = baseline_scenario if baseline_scenario in label_profile_map else safe_label(baseline_scenario)
    baseline_selector = label_profile_map.get(baseline_key)
    if not baseline_selector:
        known = ", ".join(sorted(str(row.get("scenario", "")) for row in rows))
        raise ValueError(f"baseline scenario {baseline_scenario!r} not found. Known: {known}")

    evidence_dir = output_dir / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    evidence_paths: dict[str, str] = {}
    for name in (
        "robustness_summary.json",
        "scenario_profile_catalog.json",
        "scenario_profile_catalog.csv",
        "scenario_coordinate_contracts.csv",
        "robustness_acceptance_recheck_after_catalog_gate.json",
    ):
        source = robustness_dir / name
        if source.is_file():
            target = evidence_dir / name
            shutil.copy2(source, target)
            evidence_paths[name.removesuffix(".json").removesuffix(".csv")] = relative_to_output(target, output_dir)

    profile_paths = {
        label: str(record["profile_json"])
        for label, record in sorted(profile_records.items())
    }
    package_paths = {
        label: str(record["package_dir"])
        for label, record in sorted(profile_records.items())
    }
    manifest = {
        "mode": "stereodistance_runtime_profile_label_selector",
        "baseline_selector": baseline_selector,
        "label_profile_map": label_profile_map,
        "profile_paths": profile_paths,
        "package_paths": package_paths,
        "validated_evidence": evidence_paths,
        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
        "forbidden_runtime_fields": FORBIDDEN_RUNTIME_FIELDS,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": SELECTION_CONTRACT,
        "selector_source_mode": "robustness_scenario_profile_catalog",
        "runtime_truth_used": False,
    }
    selector_json = output_dir / "runtime_profile_label_selector.json"
    write_json(selector_json, manifest)

    profile_rows = list(profile_records.values())
    write_csv(output_dir / "selector_package_profiles.csv", profile_rows)
    write_csv(output_dir / "selector_package_labels.csv", label_rows)
    frontier_summary = export_deployment_frontier(selector_json, output_dir) if with_deployment_frontier else {}
    operator_launch_summary = (
        run_operator_launch_checks(selector_json, output_dir) if with_operator_launch_checks else {}
    )
    preliminary_summary = {
        "mode": "robustness_scenario_catalog_selector_export",
        "status": "PASS",
        "robustness_dir": str(robustness_dir),
        "selector_json": str(selector_json),
        "baseline_scenario": baseline_scenario,
        "baseline_profile_label": baseline_selector,
        "profile_count": len(profile_rows),
        "deployment_label_count": len(label_profile_map),
        "profile_rows_csv": str(output_dir / "selector_package_profiles.csv"),
        "label_rows_csv": str(output_dir / "selector_package_labels.csv"),
        "deployment_frontier_status": frontier_summary.get("status", "SKIPPED")
        if with_deployment_frontier
        else "SKIPPED",
        "operator_launch_status": operator_launch_summary.get("status", "SKIPPED")
        if with_operator_launch_checks
        else "SKIPPED",
        "operator_launch_check_count": operator_launch_summary.get("check_count", 0),
        "operator_launch_pass_count": operator_launch_summary.get("pass_count", 0),
        "runtime_truth_used": bool(frontier_summary.get("runtime_truth_used", False))
        or bool(operator_launch_summary.get("runtime_truth_used", False)),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": SELECTION_CONTRACT,
    }
    write_json(output_dir / "selector_package_export_summary.json", preliminary_summary)
    readiness_summary = (
        run_readiness_check(
            robustness_dir=robustness_dir,
            selector_package_dir=output_dir,
            readme=(readme or (ROOT_DIR / "README.md")).resolve(),
        )
        if with_readiness_check
        else {}
    )
    extra_failure_reasons = []
    if with_deployment_frontier and frontier_summary.get("status") != "PASS":
        extra_failure_reasons.append("deployment_frontier")
    if with_operator_launch_checks and operator_launch_summary.get("status") != "PASS":
        extra_failure_reasons.append("operator_launch_checks")
    if with_readiness_check and readiness_summary.get("status") != "PASS":
        extra_failure_reasons.append("readiness_check")
    runtime_truth_used = (
        bool(frontier_summary.get("runtime_truth_used", False))
        or bool(operator_launch_summary.get("runtime_truth_used", False))
        or bool(readiness_summary.get("runtime_truth_used", False))
    )
    if runtime_truth_used:
        extra_failure_reasons.append("runtime_truth")

    summary = {
        "mode": "robustness_scenario_catalog_selector_export",
        "status": "PASS" if not extra_failure_reasons else "FAIL",
        "robustness_dir": str(robustness_dir),
        "selector_json": str(selector_json),
        "baseline_scenario": baseline_scenario,
        "baseline_profile_label": baseline_selector,
        "profile_count": len(profile_rows),
        "deployment_label_count": len(label_profile_map),
        "profile_rows_csv": str(output_dir / "selector_package_profiles.csv"),
        "label_rows_csv": str(output_dir / "selector_package_labels.csv"),
        "deployment_frontier_status": frontier_summary.get("status", "SKIPPED")
        if with_deployment_frontier
        else "SKIPPED",
        "operator_launch_status": operator_launch_summary.get("status", "SKIPPED")
        if with_operator_launch_checks
        else "SKIPPED",
        "readiness_status": readiness_summary.get("status", "SKIPPED") if with_readiness_check else "SKIPPED",
        "operator_launch_check_count": operator_launch_summary.get("check_count", 0),
        "operator_launch_pass_count": operator_launch_summary.get("pass_count", 0),
        "readiness_requirement_count": readiness_summary.get("requirement_count", 0),
        "readiness_pass_count": readiness_summary.get("pass_count", 0),
        "runtime_truth_used": runtime_truth_used,
        "failure_reasons": extra_failure_reasons,
        "deployment_frontier_summary": frontier_summary,
        "operator_launch_summary": operator_launch_summary,
        "readiness_summary": readiness_summary,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": SELECTION_CONTRACT,
    }
    write_json(output_dir / "selector_package_export_summary.json", summary)
    write_summary_text(output_dir / "selector_package_export_summary.txt", summary, profile_rows, label_rows)
    return summary


def write_summary_text(
    path: Path,
    summary: Mapping[str, object],
    profile_rows: Sequence[Mapping[str, object]],
    label_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Robustness scenario catalog selector package",
        f"status: {summary.get('status', '')}",
        f"selector_json: {summary.get('selector_json', '')}",
        f"baseline_scenario: {summary.get('baseline_scenario', '')}",
        f"baseline_profile_label: {summary.get('baseline_profile_label', '')}",
        f"profile_count: {summary.get('profile_count', 0)}",
        f"deployment_label_count: {summary.get('deployment_label_count', 0)}",
        f"deployment_frontier_status: {summary.get('deployment_frontier_status', '')}",
        f"operator_launch_status: {summary.get('operator_launch_status', '')}",
        f"operator_launch_check_count: {summary.get('operator_launch_check_count', 0)}",
        f"operator_launch_pass_count: {summary.get('operator_launch_pass_count', 0)}",
        f"readiness_status: {summary.get('readiness_status', '')}",
        f"readiness_pass_count: {summary.get('readiness_pass_count', 0)}",
        f"readiness_requirement_count: {summary.get('readiness_requirement_count', 0)}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', False)}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Profiles:",
    ]
    for row in profile_rows:
        lines.append(f"- {row.get('profile_label', '')}: {row.get('method', '')}")
    lines.extend(["", "Deployment labels:"])
    for row in label_rows:
        lines.append(f"- {row.get('deployment_label', '')} -> {row.get('profile_label', '')}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_selector_package(
        robustness_dir=args.robustness_dir,
        output_dir=args.output_dir,
        baseline_scenario=args.baseline_scenario,
        with_deployment_frontier=args.with_deployment_frontier,
        with_operator_launch_checks=args.with_operator_launch_checks,
        with_readiness_check=args.with_readiness_check,
        readme=args.readme,
    )


def main() -> None:
    summary = run()
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
