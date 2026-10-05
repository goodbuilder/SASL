#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import read_csv, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_from_catalog import load_catalog  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
    DEFAULT_INTRINSIC_SCALE_GAP_SOURCE_SCENARIO,
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    build_feature_library,
    load_transfer_matrix,
    runtime_measurement_features,
    select_source_by_runtime_measurement,
    source_scenarios_for_mode,
    sweep_calibration_csvs,
)
from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    PackagedRuntimeDeploymentSelector,
    finite_float,
    parse_override_rule,
    override_applies,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Apply a packaged StereoDistance runtime deployment selector to raw RDF/disparity rows. "
            "The selector uses raw feature fields only; truth fields are rejected for selection."
        )
    )
    parser.add_argument("--selector-json", type=Path, required=True)
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="")
    parser.add_argument("--audit-all-sweep-seeds", action="store_true")
    parser.add_argument(
        "--reference-audit-csv",
        type=Path,
        help="Optional precision-override audit CSV used to compare selected labels for each scenario/seed.",
    )
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def csv_rows_for_split(path: Path, split: str = "") -> list[dict[str, str]]:
    rows = read_csv(path)
    if not split:
        return rows
    return [row for row in rows if str(row.get("split", "")) == split]


def resolve_packaged_path(selector_json: Path, relative_or_abs: object) -> Path:
    path = Path(str(relative_or_abs))
    return path if path.is_absolute() else selector_json.parent / path


def apply_selector_to_rows(
    *,
    selector_json: Path,
    sweep_dir: Path,
    rows: Sequence[Mapping[str, str]],
) -> dict[str, object]:
    selector = read_json(selector_json)
    catalog_json = resolve_packaged_path(selector_json, selector["scenario_catalog_json"])
    transfer_matrix_csv = resolve_packaged_path(selector_json, selector["transfer_matrix_csv"])
    catalog = load_catalog(catalog_json)
    source_scenarios = source_scenarios_for_mode(
        catalog=catalog,
        catalog_json=catalog_json,
        minimal_set_json=None,
        candidate_source_mode="all_catalog",
    )
    library = build_feature_library(
        sweep_dir=sweep_dir,
        source_scenarios=source_scenarios,
        exclude_csv=None,
    )
    features = runtime_measurement_features(rows)
    transfer_matrix = load_transfer_matrix(transfer_matrix_csv)
    baseline = select_source_by_runtime_measurement(
        rows=rows,
        library=library,
        available_source_scenarios=source_scenarios,
        selection_policy=str(selector.get("known_selector_policy", "intrinsic_scale_gap_transfer_risk")),
        transfer_matrix=transfer_matrix,
        intrinsic_scale_gap_source_scenario=DEFAULT_INTRINSIC_SCALE_GAP_SOURCE_SCENARIO,
        intrinsic_scale_gap_product_ratio_p10_threshold=DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
    )
    packaged_selector = PackagedRuntimeDeploymentSelector(selector)
    packaged_selection = packaged_selector.select(features)
    rule = packaged_selection["override_rule"]
    use_override = bool(packaged_selection["override_applied"])
    selected_label = str(packaged_selection["selected_label"])
    selected_source_scenario = (
        str(selector.get("override_profile_label", "single_pooled"))
        if use_override
        else str(baseline.get("selected_source_scenario", ""))
    )
    result = {
        "selected_label": selected_label,
        "selected_source_scenario": selected_source_scenario,
        "baseline_source_scenario": baseline.get("selected_source_scenario", ""),
        "preliminary_source_scenario": baseline.get("preliminary_source_scenario", ""),
        "override_applied": use_override,
        "override_rule_json": json.dumps(rule, ensure_ascii=False, allow_nan=False),
        "override_profile_json": selector.get("override_profile_json", ""),
        "selection_policy": selector.get("known_selector_policy", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": selector.get("selection_contract", ""),
    }
    for name, value in features.items():
        result[f"feature_{name}"] = value
    return result


def apply_selector_to_csv(
    *,
    selector_json: Path,
    sweep_dir: Path,
    input_csv: Path,
    output_dir: Path,
    split: str = "",
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = csv_rows_for_split(input_csv, split)
    if not rows:
        raise ValueError("no input rows after split filter")
    result = apply_selector_to_rows(selector_json=selector_json, sweep_dir=sweep_dir, rows=rows)
    result.update({"input_csv": str(input_csv), "split": split, "input_row_count": len(rows)})
    write_csv(output_dir / "runtime_deployment_selector_result.csv", [result])
    write_json(output_dir / "runtime_deployment_selector_result.json", result)
    write_summary_txt(output_dir / "runtime_deployment_selector_result.txt", result)
    return result


def row_key(row: Mapping[str, object]) -> tuple[str, str]:
    return str(row.get("target_scenario", "")), str(row.get("target_seed", ""))


def reference_rows(path: Path | None) -> dict[tuple[str, str], dict[str, str]]:
    if path is None:
        return {}
    return {row_key(row): row for row in read_csv(path)}


def apply_selector_to_sweep(
    *,
    selector_json: Path,
    sweep_dir: Path,
    output_dir: Path,
    split: str = "",
    reference_audit_csv: Path | None = None,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    reference = reference_rows(reference_audit_csv)
    rows_out: list[dict[str, object]] = []
    for scenario, seed, csv_path in sweep_calibration_csvs(sweep_dir):
        rows = csv_rows_for_split(csv_path, split)
        if not rows:
            continue
        result = apply_selector_to_rows(selector_json=selector_json, sweep_dir=sweep_dir, rows=rows)
        key = (scenario, seed)
        ref = reference.get(key, {})
        result.update(
            {
                "target_scenario": scenario,
                "target_seed": seed,
                "input_csv": str(csv_path),
                "split": split,
                "input_row_count": len(rows),
                "reference_selected_candidate": ref.get("selected_candidate", ""),
                "reference_override_applied": ref.get("override_applied", ""),
                "matches_reference_selected_candidate": (
                    str(result.get("selected_label", "")) == str(ref.get("selected_candidate", ""))
                    if ref
                    else ""
                ),
                "matches_reference_override": (
                    str(result.get("override_applied", "")) == str(ref.get("override_applied", ""))
                    if ref
                    else ""
                ),
            }
        )
        rows_out.append(result)
    if not rows_out:
        raise ValueError("no sweep seed CSVs were selected")
    match_rows = [
        row for row in rows_out if row.get("matches_reference_selected_candidate") in (True, False)
    ]
    override_match_rows = [row for row in rows_out if row.get("matches_reference_override") in (True, False)]
    summary = {
        "mode": "runtime_deployment_selector_sweep_audit",
        "audit_count": len(rows_out),
        "split": split,
        "reference_audit_csv": str(reference_audit_csv or ""),
        "selected_candidate_match_count": sum(
            1 for row in match_rows if bool(row.get("matches_reference_selected_candidate"))
        ),
        "selected_candidate_match_total": len(match_rows),
        "selected_candidate_match_all": bool(match_rows)
        and all(bool(row.get("matches_reference_selected_candidate")) for row in match_rows),
        "override_match_count": sum(1 for row in override_match_rows if bool(row.get("matches_reference_override"))),
        "override_match_total": len(override_match_rows),
        "override_match_all": bool(override_match_rows)
        and all(bool(row.get("matches_reference_override")) for row in override_match_rows),
        "override_count": sum(1 for row in rows_out if bool(row.get("override_applied"))),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Sweep audit applies packaged runtime selector to raw measurement rows only; reference replay "
            "metrics are used only for offline comparison."
        ),
    }
    write_csv(output_dir / "runtime_deployment_selector_sweep_audit.csv", rows_out)
    write_json(output_dir / "runtime_deployment_selector_sweep_summary.json", summary)
    write_sweep_summary_txt(output_dir / "runtime_deployment_selector_sweep_summary.txt", summary, rows_out)
    return summary


def write_summary_txt(path: Path, result: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance runtime deployment selector result",
        "",
        f"selected_label: {result.get('selected_label', '')}",
        f"selected_source_scenario: {result.get('selected_source_scenario', '')}",
        f"baseline_source_scenario: {result.get('baseline_source_scenario', '')}",
        f"override_applied: {result.get('override_applied', '')}",
        f"selection_policy: {result.get('selection_policy', '')}",
        f"axis_contract: {result.get('axis_contract', '')}",
        f"public_topic_contract: {result.get('public_topic_contract', '')}",
        f"selection_contract: {result.get('selection_contract', '')}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_sweep_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance runtime deployment selector sweep audit",
        "",
        f"audit_count: {summary.get('audit_count', '')}",
        f"selected_candidate_match: {summary.get('selected_candidate_match_count', '')}/{summary.get('selected_candidate_match_total', '')}",
        f"override_match: {summary.get('override_match_count', '')}/{summary.get('override_match_total', '')}",
        f"override_count: {summary.get('override_count', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            (
                "- {scenario}/{seed}: selected={selected}, baseline={baseline}, override={override}, "
                "reference={reference}, match={match}"
            ).format(
                scenario=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                selected=row.get("selected_label", ""),
                baseline=row.get("baseline_source_scenario", ""),
                override=row.get("override_applied", ""),
                reference=row.get("reference_selected_candidate", ""),
                match=row.get("matches_reference_selected_candidate", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.audit_all_sweep_seeds:
        summary = apply_selector_to_sweep(
            selector_json=args.selector_json,
            sweep_dir=args.sweep_dir,
            output_dir=args.output_dir,
            split=args.split,
            reference_audit_csv=args.reference_audit_csv,
        )
        print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
        return 0
    if args.input_csv is None:
        raise SystemExit("--input-csv is required unless --audit-all-sweep-seeds is set")
    result = apply_selector_to_csv(
        selector_json=args.selector_json,
        sweep_dir=args.sweep_dir,
        input_csv=args.input_csv,
        output_dir=args.output_dir,
        split=args.split,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
