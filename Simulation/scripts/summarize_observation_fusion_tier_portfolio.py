#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


RUNTIME_CONTRACT = (
    "offline portfolio summary only; runtime observation-fusion tiers consume image "
    "left_u_px/left_v_px/measured_disparity_px and do not consume PX4/RTK truth"
)
RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def as_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: object, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def resolve_path(path_text: str, *, root_dir: Path) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return root_dir / path


def observation_count(tier: Mapping[str, object]) -> int:
    adapter = tier.get("adapter", {})
    if not isinstance(adapter, Mapping):
        return 0
    if str(adapter.get("class", "")) == "RepeatedObservationFusion":
        return as_int(adapter.get("min_observations"))
    return as_int(adapter.get("min_bursts")) * as_int(adapter.get("min_observations_per_burst"), 1)


def scenario_from_summary_dir(result_dir: Path) -> str:
    metrics_path = result_dir / "multiburst_observation_metrics.json"
    if metrics_path.is_file():
        metrics = read_json(metrics_path)
        if metrics.get("scenario"):
            return str(metrics["scenario"])
    return result_dir.name


def matching_method_rows(source_csv: Path, *, method: str) -> list[dict[str, object]]:
    if source_csv.name == "multiburst_observation_method_aggregate.csv":
        result_root = source_csv.parent.parent
        rows: list[dict[str, object]] = []
        for result_dir in sorted(result_root.iterdir()):
            summary_csv = result_dir / "multiburst_observation_method_summary.csv"
            if not result_dir.is_dir() or not summary_csv.is_file():
                continue
            for row in read_csv_rows(summary_csv):
                if row.get("method") == method:
                    out: dict[str, object] = dict(row)
                    out["scenario"] = row.get("scenario") or scenario_from_summary_dir(result_dir)
                    out["scenario_dir"] = result_dir.name
                    out["source_csv"] = str(summary_csv)
                    rows.append(out)
        return rows

    rows = []
    scenario = source_csv.parent.name
    for row in read_csv_rows(source_csv):
        if row.get("method") == method:
            out = dict(row)
            out["scenario"] = row.get("scenario") or scenario
            out["scenario_dir"] = scenario
            out["source_csv"] = str(source_csv)
            rows.append(out)
    return rows


def tier_scenario_rows(
    manifest: Mapping[str, object],
    *,
    root_dir: Path,
    include_single_source: bool = False,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    tiers = manifest.get("tiers", [])
    if not isinstance(tiers, list):
        raise ValueError("manifest tiers must be a list")
    for tier in tiers:
        if not isinstance(tier, Mapping):
            continue
        if str((tier.get("adapter") or {}).get("class", "")) != "MultiBurstObservationFusion":
            continue
        selection_scope = str(tier.get("selection_scope", ""))
        if selection_scope != "all_scenario_aggregate" and not include_single_source:
            continue
        method = str(tier.get("method", ""))
        source_csv = resolve_path(str(tier.get("source_csv", "")), root_dir=root_dir)
        for row in matching_method_rows(source_csv, method=method):
            rows.append(
                {
                    "tier": tier.get("tier", ""),
                    "method": method,
                    "selection_scope": selection_scope,
                    "scenario": row.get("scenario", ""),
                    "scenario_dir": row.get("scenario_dir", ""),
                    "status": row.get("status", ""),
                    "observation_count": observation_count(tier),
                    "range_rmse_raw_mean_m": as_float(row.get("range_rmse_raw_mean_m")),
                    "range_rmse_corrected_mean_m": as_float(row.get("range_rmse_corrected_mean_m")),
                    "position_rmse_raw_mean_m": as_float(row.get("position_rmse_raw_mean_m")),
                    "position_rmse_corrected_mean_m": as_float(row.get("position_rmse_corrected_mean_m")),
                    "target_max_abs_error_corrected_max_m": as_float(
                        row.get("target_max_abs_error_corrected_max_m")
                    ),
                    "target_rmse_corrected_mean_m": as_float(row.get("target_rmse_corrected_mean_m")),
                    "runtime_truth_used": str(row.get("runtime_truth_used", "")).lower() == "true",
                    "runtime_contract": RUNTIME_CONTRACT,
                    "rdf_axis_contract": RDF_AXIS_CONTRACT,
                    "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                }
            )
    return rows


def best_rows_by_scenario(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    best_by_scenario: dict[str, Mapping[str, object]] = {}
    for row in rows:
        if row.get("status") != "STRICT_PASS" or bool(row.get("runtime_truth_used")):
            continue
        scenario = str(row.get("scenario", ""))
        current = best_by_scenario.get(scenario)
        key = (
            as_float(row.get("target_max_abs_error_corrected_max_m"), float("inf")),
            as_float(row.get("range_rmse_corrected_mean_m"), float("inf")),
            as_int(row.get("observation_count"), 999999),
        )
        current_key = (
            as_float(current.get("target_max_abs_error_corrected_max_m"), float("inf")),
            as_float(current.get("range_rmse_corrected_mean_m"), float("inf")),
            as_int(current.get("observation_count"), 999999),
        ) if current is not None else (float("inf"), float("inf"), 999999)
        if key < current_key:
            best_by_scenario[scenario] = row
    return [dict(row) for _, row in sorted(best_by_scenario.items())]


def summary_payload(
    *,
    manifest: Mapping[str, object],
    scenario_rows: Sequence[Mapping[str, object]],
    oracle_rows: Sequence[Mapping[str, object]],
    include_single_source: bool = False,
) -> dict[str, object]:
    default_tier = str(manifest.get("default_recommendation", ""))
    default_rows = [row for row in scenario_rows if row.get("tier") == default_tier]
    default_target_worst = max(
        (as_float(row.get("target_max_abs_error_corrected_max_m")) for row in default_rows),
        default=0.0,
    )
    oracle_target_worst = max(
        (as_float(row.get("target_max_abs_error_corrected_max_m")) for row in oracle_rows),
        default=0.0,
    )
    return {
        "status": "PASS" if scenario_rows and oracle_rows else "FAIL",
        "mode": "offline_observation_fusion_tier_portfolio",
        "default_tier": default_tier,
        "scenario_count": len({str(row.get("scenario", "")) for row in scenario_rows}),
        "tier_scenario_row_count": len(scenario_rows),
        "oracle_row_count": len(oracle_rows),
        "include_single_source_tiers": include_single_source,
        "included_selection_scopes": sorted({str(row.get("selection_scope", "")) for row in scenario_rows}),
        "default_target_worst_m": default_target_worst,
        "oracle_target_worst_m": oracle_target_worst,
        "oracle_improves_default_target_worst": oracle_target_worst < default_target_worst,
        "runtime_truth_used": any(bool(row.get("runtime_truth_used")) for row in scenario_rows),
        "runtime_contract": RUNTIME_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], oracle_rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Observation-fusion tier portfolio summary",
        "",
        f"status: {summary.get('status', '')}",
        f"default_tier: {summary.get('default_tier', '')}",
        f"scenario_count: {summary.get('scenario_count', 0)}",
        f"include_single_source_tiers: {summary.get('include_single_source_tiers', False)}",
        f"included_selection_scopes: {summary.get('included_selection_scopes', [])}",
        f"default_target_worst_m: {as_float(summary.get('default_target_worst_m')):.6f}",
        f"oracle_target_worst_m: {as_float(summary.get('oracle_target_worst_m')):.6f}",
        f"oracle_improves_default_target_worst: {summary.get('oracle_improves_default_target_worst', False)}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', True)}",
        f"runtime_contract: {summary.get('runtime_contract', '')}",
        "",
        "Oracle rows:",
    ]
    for row in oracle_rows:
        lines.append(
            (
                "- {scenario}: tier={tier}, obs={obs}, target={target:.6f}m, "
                "range={range_rmse:.6f}m"
            ).format(
                scenario=row.get("scenario", ""),
                tier=row.get("tier", ""),
                obs=row.get("observation_count", ""),
                target=as_float(row.get("target_max_abs_error_corrected_max_m")),
                range_rmse=as_float(row.get("range_rmse_corrected_mean_m")),
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize offline observation-fusion tier portfolio upper bounds.")
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--root-dir", type=Path, default=Path("."))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--include-single-source",
        action="store_true",
        help=(
            "Include single-source tiers in the offline oracle comparison. By default only "
            "all_scenario_aggregate tiers are compared so scenario labels stay comparable."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest = read_json(args.manifest_json)
    scenario_rows = tier_scenario_rows(
        manifest,
        root_dir=args.root_dir,
        include_single_source=args.include_single_source,
    )
    oracle_rows = best_rows_by_scenario(scenario_rows)
    summary = summary_payload(
        manifest=manifest,
        scenario_rows=scenario_rows,
        oracle_rows=oracle_rows,
        include_single_source=args.include_single_source,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "observation_fusion_tier_scenario_metrics.csv", scenario_rows)
    write_csv(args.output_dir / "observation_fusion_tier_oracle_selection.csv", oracle_rows)
    write_json(args.output_dir / "observation_fusion_tier_portfolio_summary.json", summary)
    write_summary_txt(args.output_dir / "observation_fusion_tier_portfolio_summary.txt", summary, oracle_rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
