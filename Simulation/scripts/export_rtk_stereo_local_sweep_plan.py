#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from itertools import product
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
RUNTIME_INPUT_CONTRACT = (
    "local sweep candidates are offline fit plans; deployable runtime profiles must consume stereo-center "
    "RDF(right,down,forward) plus measured disparity and publish FRD+range without PX4/RTK/NED truth"
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a de-duplicated local method-search plan around the current RTK stereo best family. "
            "This does not fit methods; it writes candidate configs for the next automatic sweep."
        )
    )
    parser.add_argument("--catalog-audit-json", type=Path, required=True)
    parser.add_argument("--active-profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-candidates", type=int, default=64)
    parser.add_argument(
        "--exclude-methods-csv",
        type=Path,
        action="append",
        default=[],
        help=(
            "Optional CSV file with a method column to exclude from the generated plan. "
            "Can be repeated to avoid re-exporting already evaluated local-sweep candidates."
        ),
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


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def finite_float(value: object, default: float = float("inf")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def active_bin_count(profile: Mapping[str, object]) -> int:
    lut = profile.get("forward_residual_raw_forward_lut", {})
    if isinstance(lut, Mapping):
        x_values = lut.get("x_values_m", [])
        if isinstance(x_values, list) and x_values:
            return len(x_values)
    method = str(profile.get("method", ""))
    for token in method.split("_"):
        if token.startswith("bins"):
            try:
                return int(token.removeprefix("bins"))
            except ValueError:
                pass
    return 11


def active_prior_strength(profile: Mapping[str, object]) -> float:
    lut = profile.get("forward_residual_raw_forward_lut", {})
    if isinstance(lut, Mapping):
        return finite_float(lut.get("prior_strength_samples"), 8.0)
    return 8.0


def existing_methods(catalog: Mapping[str, object]) -> set[str]:
    names: set[str] = set()
    for key in ("candidates", "families"):
        for row in catalog.get(key, []):
            if not isinstance(row, Mapping):
                continue
            method = str(row.get("method", row.get("target", ""))).strip()
            if method:
                names.add(method)
    return names


def excluded_methods_from_csv(paths: Sequence[Path]) -> set[str]:
    names: set[str] = set()
    for path in paths:
        for row in read_csv(path):
            method = str(row.get("method", "")).strip()
            if method:
                names.add(method)
    return names


def method_name(row: Mapping[str, object]) -> str:
    parts = ["sim3"]
    if row["residual_basis"] == "inverse_depth":
        parts.append("inverse_depth")
    elif row["residual_basis"] == "range_normalized":
        parts.append("range_normalized")
    elif row["residual_basis"] == "meter":
        parts.append("meter")
    elif row["residual_basis"] == "blended_disparity_inverse_depth":
        parts.append("blended_disp_invdepth")
    if parse_bool(row["monotone"]):
        parts.append("monotone")
    if row["robust_loss"] != "none":
        parts.append(str(row["robust_loss"]))
    if finite_float(row["target_weight"], 0.0) > 0.0:
        parts.append(f"targetw{format_token(row['target_weight'])}")
    if finite_float(row["residual_gain"], 1.0) != 1.0:
        parts.append(f"gain{format_token(row['residual_gain'])}")
    if finite_float(row["prior_strength_samples"], 8.0) != 8.0:
        parts.append(f"prior{format_token(row['prior_strength_samples'])}")
    parts.extend(["binned_residual", str(row["forward_key"]), f"bins{int(row['bin_count'])}"])
    return "_".join(parts)


def format_token(value: object) -> str:
    parsed = finite_float(value, 0.0)
    text = f"{parsed:g}".replace("-", "m").replace(".", "p")
    return text


def closeness_score(row: Mapping[str, object], *, active_bins: int, active_prior: float) -> tuple[object, ...]:
    return (
        abs(int(row["bin_count"]) - active_bins),
        abs(finite_float(row["prior_strength_samples"]) - active_prior),
        abs(finite_float(row["residual_gain"]) - 1.0),
        finite_float(row["target_weight"]),
        0 if row["residual_basis"] == "inverse_depth" else 1,
        0 if row["robust_loss"] == "none" else 1,
        0 if row["forward_key"] == "raw_forward" else 1,
        0 if not parse_bool(row["monotone"]) else 1,
        str(row.get("method", "")),
    )


def generate_candidate_rows(
    *,
    catalog: Mapping[str, object],
    active_profile: Mapping[str, object],
    max_candidates: int,
    extra_excluded_methods: set[str] | None = None,
) -> list[dict[str, object]]:
    active_method = str(active_profile.get("method", catalog.get("selected_method", "")))
    active_bins = active_bin_count(active_profile)
    active_prior = active_prior_strength(active_profile)
    known_methods = existing_methods(catalog)
    known_methods.add(active_method)
    if extra_excluded_methods:
        known_methods.update(extra_excluded_methods)
    axes = {
        "residual_basis": ["inverse_depth", "range_normalized", "meter", "blended_disparity_inverse_depth"],
        "bin_count": sorted({5, 7, 9, active_bins, 13, 15, 17}),
        "prior_strength_samples": sorted({0.0, 4.0, active_prior, 12.0, 16.0}),
        "residual_gain": [0.5, 0.75, 1.0, 1.25],
        "target_weight": [0.0, 0.25, 0.5, 1.0],
        "robust_loss": ["none", "huber", "cauchy"],
        "monotone": [False, True],
        "forward_key": ["raw_forward", "base_forward"],
    }
    rows: list[dict[str, object]] = []
    rank = 0
    for values in product(*(axes[key] for key in axes)):
        row = dict(zip(axes.keys(), values))
        method = method_name(row)
        if method in known_methods:
            continue
        rank += 1
        rows.append(
            {
                "method": method,
                "search_family": "sim3_local_residual_sweep",
                "base_method": active_method,
                "registration_model": "sim3",
                "residual_basis": row["residual_basis"],
                "bin_count": row["bin_count"],
                "prior_strength_samples": row["prior_strength_samples"],
                "residual_gain": row["residual_gain"],
                "target_weight": row["target_weight"],
                "robust_loss": row["robust_loss"],
                "monotone": row["monotone"],
                "forward_key": row["forward_key"],
                "fit_inputs": "raw_right_m,raw_down_m,raw_forward_m,measured_disparity_px,offline_truth_rdf",
                "runtime_inputs": "raw_right_m,raw_down_m,raw_forward_m,raw_disparity_px",
                "forbidden_runtime_inputs": "calibration_truth,px4_rtk_truth,true_disparity,true_image_feature_location",
                "selection_gate": "validation_rmse_improves_raw_and_1852m_max_abs_error_le_10m_and_axis_nonregression",
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
                "_rank_seed": rank,
            }
        )
    rows = sorted(rows, key=lambda row: closeness_score(row, active_bins=active_bins, active_prior=active_prior))
    for index, row in enumerate(rows[:max_candidates], start=1):
        row["priority_rank"] = index
        row.pop("_rank_seed", None)
    return rows[:max_candidates]


def export_plan(
    *,
    catalog_audit_json: Path,
    active_profile_json: Path,
    output_dir: Path,
    max_candidates: int,
    exclude_methods_csv: Sequence[Path] = (),
) -> dict[str, object]:
    catalog = load_json(catalog_audit_json)
    active_profile = load_json(active_profile_json)
    output_dir.mkdir(parents=True, exist_ok=True)
    extra_excluded_methods = excluded_methods_from_csv(exclude_methods_csv)
    candidate_rows = generate_candidate_rows(
        catalog=catalog,
        active_profile=active_profile,
        max_candidates=max_candidates,
        extra_excluded_methods=extra_excluded_methods,
    )
    configs_dir = output_dir / "candidate_configs"
    configs_dir.mkdir(parents=True, exist_ok=True)
    config_records: list[dict[str, object]] = []
    for row in candidate_rows:
        config_path = configs_dir / f"{row['priority_rank']:03d}_{row['method']}.json"
        config = {
            key: value
            for key, value in row.items()
            if key
            not in {
                "priority_rank",
                "fit_inputs",
                "runtime_inputs",
                "forbidden_runtime_inputs",
                "selection_gate",
                "axis_contract",
                "public_topic_contract",
                "runtime_input_contract",
                "runtime_truth_used",
            }
        }
        config.update(
            {
                "mode": "rtk_stereo_local_sweep_candidate",
                "priority_rank": row["priority_rank"],
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "runtime_truth_used": False,
            }
        )
        write_json(config_path, config)
        config_records.append({"priority_rank": row["priority_rank"], "method": row["method"], "config_json": str(config_path)})
    summary = {
        "mode": "rtk_stereo_local_sweep_plan",
        "status": "PASS" if candidate_rows else "FAIL",
        "catalog_audit_json": str(catalog_audit_json),
        "active_profile_json": str(active_profile_json),
        "output_dir": str(output_dir),
        "base_method": active_profile.get("method", catalog.get("selected_method", "")),
        "candidate_count": len(candidate_rows),
        "max_candidates": max_candidates,
        "excluded_existing_method_count": len(existing_methods(catalog)),
        "excluded_extra_method_count": len(extra_excluded_methods),
        "exclude_methods_csv": [str(path) for path in exclude_methods_csv],
        "purpose": "Continue automatic search around the current deployable best family while preserving alternate candidates.",
        "selection_gate": "validation RMSE must improve over raw, 1852m target max abs error <= 10m, RDF axis nonregression, runtime truth-free profile export.",
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
        "candidate_config_dir": str(configs_dir),
        "candidate_configs": config_records,
    }
    write_json(output_dir / "rtk_stereo_local_sweep_plan.json", summary)
    write_csv(output_dir / "rtk_stereo_local_sweep_candidates.csv", candidate_rows)
    write_csv(output_dir / "rtk_stereo_local_sweep_candidate_configs.csv", config_records)
    write_text(output_dir / "rtk_stereo_local_sweep_plan.txt", summary, candidate_rows)
    return summary


def write_text(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "RTK stereo local sweep plan",
        f"status: {summary.get('status', '')}",
        f"base_method: {summary.get('base_method', '')}",
        f"candidate_count: {summary.get('candidate_count', '')}",
        f"excluded_extra_method_count: {summary.get('excluded_extra_method_count', '')}",
        f"purpose: {summary.get('purpose', '')}",
        f"selection_gate: {summary.get('selection_gate', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', '')}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        "",
        "Top candidates:",
    ]
    for row in rows[:20]:
        lines.append(
            (
                "- #{rank} {method}: basis={basis}, bins={bins}, prior={prior}, gain={gain}, "
                "target_weight={target_weight}, robust={robust}, monotone={monotone}, forward={forward}"
            ).format(
                rank=row.get("priority_rank", ""),
                method=row.get("method", ""),
                basis=row.get("residual_basis", ""),
                bins=row.get("bin_count", ""),
                prior=row.get("prior_strength_samples", ""),
                gain=row.get("residual_gain", ""),
                target_weight=row.get("target_weight", ""),
                robust=row.get("robust_loss", ""),
                monotone=row.get("monotone", ""),
                forward=row.get("forward_key", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_plan(
        catalog_audit_json=args.catalog_audit_json,
        active_profile_json=args.active_profile_json,
        output_dir=args.output_dir,
        max_candidates=args.max_candidates,
        exclude_methods_csv=args.exclude_methods_csv,
    )


def main() -> int:
    summary = run()
    compact = {
        key: summary.get(key)
        for key in (
            "status",
            "base_method",
            "candidate_count",
            "max_candidates",
            "excluded_existing_method_count",
            "runtime_truth_used",
            "axis_contract",
            "public_topic_contract",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if summary.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
