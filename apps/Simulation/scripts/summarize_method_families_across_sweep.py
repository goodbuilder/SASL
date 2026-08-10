#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


DEFAULT_FAMILIES = (
    "inverse_depth:inverse_depth",
    "residual_disparity:residual_disparity",
    "affine_runtime:affine",
    "truncated_ls:truncated_ls",
    "huber:huber",
    "tukey:tukey",
    "ransac:ransac",
    "trimmed:trimmed",
    "product_ray:disparity_product_ray_scale",
    "sim3:sim3",
)


@dataclass(frozen=True)
class FamilySpec:
    name: str
    substrings: tuple[str, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rank multiple calibration method families across a sweep from existing "
            "method_comparison.csv files. Metrics are offline validation metrics in "
            "stereo-center RDF; runtime applicability is taken from runtime_profile_family."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--family",
        action="append",
        default=[],
        help=(
            "Family spec 'name:substring[,substring...]'. Repeatable. "
            "Defaults cover inverse-depth, residual, affine, robust Sim(3), and product-ray families."
        ),
    )
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def parse_family_specs(raw_specs: Sequence[str]) -> list[FamilySpec]:
    specs = list(raw_specs) if raw_specs else list(DEFAULT_FAMILIES)
    families: list[FamilySpec] = []
    for spec in specs:
        if ":" not in spec:
            raise ValueError(f"family spec must be 'name:substring[,substring...]': {spec!r}")
        name, raw_substrings = spec.split(":", 1)
        name = name.strip()
        substrings = tuple(item.strip() for item in raw_substrings.split(",") if item.strip())
        if not name or not substrings:
            raise ValueError(f"family spec must include a name and at least one substring: {spec!r}")
        families.append(FamilySpec(name=name, substrings=substrings))
    return families


def scenario_seed_from_path(path: Path, sweep_dir: Path) -> tuple[str, str]:
    relative = path.relative_to(sweep_dir)
    parts = relative.parts
    if len(parts) >= 3:
        return parts[0], parts[1]
    return "", ""


def method_matches_family(method: str, family: FamilySpec) -> bool:
    return any(substring in method for substring in family.substrings)


def collect_family_rows(
    sweep_dir: Path,
    families: Sequence[FamilySpec],
    *,
    target_error_limit_m: float,
) -> list[dict[str, object]]:
    rows_out: list[dict[str, object]] = []
    for path in sorted(sweep_dir.glob("*/*/method_comparison.csv")):
        rows = read_csv(path)
        if not rows:
            continue
        scenario, seed = scenario_seed_from_path(path, sweep_dir)
        raw_row = next((row for row in rows if row.get("method") == "raw"), {})
        raw_rmse = finite_float(raw_row.get("validation_range_rmse_m"))
        for row in rows:
            method = str(row.get("method", ""))
            if method == "raw":
                continue
            matched_families = [family for family in families if method_matches_family(method, family)]
            if not matched_families:
                continue
            validation_rmse = finite_float(row.get("validation_range_rmse_m"))
            target_max = finite_float(row.get("validation_target_max_abs_error_m"))
            improvement = (
                raw_rmse - validation_rmse
                if math.isfinite(raw_rmse) and math.isfinite(validation_rmse)
                else float("nan")
            )
            for family in matched_families:
                rows_out.append(
                    {
                        "family": family.name,
                        "matched_substrings": ",".join(family.substrings),
                        "scenario": scenario,
                        "seed": seed,
                        "method": method,
                        "runtime_profile_family": row.get("runtime_profile_family", ""),
                        "registration_model": row.get("registration_model", ""),
                        "residual_model": row.get("residual_model", ""),
                        "regularization_model": row.get("regularization_model", ""),
                        "model_complexity": row.get("model_complexity", ""),
                        "validation_range_rmse_raw_m": raw_row.get("validation_range_rmse_m", ""),
                        "validation_range_rmse_m": row.get("validation_range_rmse_m", ""),
                        "validation_range_rmse_improvement_vs_raw_m": improvement,
                        "validation_target_abs_mean_error_m": row.get("validation_target_abs_mean_error_m", ""),
                        "validation_target_max_abs_error_m": row.get("validation_target_max_abs_error_m", ""),
                        "pass_target_max_limit": bool(
                            math.isfinite(target_max) and target_max <= target_error_limit_m
                        ),
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                    }
                )
    return rows_out


def mean(values: Sequence[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    return float(sum(finite) / len(finite)) if finite else None


def summarize_family_rows(
    rows: Sequence[Mapping[str, object]],
    families: Sequence[FamilySpec],
    *,
    sweep_dir: Path,
    target_error_limit_m: float,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    summary_rows: list[dict[str, object]] = []
    for family in families:
        family_rows = [row for row in rows if row.get("family") == family.name]
        rmse_values = [finite_float(row.get("validation_range_rmse_m")) for row in family_rows]
        target_max_values = [finite_float(row.get("validation_target_max_abs_error_m")) for row in family_rows]
        improvement_values = [
            finite_float(row.get("validation_range_rmse_improvement_vs_raw_m")) for row in family_rows
        ]
        best_row = min(
            family_rows,
            key=lambda row: (
                finite_float(row.get("validation_range_rmse_m"), float("inf")),
                finite_float(row.get("validation_target_max_abs_error_m"), float("inf")),
                finite_float(row.get("model_complexity"), float("inf")),
                str(row.get("method", "")),
            ),
            default={},
        )
        pass_count = sum(1 for row in family_rows if bool(row.get("pass_target_max_limit")))
        pass_fraction = float(pass_count / len(family_rows)) if family_rows else 0.0
        scenario_count = len({str(row.get("scenario", "")) for row in family_rows})
        runtime_families = sorted({str(row.get("runtime_profile_family", "")) for row in family_rows if row.get("runtime_profile_family")})
        best_by_run: list[Mapping[str, object]] = []
        run_keys = sorted({(str(row.get("scenario", "")), str(row.get("seed", ""))) for row in family_rows})
        for scenario, seed in run_keys:
            run_rows = [
                row
                for row in family_rows
                if str(row.get("scenario", "")) == scenario and str(row.get("seed", "")) == seed
            ]
            best_run_row = min(
                run_rows,
                key=lambda row: (
                    0 if bool(row.get("pass_target_max_limit")) else 1,
                    finite_float(row.get("validation_target_max_abs_error_m"), float("inf")),
                    finite_float(row.get("validation_range_rmse_m"), float("inf")),
                    finite_float(row.get("model_complexity"), float("inf")),
                    str(row.get("method", "")),
                ),
                default={},
            )
            if best_run_row:
                best_by_run.append(best_run_row)
        oracle_rmse_values = [finite_float(row.get("validation_range_rmse_m")) for row in best_by_run]
        oracle_target_values = [finite_float(row.get("validation_target_max_abs_error_m")) for row in best_by_run]
        oracle_improvement_values = [
            finite_float(row.get("validation_range_rmse_improvement_vs_raw_m")) for row in best_by_run
        ]
        oracle_pass_count = sum(1 for row in best_by_run if bool(row.get("pass_target_max_limit")))
        oracle_methods = sorted({str(row.get("method", "")) for row in best_by_run if row.get("method")})
        summary_rows.append(
            {
                "family": family.name,
                "matched_substrings": ",".join(family.substrings),
                "candidate_row_count": len(family_rows),
                "scenario_count": scenario_count,
                "pass_target_max_limit_count": pass_count,
                "pass_target_max_limit_fraction": pass_fraction,
                "pass_target_max_limit_all": bool(family_rows) and pass_count == len(family_rows),
                "validation_range_rmse_mean_m": mean(rmse_values),
                "validation_range_rmse_max_m": max([v for v in rmse_values if math.isfinite(v)], default=None),
                "validation_target_max_abs_error_max_m": max(
                    [v for v in target_max_values if math.isfinite(v)], default=None
                ),
                "validation_range_rmse_improvement_min_m": min(
                    [v for v in improvement_values if math.isfinite(v)], default=None
                ),
                "best_candidate_method": best_row.get("method", ""),
                "best_candidate_scenario": best_row.get("scenario", ""),
                "best_candidate_validation_range_rmse_m": best_row.get("validation_range_rmse_m", ""),
                "runtime_profile_families": ",".join(runtime_families),
                "oracle_best_run_count": len(best_by_run),
                "oracle_best_pass_target_max_limit_count": oracle_pass_count,
                "oracle_best_pass_target_max_limit_fraction": (
                    float(oracle_pass_count / len(best_by_run)) if best_by_run else 0.0
                ),
                "oracle_best_pass_target_max_limit_all": bool(best_by_run) and oracle_pass_count == len(best_by_run),
                "oracle_best_validation_range_rmse_mean_m": mean(oracle_rmse_values),
                "oracle_best_validation_range_rmse_max_m": max(
                    [v for v in oracle_rmse_values if math.isfinite(v)], default=None
                ),
                "oracle_best_validation_target_max_abs_error_max_m": max(
                    [v for v in oracle_target_values if math.isfinite(v)], default=None
                ),
                "oracle_best_validation_range_rmse_improvement_min_m": min(
                    [v for v in oracle_improvement_values if math.isfinite(v)], default=None
                ),
                "oracle_best_methods": ",".join(oracle_methods),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    summary_rows.sort(
        key=lambda row: (
            0 if bool(row.get("pass_target_max_limit_all")) else 1,
            -finite_float(row.get("pass_target_max_limit_fraction"), 0.0),
            finite_float(row.get("validation_target_max_abs_error_max_m"), float("inf")),
            finite_float(row.get("validation_range_rmse_max_m"), float("inf")),
            finite_float(row.get("validation_range_rmse_mean_m"), float("inf")),
            str(row.get("family", "")),
        )
    )
    best_family = summary_rows[0] if summary_rows else {}
    oracle_ranked = sorted(
        summary_rows,
        key=lambda row: (
            0 if bool(row.get("oracle_best_pass_target_max_limit_all")) else 1,
            -finite_float(row.get("oracle_best_pass_target_max_limit_fraction"), 0.0),
            finite_float(row.get("oracle_best_validation_target_max_abs_error_max_m"), float("inf")),
            finite_float(row.get("oracle_best_validation_range_rmse_max_m"), float("inf")),
            finite_float(row.get("oracle_best_validation_range_rmse_mean_m"), float("inf")),
            str(row.get("family", "")),
        ),
    )
    best_oracle_family = oracle_ranked[0] if oracle_ranked else {}
    summary = {
        "mode": "method_family_across_sweep_summary",
        "sweep_dir": str(sweep_dir),
        "family_count": len(summary_rows),
        "method_family_row_count": len(rows),
        "target_error_limit_m": target_error_limit_m,
        "recommended_family_by_validation": best_family.get("family", ""),
        "recommended_family_best_method": best_family.get("best_candidate_method", ""),
        "recommended_family_pass_target_max_limit_count": best_family.get("pass_target_max_limit_count", ""),
        "recommended_family_candidate_row_count": best_family.get("candidate_row_count", ""),
        "recommended_family_pass_target_max_limit_fraction": best_family.get(
            "pass_target_max_limit_fraction", None
        ),
        "recommended_family_pass_target_max_limit_all": best_family.get("pass_target_max_limit_all", False),
        "recommended_family_target_max_abs_error_max_m": best_family.get(
            "validation_target_max_abs_error_max_m", None
        ),
        "oracle_recommended_family_by_validation": best_oracle_family.get("family", ""),
        "oracle_recommended_family_pass_target_max_limit_count": best_oracle_family.get(
            "oracle_best_pass_target_max_limit_count", ""
        ),
        "oracle_recommended_family_run_count": best_oracle_family.get("oracle_best_run_count", ""),
        "oracle_recommended_family_pass_target_max_limit_fraction": best_oracle_family.get(
            "oracle_best_pass_target_max_limit_fraction", None
        ),
        "oracle_recommended_family_pass_target_max_limit_all": best_oracle_family.get(
            "oracle_best_pass_target_max_limit_all", False
        ),
        "oracle_recommended_family_target_max_abs_error_max_m": best_oracle_family.get(
            "oracle_best_validation_target_max_abs_error_max_m", None
        ),
        "oracle_recommended_family_methods": best_oracle_family.get("oracle_best_methods", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "This is an offline family comparison over method_comparison.csv validation metrics. "
            "Runtime methods must use the exported runtime_profile_family and fixed calibration parameters; "
            "runtime selection must not use truth metrics."
        ),
    }
    return summary_rows, summary


def write_summary_txt(path: Path, summary: Mapping[str, object], summary_rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Method family across sweep summary",
        "",
        f"recommended_family_by_validation: {summary.get('recommended_family_by_validation', '')}",
        f"recommended_family_best_method: {summary.get('recommended_family_best_method', '')}",
        f"recommended_family_pass_target_max_limit_count: {summary.get('recommended_family_pass_target_max_limit_count', '')}",
        f"recommended_family_candidate_row_count: {summary.get('recommended_family_candidate_row_count', '')}",
        f"recommended_family_pass_target_max_limit_fraction: {summary.get('recommended_family_pass_target_max_limit_fraction', '')}",
        f"recommended_family_pass_target_max_limit_all: {summary.get('recommended_family_pass_target_max_limit_all', '')}",
        f"recommended_family_target_max_abs_error_max_m: {summary.get('recommended_family_target_max_abs_error_max_m', '')}",
        f"oracle_recommended_family_by_validation: {summary.get('oracle_recommended_family_by_validation', '')}",
        f"oracle_recommended_family_pass_target_max_limit_count: {summary.get('oracle_recommended_family_pass_target_max_limit_count', '')}",
        f"oracle_recommended_family_run_count: {summary.get('oracle_recommended_family_run_count', '')}",
        f"oracle_recommended_family_pass_target_max_limit_fraction: {summary.get('oracle_recommended_family_pass_target_max_limit_fraction', '')}",
        f"oracle_recommended_family_pass_target_max_limit_all: {summary.get('oracle_recommended_family_pass_target_max_limit_all', '')}",
        f"oracle_recommended_family_target_max_abs_error_max_m: {summary.get('oracle_recommended_family_target_max_abs_error_max_m', '')}",
        f"oracle_recommended_family_methods: {summary.get('oracle_recommended_family_methods', '')}",
        f"target_error_limit_m: {summary.get('target_error_limit_m', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        "",
        "Families:",
    ]
    for row in summary_rows:
        lines.append(
            "- {family}: pass={passed}/{count}, rmse_mean={rmse_mean}, rmse_max={rmse_max}, "
            "target_max={target_max}, oracle_pass={oracle_pass}/{oracle_count}, "
            "oracle_target_max={oracle_target_max}, best={best}, runtime={runtime}".format(
                family=row.get("family", ""),
                passed=row.get("pass_target_max_limit_count", ""),
                count=row.get("candidate_row_count", ""),
                rmse_mean=row.get("validation_range_rmse_mean_m", ""),
                rmse_max=row.get("validation_range_rmse_max_m", ""),
                target_max=row.get("validation_target_max_abs_error_max_m", ""),
                oracle_pass=row.get("oracle_best_pass_target_max_limit_count", ""),
                oracle_count=row.get("oracle_best_run_count", ""),
                oracle_target_max=row.get("oracle_best_validation_target_max_abs_error_max_m", ""),
                best=row.get("best_candidate_method", ""),
                runtime=row.get("runtime_profile_families", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_method_families_across_sweep(
    sweep_dir: Path,
    output_dir: Path,
    *,
    family_specs: Sequence[str],
    target_error_limit_m: float,
) -> dict[str, object]:
    families = parse_family_specs(family_specs)
    rows = collect_family_rows(sweep_dir, families, target_error_limit_m=target_error_limit_m)
    summary_rows, summary = summarize_family_rows(
        rows,
        families,
        sweep_dir=sweep_dir,
        target_error_limit_m=target_error_limit_m,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "method_family_rows.csv", rows)
    write_csv(output_dir / "method_family_summary.csv", summary_rows)
    write_json(output_dir / "method_family_summary.json", summary)
    write_summary_txt(output_dir / "method_family_summary.txt", summary, summary_rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = summarize_method_families_across_sweep(
        args.sweep_dir,
        args.output_dir,
        family_specs=args.family,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
