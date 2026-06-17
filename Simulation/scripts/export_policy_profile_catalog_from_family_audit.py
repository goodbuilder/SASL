#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    load_catalog,
    method_contract_fields,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a policy-aware StereoDistance scenario profile catalog from a family calibration "
            "export audit. The audit chooses calibration-time source policies using truth, while the "
            "exported catalog profiles consume only raw stereo-center RDF/disparity at runtime."
        )
    )
    parser.add_argument("--base-catalog-json", type=Path, required=True)
    parser.add_argument("--family-audit-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-catalog-json", type=Path, required=True)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite_or_blank(value: object) -> float | str:
    number = finite_float(value, float("nan"))
    return number if math.isfinite(number) else ""


def catalog_root(catalog_json: Path) -> Path:
    return catalog_json.parent


def scenario_purpose(base_catalog: Mapping[str, object], scenario: str) -> str:
    for row in base_catalog.get("profiles", []):
        if isinstance(row, Mapping) and row.get("scenario") == scenario:
            return str(row.get("purpose", ""))
    return ""


def source_seed_score(rows: Sequence[Mapping[str, str]], source_seed: str) -> tuple[int, float, float, float, str]:
    source_rows = [row for row in rows if row.get("source_seed") == source_seed]
    pass_count = sum(1 for row in source_rows if row.get("pass_family_profile_transfer") == "True")
    target_max = max(
        (finite_float(row.get("target_max_abs_error_corrected_m"), float("-inf")) for row in source_rows),
        default=float("inf"),
    )
    rmse_max = max(
        (finite_float(row.get("validation_range_rmse_corrected_m"), float("-inf")) for row in source_rows),
        default=float("inf"),
    )
    improvement_min = min(
        (finite_float(row.get("validation_range_rmse_improvement_m"), float("inf")) for row in source_rows),
        default=float("-inf"),
    )
    return (-pass_count, target_max, rmse_max, -improvement_min, source_seed)


def summarize_source_rows(rows: Sequence[Mapping[str, str]]) -> dict[str, object]:
    pass_count = sum(1 for row in rows if row.get("pass_family_profile_transfer") == "True")
    rmse_values = [
        finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
        for row in rows
    ]
    rmse_values = [value for value in rmse_values if math.isfinite(value)]
    improvement_values = [
        finite_float(row.get("validation_range_rmse_improvement_m"), float("nan"))
        for row in rows
    ]
    improvement_values = [value for value in improvement_values if math.isfinite(value)]
    target_max_values = [
        finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
        for row in rows
    ]
    target_max_values = [value for value in target_max_values if math.isfinite(value)]
    target_abs_values = [
        finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
        for row in rows
    ]
    target_abs_values = [value for value in target_abs_values if math.isfinite(value)]
    return {
        "selected_source_transfer_count": len(rows),
        "selected_source_transfer_pass_count": pass_count,
        "selected_source_pass_all_transfer": bool(rows) and pass_count == len(rows),
        "selected_source_validation_range_rmse_max_m": max(rmse_values) if rmse_values else float("nan"),
        "selected_source_validation_range_rmse_improvement_min_m": (
            min(improvement_values) if improvement_values else float("nan")
        ),
        "selected_source_target_abs_mean_error_max_m": max(target_abs_values) if target_abs_values else float("nan"),
        "selected_source_target_max_abs_error_max_m": max(target_max_values) if target_max_values else float("nan"),
    }


def choose_source_seed(
    transfer_rows: Sequence[Mapping[str, str]],
    *,
    scenario: str,
    source_policy: str,
) -> str:
    rows = [
        row
        for row in transfer_rows
        if row.get("scenario") == scenario and row.get("source_policy") == source_policy
    ]
    source_seeds = sorted({str(row.get("source_seed", "")) for row in rows if row.get("source_seed")})
    if not source_seeds:
        raise ValueError(f"no transfer rows for {scenario}/{source_policy}")
    return min(source_seeds, key=lambda source_seed: source_seed_score(rows, source_seed))


def export_row_for(
    export_rows: Sequence[Mapping[str, str]],
    *,
    scenario: str,
    source_policy: str,
    source_seed: str,
) -> Mapping[str, str]:
    for row in export_rows:
        if (
            row.get("scenario") == scenario
            and row.get("source_policy") == source_policy
            and row.get("source_seed") == source_seed
        ):
            return row
    raise ValueError(f"no export row for {scenario}/{source_policy}/{source_seed}")


def selected_policy_row(
    policy_rows: Sequence[Mapping[str, str]],
    *,
    scenario: str,
    source_policy: str,
) -> Mapping[str, str]:
    for row in policy_rows:
        if row.get("scenario") == scenario and row.get("source_policy") == source_policy:
            return row
    return {}


def relative_to_catalog_root(path: Path, output_catalog_json: Path) -> str:
    root = catalog_root(output_catalog_json).resolve()
    return str(path.resolve().relative_to(root))


def build_policy_catalog(
    *,
    base_catalog_json: Path,
    family_audit_dir: Path,
    output_dir: Path,
    output_catalog_json: Path,
) -> dict[str, object]:
    base_catalog = load_catalog(base_catalog_json)
    recommendations_path = family_audit_dir / "family_calibration_profile_scenario_policy_recommendations.csv"
    transfer_path = family_audit_dir / "family_calibration_profile_transfer.csv"
    exports_path = family_audit_dir / "family_calibration_profile_exports.csv"
    for path in (recommendations_path, transfer_path, exports_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    recommendation_rows = read_csv(recommendations_path)
    transfer_rows = read_csv(transfer_path)
    export_rows = read_csv(exports_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_catalog_json.parent.mkdir(parents=True, exist_ok=True)
    profiles: list[dict[str, object]] = []
    catalog_rows: list[dict[str, object]] = []
    for recommendation in recommendation_rows:
        scenario = str(recommendation.get("scenario", ""))
        source_policy = str(recommendation.get("recommended_source_policy", recommendation.get("source_policy", "")))
        if not scenario or not source_policy:
            continue
        source_seed = choose_source_seed(transfer_rows, scenario=scenario, source_policy=source_policy)
        selected_transfer_rows = [
            row
            for row in transfer_rows
            if row.get("scenario") == scenario
            and row.get("source_policy") == source_policy
            and row.get("source_seed") == source_seed
        ]
        selected_source_summary = summarize_source_rows(selected_transfer_rows)
        export_row = export_row_for(
            export_rows,
            scenario=scenario,
            source_policy=source_policy,
            source_seed=source_seed,
        )
        source_profile = Path(str(export_row.get("active_profile", "")))
        if not source_profile.is_file():
            raise FileNotFoundError(source_profile)
        profile_payload = json.loads(source_profile.read_text(encoding="utf-8"))
        method = str(profile_payload.get("method", export_row.get("exported_method", "")))
        contract = method_contract_fields(method)
        destination = output_dir / f"{scenario}_runtime_profile.json"
        shutil.copyfile(source_profile, destination)
        policy_row = selected_policy_row(recommendation_rows, scenario=scenario, source_policy=source_policy)
        profile_row = {
            "scenario": scenario,
            "purpose": scenario_purpose(base_catalog, scenario),
            "method": method,
            "registration_model": contract["registration_model"],
            "residual_model": contract["residual_model"],
            "regularization_model": contract["regularization_model"],
            "runtime_profile_family": contract["runtime_profile_family"],
            "selection_note": contract["selection_note"],
            "recommendation_mode": "policy_family_calibration_export",
            "source_policy": source_policy,
            "source_seed": source_seed,
            "source_profile_path": str(source_profile),
            "catalog_profile_path": relative_to_catalog_root(destination, output_catalog_json),
            "profile_exists": True,
            "pass_cross_seed_replay": str(policy_row.get("heldout_replay_pass_all", "")).lower() == "true",
            "passed_seed_count": int(float(policy_row.get("heldout_replay_pass_count", 0) or 0)),
            "seed_count": int(float(policy_row.get("heldout_replay_count", 0) or 0)),
            "validation_range_rmse_max_m": finite_float(
                policy_row.get("validation_range_rmse_corrected_max_m"),
                float("nan"),
            ),
            "validation_range_rmse_improvement_min_m": finite_float(
                policy_row.get("validation_range_rmse_improvement_min_m"),
                float("nan"),
            ),
            "target_abs_mean_error_max_m": finite_float(
                policy_row.get("target_abs_mean_error_corrected_max_m"),
                float("nan"),
            ),
            "target_max_abs_error_max_m": finite_float(
                policy_row.get("target_max_abs_error_max_m"),
                float("nan"),
            ),
            **selected_source_summary,
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "runtime_input_contract": "scenario-policy profile consumes raw stereo-center RDF/disparity only; no runtime truth",
        }
        for key in (
            "validation_range_rmse_max_m",
            "validation_range_rmse_improvement_min_m",
            "target_abs_mean_error_max_m",
            "target_max_abs_error_max_m",
            "selected_source_validation_range_rmse_max_m",
            "selected_source_validation_range_rmse_improvement_min_m",
            "selected_source_target_abs_mean_error_max_m",
            "selected_source_target_max_abs_error_max_m",
        ):
            profile_row[key] = finite_or_blank(profile_row.get(key))
        profiles.append(profile_row)
        catalog_rows.append(dict(profile_row))
    catalog = {
        "mode": "scenario_profile_catalog",
        "selection_contract": (
            "Scenario source and policy are selected by calibration-time family replay validation; "
            "runtime selection/export uses raw stereo-center RDF/disparity only."
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "catalog profiles consume raw stereo-center RDF/disparity only",
        "source_base_catalog_json": str(base_catalog_json),
        "source_family_audit_dir": str(family_audit_dir),
        "profiles": profiles,
    }
    selected_source_target_max_values = [
        finite_float(row.get("selected_source_target_max_abs_error_max_m"), float("nan"))
        for row in profiles
    ]
    selected_source_target_max_values = [
        value for value in selected_source_target_max_values if math.isfinite(value)
    ]
    summary = {
        "mode": "policy_family_profile_catalog_export",
        "profile_count": len(profiles),
        "pass_profile_count": sum(1 for row in profiles if bool(row.get("pass_cross_seed_replay"))),
        "pass_policy_profile_catalog": bool(profiles) and all(bool(row.get("pass_cross_seed_replay")) for row in profiles),
        "selected_source_pass_count": sum(1 for row in profiles if bool(row.get("selected_source_pass_all_transfer"))),
        "selected_source_pass_all": bool(profiles)
        and all(bool(row.get("selected_source_pass_all_transfer")) for row in profiles),
        "selected_source_target_max_abs_error_max_m": max(selected_source_target_max_values)
        if selected_source_target_max_values
        else "",
        "output_catalog_json": str(output_catalog_json),
        "output_dir": str(output_dir),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "catalog profiles consume raw stereo-center RDF/disparity only",
    }
    write_json(output_catalog_json, catalog)
    write_json(output_dir / "policy_profile_catalog_summary.json", summary)
    write_csv(output_dir / "policy_profile_catalog.csv", catalog_rows)
    write_summary_txt(output_dir / "policy_profile_catalog_summary.txt", summary, catalog_rows)
    return summary


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Policy family profile catalog export",
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Summary:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = "nan" if math.isnan(value) else f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Profiles:"])
    for row in rows:
        lines.append(
            "{scenario}: policy={policy} seed={seed} method={method} heldout={passed}/{count} "
            "rmse_max={rmse_max} target_max={target_max} selected_source_target_max={source_target_max}".format(
                scenario=row.get("scenario", ""),
                policy=row.get("source_policy", ""),
                seed=row.get("source_seed", ""),
                method=row.get("method", ""),
                passed=row.get("passed_seed_count", ""),
                count=row.get("seed_count", ""),
                rmse_max=row.get("validation_range_rmse_max_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
                source_target_max=row.get("selected_source_target_max_abs_error_max_m", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    summary = build_policy_catalog(
        base_catalog_json=args.base_catalog_json,
        family_audit_dir=args.family_audit_dir,
        output_dir=args.output_dir,
        output_catalog_json=args.output_catalog_json,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
