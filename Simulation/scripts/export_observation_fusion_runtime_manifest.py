#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


DEFAULT_TIERS = (
    {
        "tier": "minimum_realtime_independent",
        "purpose": "lowest observation count that passed independent bbox-outlier stress",
        "source_csv": "Simulation/reports/experiments/EXP-20260617-202-bbox-outlier-repeated-observation-fusion-extended/repeated_observation_method_summary.csv",
        "method": "repeated_observation_median_n3_trim0p2",
        "adapter_class": "RepeatedObservationFusion",
        "runtime_condition": "Use only when observations are meaningfully independent inside the same timestamp or short burst.",
    },
    {
        "tier": "balanced_independent",
        "purpose": "balanced high-quality independent repeated-observation setting",
        "source_csv": "Simulation/reports/experiments/EXP-20260617-202-bbox-outlier-repeated-observation-fusion-extended/repeated_observation_method_summary.csv",
        "method": "repeated_observation_trimmed_mean_n15_trim0p2",
        "adapter_class": "RepeatedObservationFusion",
        "runtime_condition": "Use when 15 independent observations can be collected within the latency budget.",
    },
    {
        "tier": "upper_bound_independent",
        "purpose": "synthetic high-count upper bound for independent observations",
        "source_csv": "Simulation/reports/experiments/EXP-20260617-209-bbox-outlier-n75-trim0p2/repeated_observation_method_summary.csv",
        "method": "repeated_observation_trimmed_mean_n75_trim0p2",
        "adapter_class": "RepeatedObservationFusion",
        "runtime_condition": "Offline or slow-burst only unless 75 independent observations fit the motion/latency budget.",
    },
    {
        "tier": "common_mode_target_gate",
        "purpose": "target-window preferred common-mode outlier setting",
        "source_csv": "Simulation/reports/experiments/EXP-20260617-212-bbox-outlier-multiburst-common1-confirmation/multiburst_observation_method_summary.csv",
        "method": "multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1",
        "adapter_class": "MultiBurstObservationFusion",
        "runtime_condition": "Use when burst-internal detector/crop failures may be correlated; requires burst-to-burst independence.",
    },
    {
        "tier": "common_mode_aggregate_rmse",
        "purpose": "aggregate RMSE preferred common-mode outlier setting",
        "source_csv": "Simulation/reports/experiments/EXP-20260617-212-bbox-outlier-multiburst-common1-confirmation/multiburst_observation_method_summary.csv",
        "method": "multiburst_observation_inner_trimmed_mean_outer_median_b5x5_trim0p2_common1",
        "adapter_class": "MultiBurstObservationFusion",
        "runtime_condition": "Use when aggregate RMSE is preferred over worst target-window error and 25 observations fit the latency budget.",
    },
)

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
IMAGE_CONTRACT = "left image pixel center u/v and measured stereo disparity in pixels"
RUNTIME_TRUTH_CONTRACT = "runtime adapters consume image u/v/disparity only; PX4/RTK truth is calibration/audit-only"


def finite_float(value: object, default: float = float("nan")) -> float:
    if value in (None, ""):
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def int_value(value: object, default: int = 0) -> int:
    parsed = finite_float(value)
    return int(parsed) if math.isfinite(parsed) else default


def bool_value(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def find_method_row(path: Path, method: str) -> dict[str, str]:
    for row in read_csv_rows(path):
        if row.get("method") == method:
            return row
    raise ValueError(f"method {method!r} not found in {path}")


def repeated_adapter_config(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "class": "RepeatedObservationFusion",
        "min_observations": int_value(row.get("repeat_count")),
        "mode": str(row.get("fusion_mode", "")),
        "trim_fraction": finite_float(row.get("trim_fraction")),
    }


def multiburst_adapter_config(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "class": "MultiBurstObservationFusion",
        "min_bursts": int_value(row.get("burst_count")),
        "min_observations_per_burst": int_value(row.get("observations_per_burst")),
        "inner_mode": str(row.get("inner_mode", "")),
        "outer_mode": str(row.get("outer_mode", "")),
        "trim_fraction": finite_float(row.get("trim_fraction")),
    }


def adapter_config(adapter_class: str, row: Mapping[str, object]) -> dict[str, object]:
    if adapter_class == "RepeatedObservationFusion":
        return repeated_adapter_config(row)
    if adapter_class == "MultiBurstObservationFusion":
        return multiburst_adapter_config(row)
    raise ValueError(f"unsupported adapter class: {adapter_class}")


def evidence_payload(row: Mapping[str, object]) -> dict[str, object]:
    strict_pass = (
        row.get("status", "")
        or ("STRICT_PASS" if bool_value(row.get("all_scenarios_strict_pass")) else "")
    )
    pass_count = row.get("pass_count")
    trial_count = row.get("trial_count")
    if pass_count in (None, ""):
        pass_count = row.get("strict_pass_scenario_count")
    if trial_count in (None, ""):
        trial_count = row.get("scenario_count")
    return {
        "status": strict_pass,
        "pass_count": int_value(pass_count),
        "trial_count": int_value(trial_count),
        "range_rmse_corrected_mean_m": finite_float(
            row.get("range_rmse_corrected_mean_m")
            or row.get("range_rmse_corrected_mean_across_scenarios_m")
        ),
        "range_rmse_corrected_max_m": finite_float(row.get("range_rmse_corrected_max_m"), default=None),
        "position_rmse_corrected_mean_m": finite_float(
            row.get("position_rmse_corrected_mean_m")
            or row.get("position_rmse_corrected_mean_across_scenarios_m")
        ),
        "position_rmse_corrected_max_m": finite_float(row.get("position_rmse_corrected_max_m"), default=None),
        "target_max_abs_error_corrected_max_m": finite_float(
            row.get("target_max_abs_error_corrected_max_m")
            or row.get("target_max_abs_error_worst_m")
        ),
        "target_rmse_corrected_mean_m": finite_float(row.get("target_rmse_corrected_mean_m"), default=None),
        "runtime_truth_used": bool_value(row.get("runtime_truth_used")),
    }


def build_manifest(root_dir: Path, tier_specs: Sequence[Mapping[str, str]]) -> dict[str, object]:
    tiers: list[dict[str, object]] = []
    for spec in tier_specs:
        source_csv = root_dir / str(spec["source_csv"])
        row = find_method_row(source_csv, str(spec["method"]))
        adapter = adapter_config(str(spec["adapter_class"]), row)
        evidence = evidence_payload(row)
        tiers.append(
            {
                "tier": spec["tier"],
                "purpose": spec["purpose"],
                "method": spec["method"],
                "adapter": adapter,
                "runtime_condition": spec["runtime_condition"],
                "selection_scope": spec.get("selection_scope", "single_source"),
                "source_csv": str(source_csv),
                "evidence": evidence,
                "contracts": {
                    "image_contract": row.get("image_contract") or IMAGE_CONTRACT,
                    "rdf_axis_contract": row.get("rdf_axis_contract") or RDF_AXIS_CONTRACT,
                    "public_topic_contract": row.get("public_topic_contract") or PUBLIC_TOPIC_CONTRACT,
                    "runtime_truth_contract": RUNTIME_TRUTH_CONTRACT,
                },
            }
        )
    aggregate_multiburst = [
        tier
        for tier in tiers
        if tier.get("selection_scope") == "all_scenario_aggregate"
        and tier["adapter"]["class"] == "MultiBurstObservationFusion"
        and tier["evidence"]["status"] == "STRICT_PASS"
        and tier["evidence"]["runtime_truth_used"] is False
    ]
    strict_multiburst = [
        tier
        for tier in tiers
        if tier["adapter"]["class"] == "MultiBurstObservationFusion"
        and tier["evidence"]["status"] == "STRICT_PASS"
        and tier["evidence"]["runtime_truth_used"] is False
    ]
    recommended_candidates = aggregate_multiburst or strict_multiburst or [
        tier
        for tier in tiers
        if tier["evidence"]["status"] == "STRICT_PASS"
        and tier["evidence"]["runtime_truth_used"] is False
    ] or tiers
    recommended = min(
        recommended_candidates,
        key=lambda tier: (
            0 if tier["evidence"]["status"] == "STRICT_PASS" else 1,
            float(tier["evidence"]["target_max_abs_error_corrected_max_m"]),
            float(tier["evidence"]["range_rmse_corrected_mean_m"]),
            int_value(tier["adapter"].get("min_bursts", tier["adapter"].get("min_observations", 999999)))
            * int_value(tier["adapter"].get("min_observations_per_burst", 1)),
        ),
    )
    return {
        "manifest_name": "observation_fusion_runtime_manifest",
        "coordinate_contracts_explicit": True,
        "runtime_truth_used": False,
        "default_recommendation": recommended["tier"],
        "default_rationale": (
            "Prefer truth-free MultiBurst tiers when detector/crop correlation is unknown; "
            "prefer all-scenario aggregate tiers when present, then rank STRICT_PASS candidates by "
            "1852m-window target error, validation range RMSE, and observation count. Fall back to "
            "independent tiers only after observation independence is validated."
        ),
        "tiers": tiers,
    }


def load_tier_specs(path: Path | None) -> Sequence[Mapping[str, str]]:
    if path is None:
        return DEFAULT_TIERS
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, list):
        raise ValueError("tier spec JSON must contain a list")
    required = {"tier", "purpose", "source_csv", "method", "adapter_class", "runtime_condition"}
    specs: list[Mapping[str, str]] = []
    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"tier spec #{index} must be an object")
        missing = sorted(required.difference(item))
        if missing:
            raise ValueError(f"tier spec #{index} missing fields: {', '.join(missing)}")
        specs.append({key: str(value) for key, value in item.items()})
    return specs


def write_csv(path: Path, tiers: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "tier",
        "method",
        "adapter_class",
        "runtime_condition",
        "status",
        "pass_count",
        "trial_count",
        "range_rmse_corrected_mean_m",
        "position_rmse_corrected_mean_m",
        "target_max_abs_error_corrected_max_m",
        "runtime_truth_used",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for tier in tiers:
            evidence = tier["evidence"]
            adapter = tier["adapter"]
            writer.writerow(
                {
                    "tier": tier["tier"],
                    "method": tier["method"],
                    "adapter_class": adapter["class"],
                    "runtime_condition": tier["runtime_condition"],
                    "status": evidence["status"],
                    "pass_count": evidence["pass_count"],
                    "trial_count": evidence["trial_count"],
                    "range_rmse_corrected_mean_m": evidence["range_rmse_corrected_mean_m"],
                    "position_rmse_corrected_mean_m": evidence["position_rmse_corrected_mean_m"],
                    "target_max_abs_error_corrected_max_m": evidence["target_max_abs_error_corrected_max_m"],
                    "runtime_truth_used": evidence["runtime_truth_used"],
                }
            )


def write_summary(path: Path, manifest: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion runtime manifest",
        f"default_recommendation: {manifest['default_recommendation']}",
        f"runtime_truth_used: {manifest['runtime_truth_used']}",
        "",
        "Tiers:",
    ]
    for tier in manifest["tiers"]:
        evidence = tier["evidence"]
        adapter = tier["adapter"]
        lines.append(
            "- {tier}: {method} ({adapter}) status={status} pass={pass_count}/{trial_count} "
            "range_mean={range_mean:.3f}m target_worst={target_worst:.3f}m".format(
                tier=tier["tier"],
                method=tier["method"],
                adapter=adapter["class"],
                status=evidence["status"],
                pass_count=evidence["pass_count"],
                trial_count=evidence["trial_count"],
                range_mean=float(evidence["range_rmse_corrected_mean_m"]),
                target_worst=float(evidence["target_max_abs_error_corrected_max_m"]),
            )
        )
    lines.extend(
        [
            "",
            "Coordinate contract: image u/v/disparity -> stereo-center RDF right/down/forward.",
            "Public StereoDistance FRD and PX4/RTK truth are not runtime adapter inputs.",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export StereoDistance observation-fusion runtime deployment manifest.")
    parser.add_argument("--root-dir", type=Path, default=Path("."))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--tier-spec-json",
        type=Path,
        default=None,
        help="Optional JSON list of tier specs. Defaults to the built-in historical observation-fusion tiers.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest = build_manifest(args.root_dir.resolve(), load_tier_specs(args.tier_spec_json))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "observation_fusion_runtime_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_csv(args.output_dir / "observation_fusion_runtime_manifest.csv", manifest["tiers"])
    write_summary(args.output_dir / "observation_fusion_runtime_manifest_summary.txt", manifest)
    print(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
