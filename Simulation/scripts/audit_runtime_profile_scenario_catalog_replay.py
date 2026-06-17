#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import audit_runtime_profile_fresh_seed_replay as fresh  # noqa: E402
import rtk_stereo_calibration_pipeline as pipe  # noqa: E402


def load_catalog(path: Path) -> dict[str, Path]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, Mapping) and isinstance(payload.get("profiles"), Mapping):
        payload = payload["profiles"]
    if not isinstance(payload, Mapping):
        raise ValueError("catalog JSON must be an object or {'profiles': object}")
    catalog = {str(scenario): Path(str(profile_path)) for scenario, profile_path in payload.items()}
    unknown = [scenario for scenario in catalog if scenario not in fresh.SCENARIO_PRESETS]
    if unknown:
        raise ValueError(f"unknown scenarios in catalog: {', '.join(unknown)}")
    return catalog


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a scenario-labeled runtime profile catalog on fresh synthetic seeds.")
    parser.add_argument("--profile-catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--seed-start", type=int, default=20296859)
    parser.add_argument("--trial-count", type=int, default=50)
    parser.add_argument(
        "--scenarios",
        type=fresh.parse_scenarios,
        default=list(fresh.DEFAULT_SCENARIOS),
    )
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    catalog = load_catalog(args.profile_catalog_json)
    missing = [scenario for scenario in args.scenarios if scenario not in catalog]
    if missing:
        raise ValueError(f"catalog missing scenarios: {', '.join(missing)}")
    profiles = {scenario: fresh.load_profile(catalog[scenario]) for scenario in args.scenarios}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    seed_rows: list[dict[str, object]] = []
    prediction_rows: list[dict[str, object]] = []
    for scenario_index, scenario in enumerate(args.scenarios):
        profile = profiles[scenario]
        for offset in range(int(args.trial_count)):
            seed = int(args.seed_start) + scenario_index * int(args.trial_count) + offset
            row, predictions = fresh.evaluate_seed(
                profile,
                seed=seed,
                scenario=scenario,
                sample_count=int(args.sample_count),
                train_ratio=float(args.train_ratio),
                target_range_m=float(args.target_range_m),
                target_window_m=float(args.target_window_m),
                target_oversample_count=int(args.target_oversample_count),
            )
            row["selected_profile_id"] = profile.profile_id
            row["selected_profile_path"] = str(catalog[scenario])
            seed_rows.append(row)
            if offset < 2:
                for prediction in predictions:
                    prediction["selected_profile_id"] = profile.profile_id
                    prediction_rows.append(prediction)
    scenario_rows = fresh.aggregate_by_scenario(seed_rows)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_catalog_json": str(args.profile_catalog_json),
        "seed_start": args.seed_start,
        "seed_end": args.seed_start + len(args.scenarios) * args.trial_count - 1,
        "scenarios": args.scenarios,
        "coordinate_contracts_explicit": True,
        "rdf_axis_contract": fresh.RDF_AXIS_CONTRACT,
        "public_topic_contract": fresh.PUBLIC_TOPIC_CONTRACT,
        "ned_to_rdf_contract": fresh.NED_TO_RDF_CONTRACT,
        "selection_contract": "scenario label selects one fixed runtime profile; no truth/replay metrics are runtime inputs",
        **fresh.aggregate(seed_rows),
    }
    fresh.write_csv(args.output_dir / "scenario_catalog_replay_summary.csv", seed_rows)
    fresh.write_csv(args.output_dir / "scenario_catalog_replay_scenario_summary.csv", scenario_rows)
    fresh.write_csv(args.output_dir / "scenario_catalog_replay_predictions_sample.csv", prediction_rows)
    fresh.write_json(args.output_dir / "scenario_catalog_replay_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: scenario-labeled replay of a runtime profile catalog on independent synthetic seeds.",
                "",
                "Coordinate contract: each profile consumes stereo-center RDF right/down/forward and measured disparity. Scenario labels select profiles, while PX4/local NED truth is converted to RDF only for offline evaluation.",
                "",
                f"Scenarios: {', '.join(args.scenarios)}",
                "",
                f"Result: {summary['status']}. Pass {summary['pass_count']}/{summary['trial_count']}, range RMSE mean raw -> corrected {summary['range_rmse_raw_mean_m']:.9f} -> {summary['range_rmse_corrected_mean_m']:.9f} m, target max abs corrected max {summary['target_max_abs_error_corrected_max_m']:.9f} m.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
