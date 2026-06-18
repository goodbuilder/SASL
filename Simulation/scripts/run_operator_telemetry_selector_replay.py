#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_disparity_residual_selector_on_observation_windows as selector_replay  # noqa: E402
import export_selector_observation_csv_from_operator_telemetry as telemetry_export  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert operator telemetry.csv into a truth-free selector observation CSV, then run "
            "rolling-window residual selector replay in one command."
        )
    )
    parser.add_argument("--input-telemetry-csv", type=Path, required=True)
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-size", type=int, default=21)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--min-samples", type=int, default=1)
    parser.add_argument("--expected-label", default="")
    parser.add_argument("--feature-label", default="operator_telemetry_window")
    parser.add_argument("--allow-missing-corrected", action="store_true")
    parser.add_argument("--allow-missing-observation-rdf", action="store_true")
    parser.add_argument("--require-stable-label", action="store_true")
    parser.add_argument("--min-distance-margin", type=float, default=0.0)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    export_summary = payload.get("export_summary", {})
    replay_summary = payload.get("replay_summary", {})
    lines = [
        "Operator telemetry selector replay",
        f"status: {payload.get('status', '')}",
        f"input_telemetry_csv: {payload.get('input_telemetry_csv', '')}",
        f"selector_observation_csv: {payload.get('selector_observation_csv', '')}",
        f"selector_model_json: {payload.get('selector_model_json', '')}",
        f"catalog_json: {payload.get('catalog_json', '')}",
        f"window_size: {payload.get('window_size', 0)}",
        f"stride: {payload.get('stride', 0)}",
        f"min_samples: {payload.get('min_samples', 0)}",
        f"expected_label: {payload.get('expected_label', '')}",
        f"require_stable_label: {payload.get('require_stable_label', False)}",
        f"min_distance_margin: {payload.get('min_distance_margin', 0.0)}",
        f"export_status: {export_summary.get('status', '') if isinstance(export_summary, dict) else ''}",
        f"replay_status: {replay_summary.get('status', '') if isinstance(replay_summary, dict) else ''}",
        f"dropped_truth_column_count: {payload.get('dropped_truth_column_count', 0)}",
        f"window_count: {payload.get('window_count', 0)}",
        f"selected_labels: {','.join(str(v) for v in payload.get('selected_labels', []))}",
        f"selected_label_counts: {json.dumps(payload.get('selected_label_counts', {}), ensure_ascii=False, sort_keys=True)}",
        f"dominant_selected_label: {payload.get('dominant_selected_label', '')}",
        f"dominant_selected_label_count: {payload.get('dominant_selected_label_count', 0)}",
        f"selector_label_stable: {payload.get('selector_label_stable', False)}",
        f"label_match_count: {payload.get('label_match_count', 0)}",
        f"profile_exists_count: {payload.get('profile_exists_count', 0)}",
        f"apply_residual_allowed: {payload.get('apply_residual_allowed', False)}",
        f"recommended_residual_label: {payload.get('recommended_residual_label', '')}",
        f"recommended_residual_profile: {payload.get('recommended_residual_profile', '')}",
        f"distance_margin_min: {payload.get('distance_margin_min', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def dominant_profile_for_label(replay_rows: Sequence[Mapping[str, object]], label: str) -> str:
    profiles = [
        str(row.get("selected_profile", ""))
        for row in replay_rows
        if str(row.get("selected_label", "")) == label and str(row.get("selected_profile", ""))
    ]
    if not profiles:
        return ""
    return max(Counter(profiles).items(), key=lambda item: (item[1], item[0]))[0]


def write_decision_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator telemetry selector decision",
        f"status: {payload.get('status', '')}",
        f"apply_residual_allowed: {payload.get('apply_residual_allowed', False)}",
        f"recommended_residual_label: {payload.get('recommended_residual_label', '')}",
        f"recommended_residual_profile: {payload.get('recommended_residual_profile', '')}",
        f"selector_label_stable: {payload.get('selector_label_stable', False)}",
        f"distance_margin_min: {payload.get('distance_margin_min', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"decision_contract: {payload.get('decision_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    selector_observation_csv = args.output_dir / "selector_observations.csv"
    replay_dir = args.output_dir / "window_selector_replay"

    export_summary = telemetry_export.export_observation_csv(
        input_csv=args.input_telemetry_csv,
        output_csv=selector_observation_csv,
        allow_missing_corrected=args.allow_missing_corrected,
    )
    telemetry_export.write_json(args.output_dir / "operator_telemetry_to_selector_observation_summary.json", export_summary)
    telemetry_export.write_text(args.output_dir / "operator_telemetry_to_selector_observation_summary.txt", export_summary)

    replay_summary: dict[str, object] = {
        "status": "SKIPPED",
        "failures": ["export_failed"],
        "runtime_truth_used": False,
    }
    feature_rows: list[Mapping[str, object]] = []
    replay_rows: list[Mapping[str, object]] = []
    if export_summary.get("status") == "PASS":
        replay_payload = selector_replay.selector_windows(
            input_csv=selector_observation_csv,
            selector_model_json=args.selector_model_json,
            catalog_json=args.catalog_json,
            window_size=args.window_size,
            stride=args.stride,
            min_samples=args.min_samples,
            feature_label=args.feature_label,
            expected_label=args.expected_label,
            allow_missing_observation_rdf=args.allow_missing_observation_rdf,
            require_stable_label=args.require_stable_label,
            min_distance_margin=args.min_distance_margin,
        )
        feature_rows = replay_payload.pop("feature_rows")
        replay_rows = replay_payload.pop("replay_rows")
        replay_summary = replay_payload
        replay_dir.mkdir(parents=True, exist_ok=True)
        selector_replay.write_json(replay_dir / "observation_window_selector_replay_summary.json", replay_summary)
        selector_replay.write_csv(replay_dir / "observation_window_selector_features.csv", feature_rows)
        selector_replay.write_csv(replay_dir / "observation_window_selector_replay.csv", replay_rows)
        selector_replay.write_text(replay_dir / "observation_window_selector_replay_summary.txt", replay_summary)

    failures: list[str] = []
    if export_summary.get("status") != "PASS":
        failures.extend(f"export:{failure}" for failure in export_summary.get("failures", []))
    if replay_summary.get("status") != "PASS":
        failures.extend(f"replay:{failure}" for failure in replay_summary.get("failures", []))
    runtime_truth_used = bool(export_summary.get("runtime_truth_used", False)) or bool(
        replay_summary.get("runtime_truth_used", False)
    )
    if runtime_truth_used:
        failures.append("runtime_truth_used")
    status = "PASS" if not failures else "FAIL"
    recommended_label = str(replay_summary.get("dominant_selected_label", "")) if status == "PASS" else ""
    recommended_profile = dominant_profile_for_label(replay_rows, recommended_label) if recommended_label else ""
    apply_residual_allowed = bool(status == "PASS" and recommended_label and recommended_profile)

    payload = {
        "mode": "operator_telemetry_truth_drop_selector_replay",
        "status": status,
        "input_telemetry_csv": str(args.input_telemetry_csv),
        "selector_observation_csv": str(selector_observation_csv),
        "selector_model_json": str(args.selector_model_json),
        "catalog_json": str(args.catalog_json),
        "window_size": max(1, int(args.window_size)),
        "stride": max(1, int(args.stride)),
        "min_samples": max(1, int(args.min_samples)),
        "expected_label": args.expected_label,
        "require_stable_label": bool(args.require_stable_label),
        "min_distance_margin": max(0.0, float(args.min_distance_margin)),
        "dropped_truth_column_count": export_summary.get("dropped_truth_column_count", 0),
        "window_count": replay_summary.get("window_count", 0),
        "selected_labels": replay_summary.get("selected_labels", []),
        "selected_label_counts": replay_summary.get("selected_label_counts", {}),
        "dominant_selected_label": replay_summary.get("dominant_selected_label", ""),
        "dominant_selected_label_count": replay_summary.get("dominant_selected_label_count", 0),
        "selector_label_stable": replay_summary.get("selector_label_stable", False),
        "label_match_count": replay_summary.get("label_match_count", 0),
        "profile_exists_count": replay_summary.get("profile_exists_count", 0),
        "apply_residual_allowed": apply_residual_allowed,
        "recommended_residual_label": recommended_label,
        "recommended_residual_profile": recommended_profile,
        "distance_margin_min": replay_summary.get("distance_margin_min", 0.0),
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contract": (
            "operator telemetry input is public FRD(forward,right,down,range); the generated selector "
            "observation CSV is explicit stereo-center RDF(right,down,forward); truth/PX4/RTK/NED fields "
            "are dropped before replay; public StereoDistance output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "export_summary": export_summary,
        "replay_summary": replay_summary,
    }
    decision_payload = {
        "mode": "operator_telemetry_selector_decision",
        "status": status,
        "apply_residual_allowed": apply_residual_allowed,
        "recommended_residual_label": recommended_label,
        "recommended_residual_profile": recommended_profile,
        "selector_label_stable": replay_summary.get("selector_label_stable", False),
        "selected_label_counts": replay_summary.get("selected_label_counts", {}),
        "distance_margin_min": replay_summary.get("distance_margin_min", 0.0),
        "require_stable_label": bool(args.require_stable_label),
        "min_distance_margin": max(0.0, float(args.min_distance_margin)),
        "runtime_truth_used": runtime_truth_used,
        "decision_contract": (
            "apply_residual_allowed is true only when the truth-free telemetry export and selector replay gates "
            "pass; downstream operator launch may use recommended_residual_profile only in that case"
        ),
        "coordinate_contract": payload["coordinate_contract"],
        "failure_count": len(failures),
        "failures": failures,
    }
    write_json(args.output_dir / "operator_telemetry_selector_replay_summary.json", payload)
    write_text(args.output_dir / "operator_telemetry_selector_replay_summary.txt", payload)
    write_json(args.output_dir / "operator_telemetry_selector_decision.json", decision_payload)
    write_decision_text(args.output_dir / "operator_telemetry_selector_decision.txt", decision_payload)
    payload["feature_rows"] = feature_rows
    payload["replay_rows"] = replay_rows
    payload["decision"] = decision_payload
    return payload


def main() -> None:
    payload = run()
    printable = dict(payload)
    printable.pop("feature_rows", None)
    printable.pop("replay_rows", None)
    printable.pop("decision", None)
    print(json.dumps(printable, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
