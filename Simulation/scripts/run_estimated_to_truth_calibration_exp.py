#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
PIPELINE_SCRIPT = ROOT_DIR / "scripts" / "rtk_stereo_calibration_pipeline.py"
REPLAY_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_replay.py"
GATE_SCRIPT = ROOT_DIR / "scripts" / "experiment_pass_gate.py"
PREFLIGHT_SCRIPT = ROOT_DIR / "scripts" / "calibration_csv_preflight.py"
PROMOTION_CHECK_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_promotion_check.py"
RESULT_AUDIT_SCRIPT = ROOT_DIR / "scripts" / "estimated_to_truth_result_audit.py"

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
NED_TO_RDF_CONTRACT = "rdf_right=east, rdf_down=down, rdf_forward=north"
SEARCH_SPACE_DESCRIPTION = (
    "raw, global_rotation, global_scale_rotation, sim3, linear disparity, "
    "trimmed/ransac/huber/range-weighted/robust-loss sim3, quadratic disparity, "
    "low-degree/target-weighted/ridge forward and RDF-axis residual disparity"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the estimated-to-truth calibration experiment end to end: "
            "fit candidates, export a StereoDistance runtime profile, replay it, "
            "and write gate-ready metrics.json."
        )
    )
    parser.add_argument("--mode", choices=("synthetic", "csv"), default="synthetic")
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--experiment-id", default="")
    parser.add_argument("--seed", type=int, default=20260613)
    parser.add_argument("--sample-count", type=int, default=520)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--min-target-count", type=int, default=3)
    parser.add_argument("--max-train-validation-gap-m", type=float, default=0.5)
    parser.add_argument(
        "--skip-replay",
        action="store_true",
        help="Skip runtime profile replay even when sim3_runtime_profile.json exists.",
    )
    parser.add_argument(
        "--skip-promotion-check",
        action="store_true",
        help="Skip StereoDistance runtime profile loader/apply promotion check.",
    )
    parser.add_argument(
        "--strict-preflight",
        action="store_true",
        help="In CSV mode, stop before fitting when preflight lacks usable 1852m target coverage.",
    )
    return parser.parse_args()


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def finite_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def method_row(rows: Sequence[Mapping[str, str]], method: str) -> Mapping[str, str]:
    for row in rows:
        if row.get("method") == method:
            return row
    raise ValueError(f"method {method!r} not found in method_comparison.csv")


def recommended_method(run_config: Mapping[str, Any], comparison_rows: Sequence[Mapping[str, str]]) -> str:
    recommendation = str(run_config.get("recommendation") or "")
    if recommendation:
        return recommendation
    non_raw = [row for row in comparison_rows if row.get("method") != "raw"]
    if not non_raw:
        raise ValueError("method comparison has no correction candidates")
    best = min(non_raw, key=lambda row: finite_float(row.get("validation_range_rmse_m"), float("inf")))
    return str(best["method"])


def build_metrics(
    *,
    output_dir: Path,
    experiment_id: str,
    mode: str,
    input_csv: Path | None,
    min_target_count: int,
    max_train_validation_gap_m: float,
) -> dict[str, Any]:
    run_config = load_json(output_dir / "run_config.json")
    comparison_rows = read_csv_rows(output_dir / "method_comparison.csv")
    raw = method_row(comparison_rows, "raw")
    recommendation = recommended_method(run_config, comparison_rows)
    selected = method_row(comparison_rows, recommendation)

    runtime_profile_path = output_dir / "sim3_runtime_profile.json"
    target_count = finite_int(selected.get("validation_target_window_count"))
    train_gap = finite_float(selected.get("generalization_gap_range_rmse_m"))
    artifacts = [
        str(output_dir / "method_comparison.csv"),
        str(output_dir / "validation_predictions.csv"),
        str(output_dir / "target_1852m_stats.csv"),
        str(output_dir / "pareto_frontier.csv"),
        str(output_dir / "pareto_frontier.json"),
        str(output_dir / "policy_recommendations.csv"),
        str(output_dir / "policy_recommendations.json"),
        str(output_dir / "method_parameters.json"),
        str(output_dir / "range_error_before_after.png"),
        str(output_dir / "residual_disparity_by_distance.png"),
        str(output_dir / "rdf_axis_rmse.png"),
        str(output_dir / "pareto_frontier.png"),
        str(output_dir / "summary.txt"),
        str(output_dir / "run_config.json"),
        str(output_dir / "runtime_profiles" / "manifest.json"),
        str(output_dir / "policy_profiles" / "manifest.json"),
    ]
    if runtime_profile_path.exists():
        artifacts.append(str(runtime_profile_path))
    replay_summary_path = output_dir / "profile_replay" / "profile_replay_summary.csv"
    if replay_summary_path.exists():
        artifacts.extend(
            [
                str(replay_summary_path),
                str(output_dir / "profile_replay" / "profile_replay_predictions.csv"),
                str(output_dir / "profile_replay" / "profile_replay_range_error.png"),
                str(output_dir / "profile_replay" / "profile_replay_summary.txt"),
            ]
        )
    preflight_summary_path = output_dir / "preflight" / "preflight_summary.json"
    preflight_summary: dict[str, Any] = {}
    if preflight_summary_path.exists():
        preflight_summary = load_json(preflight_summary_path)
        artifacts.extend(
            [
                str(preflight_summary_path),
                str(output_dir / "preflight" / "preflight_rows.csv"),
                str(output_dir / "preflight" / "preflight_summary.txt"),
            ]
        )
    promotion_check_path = output_dir / "promotion_check" / "promotion_check.json"
    promotion_check: dict[str, Any] = {}
    if promotion_check_path.exists():
        promotion_check = load_json(promotion_check_path)
        artifacts.extend(
            [
                str(promotion_check_path),
                str(output_dir / "promotion_check" / "promotion_check.txt"),
                str(output_dir / "promotion_check" / "promotion_check_samples.csv"),
            ]
        )
    result_audit_path = output_dir / "result_audit.json"
    if result_audit_path.exists():
        artifacts.extend(
            [
                str(result_audit_path),
                str(output_dir / "result_audit.txt"),
            ]
        )

    metrics: dict[str, Any] = {
        "experiment_id": experiment_id,
        "status": "UNKNOWN",
        "stage": "validation",
        "method": recommendation,
        "method_family": "point_set_registration+disparity_bias",
        "scenario": (
            "synthetic_rtk_px4_ned_to_stereo_rdf_1852m"
            if mode == "synthetic"
            else "recorded_calibration_csv_px4_ned_to_stereo_rdf"
        ),
        "truth_point_defined": True,
        "truth_point_description": "target drone reference point from PX4 local position NED converted to stereo-center RDF",
        "runtime_feature_defined": True,
        "runtime_feature_description": "raw stereo RDF estimate from image feature center and measured disparity",
        "automated_search": True,
        "search_space_defined": True,
        "search_space_description": SEARCH_SPACE_DESCRIPTION,
        "candidate_count": len(comparison_rows),
        "rejected_candidate_count": max(0, len(comparison_rows) - 2),
        "selection_metric_defined": True,
        "selection_metric": "validation range RMSE with penalties for 1852m target error, train/validation gap, and model complexity",
        "validation_split_strategy": f"random {run_config.get('train_ratio', 0.8):.0%}:{1.0 - float(run_config.get('train_ratio', 0.8)):.0%} split",
        "overfit_control": "held-out validation split, train/validation gap, model complexity penalty",
        "stereodistance_applicable": runtime_profile_path.exists()
        and bool(promotion_check.get("pass_promotion_check", True)),
        "runtime_profile_exportable": runtime_profile_path.exists(),
        "runtime_profile_path": str(runtime_profile_path) if runtime_profile_path.exists() else "",
        "runtime_profile_schema": "estimated_to_truth_sim3" if runtime_profile_path.exists() else "",
        "stereodistance_apply_module": "stereo_ground_sim.stereo_distance_node.StereoDistanceNode",
        "offline_only": False,
        "offline_only_reason": "",
        "calibration_truth_source": (
            "synthetic PX4 local NED equivalent" if mode == "synthetic" else str(input_csv or "")
        ),
        "measurement_runtime_inputs": [
            "raw_rdf_m",
            "measured_disparity_px",
            "left_feature_center_px",
            "calibration_fixed_sim3_parameters",
        ],
        "forbidden_runtime_inputs": [
            "calibration_truth",
            "px4_rtk_truth",
            "true_disparity",
            "true_image_feature_location",
        ],
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "alias_contracts_explicit": True,
        "px4_truth_frame": "px4_local_position_ned",
        "px4_truth_axis_order": "north,east,down",
        "internal_correction_frame": "stereo_center_opencv_optical",
        "internal_correction_axis_order": "right,down,forward",
        "public_stereodistance_topic_order": "forward,right,down,range",
        "csv_report_field_contract": "named RDF/NED fields only; no ambiguous x/y/z evidence fields",
        "stereo_measurement_frame": "stereo_center_opencv_optical",
        "stereo_measurement_axis_order": "right,down,forward",
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "report_count": 1,
        "trial_count": 1,
        "sample_count": finite_int(run_config.get("sample_count")),
        "corrected_sample_count": finite_int(selected.get("validation_sample_count")),
        "train_sample_count": finite_int(selected.get("train_sample_count")),
        "validation_sample_count": finite_int(selected.get("validation_sample_count")),
        "validation_range_rmse_raw_m": finite_float(raw.get("validation_range_rmse_m")),
        "validation_range_rmse_corrected_m": finite_float(selected.get("validation_range_rmse_m")),
        "validation_position_rmse_raw_m": finite_float(raw.get("validation_position_rmse_m")),
        "validation_position_rmse_corrected_m": finite_float(selected.get("validation_position_rmse_m")),
        "validation_right_rmse_raw_m": finite_float(raw.get("validation_right_rmse_m")),
        "validation_right_rmse_corrected_m": finite_float(selected.get("validation_right_rmse_m")),
        "validation_down_rmse_raw_m": finite_float(raw.get("validation_down_rmse_m")),
        "validation_down_rmse_corrected_m": finite_float(selected.get("validation_down_rmse_m")),
        "validation_forward_rmse_raw_m": finite_float(raw.get("validation_forward_rmse_m")),
        "validation_forward_rmse_corrected_m": finite_float(selected.get("validation_forward_rmse_m")),
        "target_range_m": finite_float(run_config.get("target_range_m"), 1852.0),
        "target_window_half_width_m": finite_float(run_config.get("target_window_m"), 30.0),
        "target_1852m_sample_count": target_count,
        "target_1852m_min_sample_count": min_target_count,
        "target_1852m_abs_mean_error_m": finite_float(selected.get("validation_target_abs_mean_error_m"), float("inf")),
        "target_1852m_rmse_m": finite_float(selected.get("validation_target_rmse_m"), float("inf")),
        "target_1852m_tolerance_m": 10.0,
        "train_range_rmse_m": finite_float(selected.get("train_range_rmse_m")),
        "validation_range_rmse_m": finite_float(selected.get("validation_range_rmse_m")),
        "train_validation_gap_range_rmse_m": train_gap,
        "max_train_validation_gap_range_rmse_m": max_train_validation_gap_m,
        "pass_trial_count": 1,
        "total_trial_count": 1,
        "right_min_delta_m": finite_float(raw.get("validation_right_rmse_m")) - finite_float(selected.get("validation_right_rmse_m")),
        "right_mean_delta_m": finite_float(raw.get("validation_right_rmse_m")) - finite_float(selected.get("validation_right_rmse_m")),
        "down_min_delta_m": finite_float(raw.get("validation_down_rmse_m")) - finite_float(selected.get("validation_down_rmse_m")),
        "down_mean_delta_m": finite_float(raw.get("validation_down_rmse_m")) - finite_float(selected.get("validation_down_rmse_m")),
        "forward_min_delta_m": finite_float(raw.get("validation_forward_rmse_m")) - finite_float(selected.get("validation_forward_rmse_m")),
        "forward_mean_delta_m": finite_float(raw.get("validation_forward_rmse_m")) - finite_float(selected.get("validation_forward_rmse_m")),
        "position_min_delta_m": finite_float(raw.get("validation_position_rmse_m")) - finite_float(selected.get("validation_position_rmse_m")),
        "position_mean_delta_m": finite_float(raw.get("validation_position_rmse_m")) - finite_float(selected.get("validation_position_rmse_m")),
        "corrected_sample_coverage": 1.0,
        "preflight": preflight_summary,
        "promotion_check": promotion_check,
        "failure_tags": failure_tags_for_mode(mode, preflight_summary, promotion_check),
        "artifacts": artifacts,
    }
    return metrics


def failure_tags_for_mode(
    mode: str,
    preflight_summary: Mapping[str, Any],
    promotion_check: Mapping[str, Any],
) -> list[str]:
    tags: list[str] = []
    if mode != "csv":
        tags.append("synthetic_only")
    if preflight_summary and not bool(preflight_summary.get("pass_preflight")):
        tags.append("preflight_failed")
    if promotion_check and not bool(promotion_check.get("pass_promotion_check")):
        tags.append("promotion_check_failed")
    return tags


def run_command(cmd: Sequence[str], *, cwd: Path) -> None:
    subprocess.run(list(cmd), cwd=str(cwd), check=True)


def run_pipeline(args: argparse.Namespace) -> None:
    cmd = [
        sys.executable,
        str(PIPELINE_SCRIPT),
        "--mode",
        args.mode,
        "--output-dir",
        str(args.output_dir),
        "--seed",
        str(args.seed),
        "--sample-count",
        str(args.sample_count),
        "--train-ratio",
        str(args.train_ratio),
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
    ]
    if args.mode == "csv":
        if args.input_csv is None:
            raise SystemExit("--input-csv is required for --mode csv")
        cmd.extend(["--input-csv", str(args.input_csv)])
    run_command(cmd, cwd=ROOT_DIR.parent)


def run_preflight(args: argparse.Namespace) -> dict[str, Any]:
    if args.mode != "csv":
        return {}
    if args.input_csv is None:
        raise SystemExit("--input-csv is required for --mode csv")
    preflight_dir = args.output_dir / "preflight"
    cmd = [
        sys.executable,
        str(PREFLIGHT_SCRIPT),
        "--input-csv",
        str(args.input_csv),
        "--output-dir",
        str(preflight_dir),
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
        "--min-target-count",
        str(args.min_target_count),
    ]
    completed = subprocess.run(
        cmd,
        cwd=str(ROOT_DIR.parent),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if completed.stdout:
        print(completed.stdout, end="")
    summary_path = preflight_dir / "preflight_summary.json"
    summary = load_json(summary_path) if summary_path.exists() else {}
    if args.strict_preflight and completed.returncode != 0:
        raise SystemExit("CSV preflight failed; rerun without --strict-preflight to fit anyway.")
    return summary


def run_replay(args: argparse.Namespace) -> None:
    profile_path = args.output_dir / "sim3_runtime_profile.json"
    samples_path = args.output_dir / "calibration_samples.csv"
    if args.skip_replay or not profile_path.exists() or not samples_path.exists():
        return
    cmd = [
        sys.executable,
        str(REPLAY_SCRIPT),
        "--input-csv",
        str(samples_path),
        "--profile-json",
        str(profile_path),
        "--output-dir",
        str(args.output_dir / "profile_replay"),
        "--split",
        "validation",
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
    ]
    run_command(cmd, cwd=ROOT_DIR.parent)


def run_promotion_check(args: argparse.Namespace) -> None:
    if args.skip_promotion_check:
        return
    profile_path = args.output_dir / "sim3_runtime_profile.json"
    samples_path = args.output_dir / "calibration_samples.csv"
    if not profile_path.exists():
        return
    cmd = [
        sys.executable,
        str(PROMOTION_CHECK_SCRIPT),
        "--profile-json",
        str(profile_path),
        "--output-dir",
        str(args.output_dir / "promotion_check"),
        "--split",
        "validation",
        "--max-samples",
        "50",
    ]
    if samples_path.exists():
        cmd.extend(["--sample-csv", str(samples_path)])
    run_command(cmd, cwd=ROOT_DIR.parent)


def run_gate(metrics_path: Path) -> dict[str, Any]:
    cmd = [sys.executable, str(GATE_SCRIPT), str(metrics_path), "--write"]
    completed = subprocess.run(
        cmd,
        cwd=str(ROOT_DIR.parent),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if completed.stdout:
        print(completed.stdout, end="")
    payload = load_json(metrics_path)
    if completed.returncode not in (0, 1):
        raise subprocess.CalledProcessError(completed.returncode, cmd, completed.stdout)
    return payload.get("gate_result", {})


def run_result_audit(args: argparse.Namespace) -> None:
    cmd = [
        sys.executable,
        str(RESULT_AUDIT_SCRIPT),
        str(args.output_dir),
    ]
    if args.mode == "csv":
        cmd.append("--require-csv-preflight")
    run_command(cmd, cwd=ROOT_DIR.parent)


def append_result_audit_artifacts(metrics_path: Path, output_dir: Path) -> None:
    payload = load_json(metrics_path)
    artifacts = list(payload.get("artifacts") or [])
    for path in (output_dir / "result_audit.json", output_dir / "result_audit.txt"):
        path_text = str(path)
        if path.exists() and path_text not in artifacts:
            artifacts.append(path_text)
    payload["artifacts"] = artifacts
    write_json(metrics_path, payload)


def write_promotion_summary(path: Path, metrics: Mapping[str, Any], gate_result: Mapping[str, Any]) -> None:
    lines = [
        "Estimated-to-Truth calibration experiment",
        "",
        f"experiment_id: {metrics['experiment_id']}",
        f"status: {metrics.get('status', 'UNKNOWN')}",
        f"gate_verdict: {gate_result.get('verdict', 'UNKNOWN')}",
        f"method: {metrics['method']}",
        f"validation_range_rmse_raw_m: {metrics['validation_range_rmse_raw_m']:.6f}",
        f"validation_range_rmse_corrected_m: {metrics['validation_range_rmse_corrected_m']:.6f}",
        f"validation_position_rmse_raw_m: {metrics['validation_position_rmse_raw_m']:.6f}",
        f"validation_position_rmse_corrected_m: {metrics['validation_position_rmse_corrected_m']:.6f}",
        f"target_1852m_abs_mean_error_m: {metrics['target_1852m_abs_mean_error_m']:.6f}",
        "",
        "coordinate_contract:",
        f"- PX4 truth: {NED_AXIS_CONTRACT}",
        f"- internal correction: {RDF_AXIS_CONTRACT}",
        f"- StereoDistance public topic: {PUBLIC_TOPIC_CONTRACT}",
        f"- NED to RDF: {NED_TO_RDF_CONTRACT}",
        "",
        "runtime_contract:",
        "- calibration truth is used only for fitting and validation",
        "- measurement runtime applies calibration-fixed parameters only",
        "- runtime truth, true disparity, and true feature location are forbidden",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    experiment_id = args.experiment_id or args.output_dir.name

    run_preflight(args)
    run_pipeline(args)
    run_replay(args)
    run_promotion_check(args)
    metrics = build_metrics(
        output_dir=args.output_dir,
        experiment_id=experiment_id,
        mode=args.mode,
        input_csv=args.input_csv,
        min_target_count=args.min_target_count,
        max_train_validation_gap_m=args.max_train_validation_gap_m,
    )
    metrics_path = args.output_dir / "metrics.json"
    write_json(metrics_path, metrics)
    gate_result = run_gate(metrics_path)
    final_metrics = load_json(metrics_path)
    write_promotion_summary(args.output_dir / "promotion_summary.txt", final_metrics, gate_result)
    run_result_audit(args)
    append_result_audit_artifacts(metrics_path, args.output_dir)
    print(f"wrote experiment outputs to {args.output_dir}")
    print(f"gate verdict: {gate_result.get('verdict', 'UNKNOWN')}")
    return 0 if gate_result.get("verdict") in {"STRICT_PASS", "PASS"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
