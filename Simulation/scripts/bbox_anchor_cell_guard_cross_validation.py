#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Sequence


RDF_AXES = ("right", "down", "forward")
AXES = ("right", "down", "forward", "position")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Leave-one-report-out validation for runtime-safe bbox-anchor "
            "cell/range Down guards. Right and Forward use fixed conservative "
            "axis guards; Down is enabled only for feature cells that improve "
            "on the training reports."
        )
    )
    parser.add_argument("report_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--holdout-mode",
        choices=("report", "report_trial"),
        default="report",
        help=(
            "report keeps the existing leave-one-report-out split. "
            "report_trial evaluates one report/trial pair at a time and trains "
            "only on rows from different reports and different trial numbers."
        ),
    )
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--range-edges-m", default="0,500,800,1100,1400,2000")
    parser.add_argument(
        "--feature-sets",
        default="anchor_cell,range_bin,anchor_cell_range_bin",
        help="Comma-separated subset of anchor_cell, range_bin, anchor_cell_range_bin.",
    )
    parser.add_argument("--right-gain", type=float, default=0.1)
    parser.add_argument("--right-min-range-m", type=float, default=1100.0)
    parser.add_argument("--forward-gain", type=float, default=1.0)
    parser.add_argument("--forward-min-range-m", type=float, default=500.0)
    parser.add_argument("--down-gains", default="0,0.005,0.01,0.025,0.05,0.075,0.1,0.15,0.2")
    parser.add_argument("--train-min-samples", default="20,50,100,200,400")
    parser.add_argument("--train-min-down-improvements-m", default="0,0.0005,0.001,0.002,0.005")
    return parser.parse_args()


def parse_int_list(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_float_list(text: str) -> list[float]:
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_str_list(text: str) -> list[str]:
    return [part.strip() for part in str(text).split(",") if part.strip()]


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def range_bin(raw_forward_m: float, edges: Sequence[float]) -> int:
    for index in range(max(0, len(edges) - 1)):
        if float(edges[index]) <= raw_forward_m < float(edges[index + 1]):
            return index
    return max(0, len(edges) - 2)


def load_samples(report_dirs: Sequence[Path], trials: Sequence[int], range_edges: Sequence[float]) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for report_dir in report_dirs:
        for trial in trials:
            path = report_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
            if not path.exists():
                continue
            with path.open(newline="", encoding="utf-8") as handle:
                for row_index, row in enumerate(csv.DictReader(handle)):
                    raw_forward = float(row["raw_forward_m"])
                    sample = {
                        "report_dir": report_dir.name,
                        "trial": trial,
                        "row_index": row_index,
                        "raw_forward_m": raw_forward,
                        "range_bin": range_bin(raw_forward, range_edges),
                        "anchor_cell": row.get("anchor_cell", ""),
                    }
                    for axis in RDF_AXES:
                        sample[f"raw_error_{axis}"] = float(row[f"raw_error_{axis}_m"])
                        sample[f"anchor_error_{axis}"] = float(row[f"anchor_error_{axis}_m"])
                    samples.append(sample)
    return samples


def feature_key(sample: dict[str, object], feature_set: str) -> tuple[str, ...]:
    if feature_set == "anchor_cell":
        return (str(sample["anchor_cell"]),)
    if feature_set == "range_bin":
        return (str(sample["range_bin"]),)
    if feature_set == "anchor_cell_range_bin":
        return (str(sample["anchor_cell"]), str(sample["range_bin"]))
    raise ValueError(f"unsupported feature set: {feature_set}")


def blended_error(raw_error: float, anchor_error: float, gain: float) -> float:
    return raw_error + gain * (anchor_error - raw_error)


def axis_guarded_error(
    *,
    raw_error: float,
    anchor_error: float,
    raw_forward_m: float,
    gain: float,
    min_range_m: float,
) -> tuple[float, str]:
    if raw_forward_m < min_range_m:
        return raw_error, "range_guarded_noop"
    if gain == 0.0:
        return raw_error, "gain_noop"
    return blended_error(raw_error, anchor_error, gain), "none"


def down_improvement(rows: Sequence[dict[str, object]], down_gain: float) -> float:
    raw = [float(row["raw_error_down"]) for row in rows]
    corrected = [
        blended_error(float(row["raw_error_down"]), float(row["anchor_error_down"]), down_gain)
        for row in rows
    ]
    return rmse(raw) - rmse(corrected)


def build_down_policy(
    train_rows: Sequence[dict[str, object]],
    *,
    feature_set: str,
    down_gain: float,
    min_samples: int,
    min_down_improvement_m: float,
) -> tuple[set[tuple[str, ...]], list[dict[str, object]]]:
    grouped: dict[tuple[str, ...], list[dict[str, object]]] = {}
    for row in train_rows:
        grouped.setdefault(feature_key(row, feature_set), []).append(row)

    enabled: set[tuple[str, ...]] = set()
    summaries: list[dict[str, object]] = []
    for key, rows in sorted(grouped.items()):
        improvement = down_improvement(rows, down_gain)
        is_enabled = len(rows) >= min_samples and improvement >= min_down_improvement_m and down_gain != 0.0
        if is_enabled:
            enabled.add(key)
        summaries.append(
            {
                "feature_set": feature_set,
                "feature_key": ":".join(key),
                "sample_count": len(rows),
                "down_gain": down_gain,
                "train_down_rmse_improvement_m": f"{improvement:.9f}",
                "enabled": "yes" if is_enabled else "no",
            }
        )
    return enabled, summaries


def holdout_splits(
    samples: Sequence[dict[str, object]],
    *,
    holdout_mode: str,
) -> list[dict[str, object]]:
    if holdout_mode == "report":
        report_ids = sorted({str(row["report_dir"]) for row in samples})
        return [
            {
                "holdout_report": report_id,
                "holdout_trial": "",
                "train_rows": [row for row in samples if str(row["report_dir"]) != report_id],
                "holdout_rows": [row for row in samples if str(row["report_dir"]) == report_id],
            }
            for report_id in report_ids
        ]

    if holdout_mode == "report_trial":
        report_trials = sorted({(str(row["report_dir"]), int(row["trial"])) for row in samples})
        return [
            {
                "holdout_report": report_id,
                "holdout_trial": trial,
                "train_rows": [
                    row
                    for row in samples
                    if str(row["report_dir"]) != report_id and int(row["trial"]) != trial
                ],
                "holdout_rows": [
                    row
                    for row in samples
                    if str(row["report_dir"]) == report_id and int(row["trial"]) == trial
                ],
            }
            for report_id, trial in report_trials
        ]

    raise ValueError(f"unsupported holdout mode: {holdout_mode}")


def apply_policy(
    rows: Sequence[dict[str, object]],
    *,
    feature_set: str,
    enabled_down_keys: set[tuple[str, ...]],
    right_gain: float,
    right_min_range_m: float,
    forward_gain: float,
    forward_min_range_m: float,
    down_gain: float,
) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for row in rows:
        raw_forward = float(row["raw_forward_m"])
        corrected_right, right_fallback = axis_guarded_error(
            raw_error=float(row["raw_error_right"]),
            anchor_error=float(row["anchor_error_right"]),
            raw_forward_m=raw_forward,
            gain=right_gain,
            min_range_m=right_min_range_m,
        )
        corrected_forward, forward_fallback = axis_guarded_error(
            raw_error=float(row["raw_error_forward"]),
            anchor_error=float(row["anchor_error_forward"]),
            raw_forward_m=raw_forward,
            gain=forward_gain,
            min_range_m=forward_min_range_m,
        )
        key = feature_key(row, feature_set)
        if key in enabled_down_keys:
            corrected_down = blended_error(float(row["raw_error_down"]), float(row["anchor_error_down"]), down_gain)
            down_fallback = "none"
        elif down_gain == 0.0:
            corrected_down = float(row["raw_error_down"])
            down_fallback = "gain_noop"
        else:
            corrected_down = float(row["raw_error_down"])
            down_fallback = "cell_guarded_noop"

        corrected = dict(row)
        corrected.update(
            {
                "corrected_error_right": corrected_right,
                "corrected_error_down": corrected_down,
                "corrected_error_forward": corrected_forward,
                "raw_error_position": math.sqrt(sum(float(row[f"raw_error_{axis}"]) ** 2 for axis in RDF_AXES)),
                "corrected_error_position": math.sqrt(
                    corrected_right * corrected_right
                    + corrected_down * corrected_down
                    + corrected_forward * corrected_forward
                ),
                "right_fallback": right_fallback,
                "down_fallback": down_fallback,
                "forward_fallback": forward_fallback,
                "down_feature_key": ":".join(key),
            }
        )
        output.append(corrected)
    return output


def summarize_rows(rows: Sequence[dict[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for axis in RDF_AXES:
        raw = [float(row[f"raw_error_{axis}"]) for row in rows]
        corrected = [float(row[f"corrected_error_{axis}"]) for row in rows]
        result[f"{axis}_rmse_improvement_m"] = rmse(raw) - rmse(corrected)
    raw_position = [float(row["raw_error_position"]) for row in rows]
    corrected_position = [float(row["corrected_error_position"]) for row in rows]
    result["position_rmse_improvement_m"] = rmse(raw_position) - rmse(corrected_position)
    result["pass_rdf_nonregression"] = (
        "yes" if all(float(result[f"{axis}_rmse_improvement_m"]) >= 0.0 for axis in RDF_AXES) else "no"
    )
    return result


def per_trial_summary(
    rows: Sequence[dict[str, object]],
    *,
    candidate_id: str,
    holdout_mode: str,
    feature_set: str,
    down_gain: float,
    train_min_samples: int,
    train_min_down_improvement_m: float,
) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    keys = sorted({(str(row["report_dir"]), int(row["trial"])) for row in rows})
    for report_dir, trial in keys:
        part = [row for row in rows if str(row["report_dir"]) == report_dir and int(row["trial"]) == trial]
        summary = summarize_rows(part)
        output.append(
            {
                "candidate_id": candidate_id,
                "holdout_mode": holdout_mode,
                "feature_set": feature_set,
                "down_gain": down_gain,
                "train_min_samples": train_min_samples,
                "train_min_down_improvement_m": f"{train_min_down_improvement_m:.9f}",
                "report_dir": report_dir,
                "trial": trial,
                "sample_count": len(part),
                **{
                    f"{axis}_rmse_improvement_m": f"{float(summary[f'{axis}_rmse_improvement_m']):.9f}"
                    for axis in AXES
                },
                "pass_rdf_nonregression": summary["pass_rdf_nonregression"],
                "down_cell_guarded_noop_count": sum(row["down_fallback"] == "cell_guarded_noop" for row in part),
                "down_applied_count": sum(row["down_fallback"] == "none" for row in part),
            }
        )
    return output


def candidate_summary(per_trial_rows: Sequence[dict[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {
        "candidate_id": per_trial_rows[0]["candidate_id"],
        "holdout_mode": per_trial_rows[0]["holdout_mode"],
        "feature_set": per_trial_rows[0]["feature_set"],
        "down_gain": per_trial_rows[0]["down_gain"],
        "train_min_samples": per_trial_rows[0]["train_min_samples"],
        "train_min_down_improvement_m": per_trial_rows[0]["train_min_down_improvement_m"],
        "report_count": len({row["report_dir"] for row in per_trial_rows}),
        "trial_count": len(per_trial_rows),
        "pass_trial_count": sum(row["pass_rdf_nonregression"] == "yes" for row in per_trial_rows),
        "pass_all_nonregression": "yes"
        if all(row["pass_rdf_nonregression"] == "yes" for row in per_trial_rows)
        else "no",
        "down_cell_guarded_noop_count": sum(int(row["down_cell_guarded_noop_count"]) for row in per_trial_rows),
        "down_applied_count": sum(int(row["down_applied_count"]) for row in per_trial_rows),
    }
    for axis in AXES:
        values = [float(row[f"{axis}_rmse_improvement_m"]) for row in per_trial_rows]
        result[f"min_{axis}_rmse_improvement_m"] = f"{min(values):.9f}"
        result[f"mean_{axis}_rmse_improvement_m"] = f"{sum(values) / len(values):.9f}"
        result[f"max_{axis}_rmse_improvement_m"] = f"{max(values):.9f}"
    return result


def group_summary(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    groups = (
        ("all", lambda row: ("all",)),
        ("report", lambda row: (str(row["report_dir"]),)),
        ("trial", lambda row: (str(row["trial"]),)),
        ("range_bin", lambda row: (str(row["range_bin"]),)),
        ("down_feature_key", lambda row: (str(row["down_feature_key"]),)),
    )
    for group_name, key_func in groups:
        keys = sorted({key_func(row) for row in rows})
        for key in keys:
            part = [row for row in rows if key_func(row) == key]
            summary = summarize_rows(part)
            output.append(
                {
                    "group": group_name,
                    "key": ":".join(key),
                    "sample_count": len(part),
                    **{
                        f"{axis}_rmse_improvement_m": f"{float(summary[f'{axis}_rmse_improvement_m']):.9f}"
                        for axis in AXES
                    },
                    "pass_rdf_nonregression": summary["pass_rdf_nonregression"],
                    "down_applied_count": sum(row["down_fallback"] == "none" for row in part),
                }
            )
    return output


def candidate_id(
    feature_set: str,
    down_gain: float,
    train_min_samples: int,
    train_min_down_improvement_m: float,
) -> str:
    return (
        f"feature={feature_set};dg={down_gain:g};"
        f"min_samples={train_min_samples};min_train_down={train_min_down_improvement_m:g}"
    )


def sample_diagnostics(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for row in rows:
        output.append(
            {
                "report_dir": row["report_dir"],
                "trial": row["trial"],
                "row_index": row["row_index"],
                "raw_forward_m": f"{float(row['raw_forward_m']):.9f}",
                "range_bin": row["range_bin"],
                "anchor_cell": row["anchor_cell"],
                "down_feature_key": row["down_feature_key"],
                "fallback": "none",
                **{
                    f"raw_error_{axis}_m": f"{float(row[f'raw_error_{axis}']):.9f}"
                    for axis in RDF_AXES
                },
                **{
                    f"corrected_error_{axis}_m": f"{float(row[f'corrected_error_{axis}']):.9f}"
                    for axis in RDF_AXES
                },
                "raw_error_position_m": f"{float(row['raw_error_position']):.9f}",
                "corrected_error_position_m": f"{float(row['corrected_error_position']):.9f}",
                "right_fallback": row["right_fallback"],
                "down_fallback": row["down_fallback"],
                "forward_fallback": row["forward_fallback"],
            }
        )
    return output


def main() -> int:
    args = parse_args()
    trials = parse_int_list(args.validation_trials)
    range_edges = parse_float_list(args.range_edges_m)
    feature_sets = parse_str_list(args.feature_sets)
    down_gains = parse_float_list(args.down_gains)
    min_samples_values = parse_int_list(args.train_min_samples)
    min_improvements = parse_float_list(args.train_min_down_improvements_m)
    samples = load_samples(args.report_dirs, trials, range_edges)
    report_ids = sorted({str(row["report_dir"]) for row in samples})
    splits = holdout_splits(samples, holdout_mode=args.holdout_mode)

    all_candidate_rows: list[dict[str, object]] = []
    all_trial_rows: list[dict[str, object]] = []
    best_corrected_rows: list[dict[str, object]] = []
    best_policy_rows: list[dict[str, object]] = []

    for feature_set in feature_sets:
        for down_gain in down_gains:
            for min_samples in min_samples_values:
                for min_improvement in min_improvements:
                    cid = candidate_id(feature_set, down_gain, min_samples, min_improvement)
                    corrected_rows: list[dict[str, object]] = []
                    policy_rows: list[dict[str, object]] = []
                    for split in splits:
                        holdout_report = str(split["holdout_report"])
                        holdout_trial = split["holdout_trial"]
                        train_rows = split["train_rows"]
                        holdout_rows = split["holdout_rows"]
                        enabled, summaries = build_down_policy(
                            train_rows,
                            feature_set=feature_set,
                            down_gain=down_gain,
                            min_samples=min_samples,
                            min_down_improvement_m=min_improvement,
                        )
                        for summary in summaries:
                            summary.update(
                                {
                                    "candidate_id": cid,
                                    "holdout_mode": args.holdout_mode,
                                    "holdout_report": holdout_report,
                                    "holdout_trial": holdout_trial,
                                    "train_min_samples": min_samples,
                                    "train_min_down_improvement_m": f"{min_improvement:.9f}",
                                }
                            )
                        policy_rows.extend(summaries)
                        corrected_rows.extend(
                            apply_policy(
                                holdout_rows,
                                feature_set=feature_set,
                                enabled_down_keys=enabled,
                                right_gain=float(args.right_gain),
                                right_min_range_m=float(args.right_min_range_m),
                                forward_gain=float(args.forward_gain),
                                forward_min_range_m=float(args.forward_min_range_m),
                                down_gain=down_gain,
                            )
                        )

                    per_trial = per_trial_summary(
                        corrected_rows,
                        candidate_id=cid,
                        holdout_mode=args.holdout_mode,
                        feature_set=feature_set,
                        down_gain=down_gain,
                        train_min_samples=min_samples,
                        train_min_down_improvement_m=min_improvement,
                    )
                    all_trial_rows.extend(per_trial)
                    summary = candidate_summary(per_trial)
                    all_candidate_rows.append(summary)

                    current_best = all_candidate_rows[0] if len(all_candidate_rows) == 1 else None
                    if current_best is summary:
                        best_corrected_rows = corrected_rows
                        best_policy_rows = policy_rows

    all_candidate_rows.sort(
        key=lambda row: (
            row["pass_all_nonregression"] == "yes",
            float(row["min_down_rmse_improvement_m"]),
            float(row["mean_down_rmse_improvement_m"]),
            float(row["min_right_rmse_improvement_m"]),
            float(row["min_forward_rmse_improvement_m"]),
            float(row["mean_position_rmse_improvement_m"]),
        ),
        reverse=True,
    )

    if all_candidate_rows:
        best_id = all_candidate_rows[0]["candidate_id"]
        # Recompute best diagnostics after final sorting.
        for feature_set in feature_sets:
            for down_gain in down_gains:
                for min_samples in min_samples_values:
                    for min_improvement in min_improvements:
                        if candidate_id(feature_set, down_gain, min_samples, min_improvement) != best_id:
                            continue
                        best_corrected_rows = []
                        best_policy_rows = []
                        for split in splits:
                            holdout_report = str(split["holdout_report"])
                            holdout_trial = split["holdout_trial"]
                            train_rows = split["train_rows"]
                            holdout_rows = split["holdout_rows"]
                            enabled, summaries = build_down_policy(
                                train_rows,
                                feature_set=feature_set,
                                down_gain=down_gain,
                                min_samples=min_samples,
                                min_down_improvement_m=min_improvement,
                            )
                            for summary in summaries:
                                summary.update(
                                    {
                                        "candidate_id": best_id,
                                        "holdout_mode": args.holdout_mode,
                                        "holdout_report": holdout_report,
                                        "holdout_trial": holdout_trial,
                                        "train_min_samples": min_samples,
                                        "train_min_down_improvement_m": f"{min_improvement:.9f}",
                                    }
                                )
                            best_policy_rows.extend(summaries)
                            best_corrected_rows.extend(
                                apply_policy(
                                    holdout_rows,
                                    feature_set=feature_set,
                                    enabled_down_keys=enabled,
                                    right_gain=float(args.right_gain),
                                    right_min_range_m=float(args.right_min_range_m),
                                    forward_gain=float(args.forward_gain),
                                    forward_min_range_m=float(args.forward_min_range_m),
                                    down_gain=down_gain,
                                )
                            )

    output_dir = args.output_dir
    write_csv(output_dir / "cell_guard_candidate_summary.csv", all_candidate_rows)
    write_csv(output_dir / "cell_guard_trial_summary.csv", all_trial_rows)
    if best_corrected_rows:
        write_csv(output_dir / "candidate_sample_diagnostics.csv", sample_diagnostics(best_corrected_rows))
        write_csv(output_dir / "candidate_group_summary.csv", group_summary(best_corrected_rows))
        write_csv(output_dir / "enabled_policy_summary.csv", best_policy_rows)
    write_json(
        output_dir / "metadata.json",
        {
            "input_reports": [str(path) for path in args.report_dirs],
            "sample_count": len(samples),
            "report_count": len(report_ids),
            "holdout_mode": args.holdout_mode,
            "holdout_count": len(splits),
            "validation_contract": (
                "Down policy is trained from runtime-known anchor_cell/range_bin "
                "features. In report mode, held-out report rows are never used "
                "to choose enabled Down cells. In report_trial mode, held-out "
                "report rows and rows from the same validation trial number are "
                "both excluded from policy selection."
            ),
            "right_policy": {"gain": float(args.right_gain), "min_range_m": float(args.right_min_range_m)},
            "forward_policy": {"gain": float(args.forward_gain), "min_range_m": float(args.forward_min_range_m)},
        },
    )

    if all_candidate_rows:
        best = all_candidate_rows[0]
        print(
            "Best LORO: "
            f"{best['candidate_id']} pass_all={best['pass_all_nonregression']} "
            f"min_rdf=({best['min_right_rmse_improvement_m']},"
            f"{best['min_down_rmse_improvement_m']},"
            f"{best['min_forward_rmse_improvement_m']}) "
            f"down_applied={best['down_applied_count']}"
        )
        print(f"Candidate summary: {output_dir / 'cell_guard_candidate_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
