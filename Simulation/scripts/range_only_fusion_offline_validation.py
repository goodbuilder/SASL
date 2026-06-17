#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Sequence


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a conservative range-only fusion candidate. Right/Down stay at "
            "the raw stereo estimate, while Forward is replaced or blended with a "
            "bbox-origin/extrinsic corrected range source from anchor comparison CSVs."
        )
    )
    parser.add_argument("anchor_result_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument(
        "--range-source",
        choices=("anchor", "center_rot", "corrected", "origin_bbox_reprojection"),
        default="anchor",
    )
    parser.add_argument("--range-blend", type=float, default=1.0)
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def load_profile_id(anchor_result_dir: Path) -> str:
    profile_path = anchor_result_dir / "bbox_anchor_profile.json"
    if not profile_path.exists():
        return anchor_result_dir.name
    payload = json.loads(profile_path.read_text(encoding="utf-8"))
    method = payload.get("method", "bbox_anchor")
    mode = payload.get("anchor_mode")
    force_u = payload.get("force_center_u")
    suffix = f"_{mode}" if mode else ""
    if force_u:
        suffix += "_force_center_u"
    return f"{method}{suffix}"


def evaluate_trial(
    path: Path,
    *,
    range_source: str,
    range_blend: float,
) -> tuple[list[dict[str, str]], dict[str, float | int | str]]:
    rows: list[dict[str, str]] = []
    errors = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "fused_right": [],
        "fused_down": [],
        "fused_forward": [],
    }
    blend = max(0.0, min(1.0, float(range_blend)))
    source_forward_key = {
        "origin_bbox_reprojection": "origin_bbox_reprojection_corrected_forward_m",
    }.get(range_source, f"{range_source}_forward_m")
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth = (
                finite_float(row.get("truth_right_m")),
                finite_float(row.get("truth_down_m")),
                finite_float(row.get("truth_forward_m")),
            )
            raw = (
                finite_float(row.get("raw_right_m")),
                finite_float(row.get("raw_down_m")),
                finite_float(row.get("raw_forward_m")),
            )
            source_forward = finite_float(row.get(source_forward_key))
            if None in truth or None in raw or source_forward is None:
                continue
            truth_point = (float(truth[0]), float(truth[1]), float(truth[2]))
            raw_point = (float(raw[0]), float(raw[1]), float(raw[2]))
            fused_forward = (1.0 - blend) * raw_point[2] + blend * float(source_forward)
            fused_point = (raw_point[0], raw_point[1], fused_forward)
            for prefix, point in (("raw", raw_point), ("fused", fused_point)):
                for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
                    errors[f"{prefix}_{axis}"].append(point[index] - truth_point[index])
            rows.append(
                {
                    "trial_t_s": row.get("trial_t_s", ""),
                    "truth_right_m": f"{truth_point[0]:.9f}",
                    "truth_down_m": f"{truth_point[1]:.9f}",
                    "truth_forward_m": f"{truth_point[2]:.9f}",
                    "raw_right_m": f"{raw_point[0]:.9f}",
                    "raw_down_m": f"{raw_point[1]:.9f}",
                    "raw_forward_m": f"{raw_point[2]:.9f}",
                    f"{range_source}_forward_m": f"{float(source_forward):.9f}",
                    "fused_right_m": f"{fused_point[0]:.9f}",
                    "fused_down_m": f"{fused_point[1]:.9f}",
                    "fused_forward_m": f"{fused_point[2]:.9f}",
                    "raw_error_right_m": f"{errors['raw_right'][-1]:.9f}",
                    "raw_error_down_m": f"{errors['raw_down'][-1]:.9f}",
                    "raw_error_forward_m": f"{errors['raw_forward'][-1]:.9f}",
                    "fused_error_right_m": f"{errors['fused_right'][-1]:.9f}",
                    "fused_error_down_m": f"{errors['fused_down'][-1]:.9f}",
                    "fused_error_forward_m": f"{errors['fused_forward'][-1]:.9f}",
                }
            )

    summary: dict[str, float | int | str] = {
        "sample_count": len(rows),
        "evaluated_count": len(rows),
        "range_source": range_source,
        "range_blend": blend,
    }
    pass_nonregression = True
    pass_strict_improvement = True
    for axis in ("right", "down", "forward"):
        raw_rmse = rmse(errors[f"raw_{axis}"])
        fused_rmse = rmse(errors[f"fused_{axis}"])
        improvement = raw_rmse - fused_rmse
        summary[f"raw_{axis}_rmse_m"] = raw_rmse
        summary[f"fused_{axis}_rmse_m"] = fused_rmse
        summary[f"{axis}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= -1.0e-9):
            pass_nonregression = False
        if not (math.isfinite(improvement) and improvement > 1.0e-9):
            pass_strict_improvement = False

    raw_position_errors = [
        math.sqrt(right * right + down * down + forward * forward)
        for right, down, forward in zip(errors["raw_right"], errors["raw_down"], errors["raw_forward"])
    ]
    fused_position_errors = [
        math.sqrt(right * right + down * down + forward * forward)
        for right, down, forward in zip(errors["fused_right"], errors["fused_down"], errors["fused_forward"])
    ]
    summary["raw_position_rmse_m"] = rmse(raw_position_errors)
    summary["fused_position_rmse_m"] = rmse(fused_position_errors)
    summary["position_rmse_improvement_m"] = summary["raw_position_rmse_m"] - summary["fused_position_rmse_m"]
    summary["pass_nonregression"] = "yes" if pass_nonregression else "no"
    summary["pass_strict_improvement"] = "yes" if pass_strict_improvement else "no"
    return rows, summary


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


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = args.anchor_result_dir.parent / f"{args.anchor_result_dir.name}_range_only_{args.range_source}"
    output_dir.mkdir(parents=True, exist_ok=True)

    profile_id = load_profile_id(args.anchor_result_dir)
    gate_rows: list[dict[str, object]] = []
    final_nonregression = True
    final_strict = True
    for trial in validation_trials(args.validation_trials):
        input_path = args.anchor_result_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
        if not input_path.exists():
            input_path = args.anchor_result_dir / f"validation_trial_{trial:02d}_comparison.csv"
        rows, summary = evaluate_trial(
            input_path,
            range_source=args.range_source,
            range_blend=args.range_blend,
        )
        if summary["pass_nonregression"] != "yes":
            final_nonregression = False
        if summary["pass_strict_improvement"] != "yes":
            final_strict = False
        write_csv(output_dir / f"validation_trial_{trial:02d}_range_only_comparison.csv", rows)
        write_json(output_dir / f"validation_trial_{trial:02d}_summary.json", summary)
        gate_rows.append(
            {
                "trial": trial,
                "sample_count": summary["sample_count"],
                "evaluated_count": summary["evaluated_count"],
                "right_rmse_improvement_m": f"{float(summary['right_rmse_improvement_m']):.9f}",
                "down_rmse_improvement_m": f"{float(summary['down_rmse_improvement_m']):.9f}",
                "forward_rmse_improvement_m": f"{float(summary['forward_rmse_improvement_m']):.9f}",
                "position_rmse_improvement_m": f"{float(summary['position_rmse_improvement_m']):.9f}",
                "pass_nonregression": summary["pass_nonregression"],
                "pass_strict_improvement": summary["pass_strict_improvement"],
            }
        )

    write_csv(output_dir / "validation_gate_summary.csv", gate_rows)
    write_json(
        output_dir / "range_only_fusion_profile.json",
        {
            "method": "range_only_forward_fusion_v1",
            "profile_id": profile_id,
            "anchor_result_dir": str(args.anchor_result_dir),
            "range_source": args.range_source,
            "range_blend": max(0.0, min(1.0, float(args.range_blend))),
            "contract": "Right/Down remain raw stereo axes; Forward is replaced/blended from the corrected range source.",
            "final_pass_nonregression": "yes" if final_nonregression else "no",
            "final_pass_strict_improvement": "yes" if final_strict else "no",
        },
    )
    (output_dir / "result.txt").write_text(
        "\n".join(
            [
                f"final_pass_nonregression={'yes' if final_nonregression else 'no'}",
                f"final_pass_strict_improvement={'yes' if final_strict else 'no'}",
                f"summary={output_dir / 'validation_gate_summary.csv'}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Summary: {output_dir / 'validation_gate_summary.csv'}")
    print(f"Final pass nonregression: {'yes' if final_nonregression else 'no'}")
    print(f"Final pass strict improvement: {'yes' if final_strict else 'no'}")
    return 0 if final_nonregression else 1


if __name__ == "__main__":
    raise SystemExit(main())
