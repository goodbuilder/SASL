#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SELECTOR_PATH = (
    ROOT_DIR
    / "reports/experiments/EXP-20260611-003-runtime-feature-residual-selector/runtime_feature_selector_fast.py"
)
AXIS_CONTRACT = (
    "internal_policy_vector=RDF(right,down,forward); "
    "legacy_stereodistance_position_topics=FRD_range(forward,right,down,range); "
    "named CSV/report fields must preserve explicit axis names"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export EXP009 bounded-gain guard as a StereoDistance runtime JSON policy."
    )
    parser.add_argument("--reports-root", type=Path, default=ROOT_DIR / "reports")
    parser.add_argument(
        "--anchor-profile",
        type=Path,
        default=ROOT_DIR / "reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260625/bbox_anchor_profile.json",
    )
    parser.add_argument("--profile-family", default="")
    parser.add_argument("--output-path", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--feature-set", default="profile_family_range_raw_down_center_delta_down_bin")
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--min-train-improvement-m", type=float, default=0.05)
    parser.add_argument("--max-selected-gain", type=float, default=0.25)
    parser.add_argument("--forward-raw-anchor-cells", default="disparity_bin_3")
    return parser.parse_args()


def parse_ints(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_strings(text: str) -> list[str]:
    return [part.strip() for part in str(text).split(",") if part.strip()]


def load_selector_module():
    spec = importlib.util.spec_from_file_location("runtime_feature_selector_fast", SELECTOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load selector script: {SELECTOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if values.size else float("nan")


def infer_profile_family(path: Path, anchor_profile: dict[str, object]) -> str:
    text = str(path)
    for name in (
        "vertical_global",
        "global_screen_coverage",
        "global_fullscreen",
        "screen5x3",
        "screen3x3",
        "disparity_bin",
        "global",
    ):
        if name in text:
            return name
    mode = ""
    nested = anchor_profile.get("anchor_profile")
    if isinstance(nested, dict):
        mode = str(nested.get("mode", ""))
    else:
        mode = str(anchor_profile.get("mode", ""))
    return mode or "unknown"


def feature_key(row: dict[str, object], feature_set: str) -> str:
    if feature_set != "profile_family_range_raw_down_center_delta_down_bin":
        raise ValueError(f"unsupported EXP009 runtime feature set: {feature_set}")
    return (
        f"{row['profile_family']}|{row['range_bin']}|"
        f"{row['raw_down_bin']}|{row['center_delta_down_bin']}"
    )


def build_down_cells(
    data,
    rows: Sequence[dict[str, object]],
    *,
    feature_set: str,
    min_samples: int,
    min_train_improvement_m: float,
    max_selected_gain: float,
) -> dict[str, dict[str, object]]:
    groups: dict[str, list[int]] = {}
    for index, row in enumerate(rows):
        groups.setdefault(feature_key(row, feature_set), []).append(index)

    cells: dict[str, dict[str, object]] = {}
    down_axis_index = 1
    for key, indices_list in sorted(groups.items()):
        indices = np.array(indices_list, dtype=np.int32)
        if indices.size < min_samples:
            continue
        raw_error = data.raw_error[indices, down_axis_index]
        raw_rmse = rmse(raw_error)
        candidate_errors = data.candidate_error[indices, down_axis_index, :]
        candidate_rmse = np.sqrt(np.mean(np.square(candidate_errors), axis=0))
        improvements = raw_rmse - candidate_rmse
        for candidate_index, gain in enumerate(data.candidate_gains):
            if float(gain) > max_selected_gain:
                improvements[candidate_index] = -float("inf")
        best_index = int(np.argmax(improvements))
        best_improvement = float(improvements[best_index])
        if best_index == 0 or best_improvement < min_train_improvement_m:
            continue
        cells[key] = {
            "candidate": data.candidate_names[best_index],
            "sample_count": int(indices.size),
            "train_down_rmse_improvement_m": round(best_improvement, 9),
        }
    return cells


def main() -> int:
    args = parse_args()
    selector = load_selector_module()
    trials = parse_ints(args.validation_trials)
    gains = selector.parse_floats("0.025,0.05,0.075,0.1,0.15,0.2,0.25,0.35,0.5,0.75,1.0")
    rows = selector.load_rows(args.reports_root, trials)
    data = selector.Dataset(
        rows,
        gains,
        forward_raw_anchor_cells=set(parse_strings(args.forward_raw_anchor_cells)),
    )
    cells = build_down_cells(
        data,
        rows,
        feature_set=args.feature_set,
        min_samples=args.min_samples,
        min_train_improvement_m=args.min_train_improvement_m,
        max_selected_gain=args.max_selected_gain,
    )

    with args.anchor_profile.open(encoding="utf-8") as handle:
        anchor_profile_payload = json.load(handle)
    profile_family = args.profile_family.strip() or infer_profile_family(args.anchor_profile, anchor_profile_payload)
    anchor_profile = anchor_profile_payload.get("anchor_profile", anchor_profile_payload)
    rotation = anchor_profile_payload.get("right_actual_to_nominal_rotation")

    policy = {
        "mode": "bounded_gain_forward_cell_guard",
        "profile_id": "EXP-20260611-009-bounded-gain-forward-cell-guard",
        "method": "bounded_gain_forward_cell_guard",
        "source_experiment": "EXP-20260611-009-bounded-gain-forward-cell-guard",
        "profile_family": profile_family,
        "feature_set": args.feature_set,
        "axis_contract": AXIS_CONTRACT,
        "training_contract": (
            "Policy cells are trained from calibration data with ground truth; "
            "runtime apply uses only bbox, raw stereo, anchor profile, and configured profile_family."
        ),
        "range_edges_m": list(selector.DEFAULT_RANGE_EDGES_M),
        "raw_down_edges_m": list(selector.RAW_DOWN_EDGES_M),
        "center_delta_down_edges_m": list(selector.CENTER_DELTA_DOWN_EDGES_M),
        "right_policy": {
            "candidate": "anchor_gain_0.1",
            "min_range_m": 1100.0,
        },
        "forward_policy": {
            "candidate": "anchor_gain_1",
            "min_range_m": 500.0,
            "raw_fallback_anchor_cells": parse_strings(args.forward_raw_anchor_cells),
        },
        "down_policy": {
            "feature_set": args.feature_set,
            "min_samples": args.min_samples,
            "min_train_improvement_m": args.min_train_improvement_m,
            "max_selected_gain": args.max_selected_gain,
            "cells": cells,
        },
        "anchor_profile_source": str(args.anchor_profile),
        "anchor_profile": anchor_profile,
        "right_actual_to_nominal_rotation": rotation,
        "metadata": {
            "input_sample_count": len(rows),
            "down_cell_count": len(cells),
            "available_profile_families": sorted({str(row["profile_family"]) for row in rows}),
        },
    }
    args.output_path.parent.mkdir(parents=True, exist_ok=True)
    args.output_path.write_text(json.dumps(policy, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.output_path}")
    print(f"Down cells: {len(cells)}")
    print(f"Runtime profile family: {profile_family}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
