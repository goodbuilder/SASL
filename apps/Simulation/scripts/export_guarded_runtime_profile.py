#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Mapping


SUPPORTED_GUARD_MODES = {
    "target_disparity_range_delta_cap",
    "target_disparity_blend",
    "target_range_away_blend",
}


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def focal_px(width_px: float, horizontal_fov_rad: float) -> float:
    return (float(width_px) * 0.5) / math.tan(float(horizontal_fov_rad) * 0.5)


def target_disparity_px(
    *,
    width_px: float,
    horizontal_fov_rad: float,
    focal_scale_error: float,
    baseline_m: float,
    baseline_scale_error: float,
    target_range_m: float,
) -> float:
    fx_meas = focal_px(width_px, horizontal_fov_rad) * (1.0 + float(focal_scale_error))
    baseline_meas = float(baseline_m) * (1.0 + float(baseline_scale_error))
    return fx_meas * baseline_meas / float(target_range_m)


def guarded_profile(args: argparse.Namespace) -> dict[str, object]:
    data = read_json(args.input_profile)
    if data.get("mode") != "estimated_to_truth_sim3":
        raise ValueError("runtime guards in this exporter currently require estimated_to_truth_sim3 profiles")

    target_d = (
        float(args.target_disparity_px)
        if args.target_disparity_px is not None
        else target_disparity_px(
            width_px=float(args.width_px),
            horizontal_fov_rad=float(args.horizontal_fov_rad),
            focal_scale_error=float(args.focal_scale_error),
            baseline_m=float(args.baseline_m),
            baseline_scale_error=float(args.baseline_scale_error),
            target_range_m=float(args.target_range_m),
        )
    )

    profile_id = str(data.get("profile_id", "runtime_profile"))
    suffix = str(args.profile_id_suffix)
    data["profile_id"] = f"{profile_id}_{suffix}" if suffix else profile_id
    data["guard_export_contract"] = (
        "Runtime guard uses measured disparity/raw range/corrected range only; "
        "PX4/RTK/NED truth is not a runtime input."
    )
    data["guard_target_range_m"] = float(args.target_range_m)

    if args.guard_mode == "target_disparity_range_delta_cap":
        data["target_disparity_range_delta_cap_guard"] = {
            "target_disparity_px": target_d,
            "half_width_px": float(args.half_width_px),
            "range_delta_cap_m": float(args.range_delta_cap_m),
        }
    elif args.guard_mode == "target_disparity_blend":
        data["target_disparity_blend_guard"] = {
            "target_disparity_px": target_d,
            "half_width_px": float(args.half_width_px),
            "alpha_inside": float(args.alpha_inside),
            "alpha_outside": float(args.alpha_outside),
        }
    elif args.guard_mode == "target_range_away_blend":
        data["target_range_away_blend_guard"] = {
            "target_range_m": float(args.target_range_m),
            "window_m": float(args.range_window_m),
            "alpha_away": float(args.alpha_away),
        }
    else:
        raise ValueError(f"unsupported guard mode: {args.guard_mode}")

    return data


def write_summary(path: Path, payload: Mapping[str, object], args: argparse.Namespace) -> None:
    lines = [
        "Guarded runtime profile export",
        "",
        f"input_profile: {args.input_profile}",
        f"output_profile: {args.output_profile}",
        f"profile_id: {payload.get('profile_id', '')}",
        f"mode: {payload.get('mode', '')}",
        f"guard_mode: {args.guard_mode}",
        f"guard_target_range_m: {payload.get('guard_target_range_m', '')}",
        f"guard_export_contract: {payload.get('guard_export_contract', '')}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a runtime profile JSON with a runtime-safe target guard.")
    parser.add_argument("--input-profile", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--output-summary", type=Path)
    parser.add_argument("--guard-mode", choices=sorted(SUPPORTED_GUARD_MODES), default="target_disparity_range_delta_cap")
    parser.add_argument("--profile-id-suffix", default="")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-disparity-px", type=float)
    parser.add_argument("--half-width-px", type=float, default=0.35)
    parser.add_argument("--range-delta-cap-m", type=float, default=1.0)
    parser.add_argument("--alpha-inside", type=float, default=0.75)
    parser.add_argument("--alpha-outside", type=float, default=1.0)
    parser.add_argument("--range-window-m", type=float, default=70.0)
    parser.add_argument("--alpha-away", type=float, default=0.85)
    parser.add_argument("--width-px", type=float, default=1280.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--baseline-scale-error", type=float, default=0.006)
    parser.add_argument("--focal-scale-error", type=float, default=-0.004)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = guarded_profile(args)
    write_json(args.output_profile, payload)
    if args.output_summary is not None:
        write_summary(args.output_summary, payload, args)
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
