#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))
for parent in Path(__file__).resolve().parents:
    if (parent / "sasl_common").is_dir():
        if str(parent) not in sys.path:
            sys.path.insert(0, str(parent))
        break

from wiris_stereo_distance.reprojection_profile import (  # noqa: E402
    DEFAULT_REPROJECTION_CORRECTION_MODE,
    DEFAULT_REPROJECTION_DISPARITY_GAIN,
    DEFAULT_REPROJECTION_OUTPUT_POLICY,
    DEFAULT_REPROJECTION_SCREEN_U_GAIN,
    DEFAULT_REPROJECTION_SCREEN_V_GAIN,
)
from wiris_stereo_distance.simulation_origin import (
    aggregate_summary_files,
    build_origin_bbox_profile_from_telemetry,
    build_origin_bbox_reprojection_profile_from_telemetry,
    DEFAULT_ORIGIN_BBOX_MIN_CELL_SAMPLES,
    DEFAULT_ORIGIN_BBOX_AXIS_GAIN_CANDIDATES,
    DEFAULT_ORIGIN_BBOX_AXIS_MODEL_KINDS,
    DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_M,
    DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_RATIO,
    DEFAULT_ORIGIN_BBOX_MIN_CORRECTION_NORM_M,
    DEFAULT_ORIGIN_BBOX_MIN_RANGE_CORRECTION_M,
    DEFAULT_ORIGIN_BBOX_MIN_VALIDATION_CELL_SAMPLES,
    DEFAULT_ORIGIN_BBOX_RESIDUAL_GAIN,
    build_profile_from_telemetry,
    detect_profile_kind,
    evaluate_origin_bbox_profile_rows,
    evaluate_origin_bbox_reprojection_profile_rows,
    evaluate_profile_rows,
    generate_random_origin_waypoints,
    generate_origin_waypoints,
    load_origin_bbox_profile,
    load_origin_bbox_reprojection_profile,
    load_profile,
    load_telemetry_rows,
    read_env_file,
    save_profile,
    save_reprojection_profile,
    simulation_comparison_rows,
    write_dynamic_csv,
    write_json,
    write_waypoint_csv,
)


def _default_sim_dir() -> Path:
    if os.environ.get("SIM_DIR"):
        return Path(os.environ["SIM_DIR"]).expanduser()
    if os.environ.get("SASL_ROOT"):
        return Path(os.environ["SASL_ROOT"]).expanduser() / "Simulation"
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "Simulation"
        if candidate.is_dir():
            return candidate
    return ROOT_DIR / "Simulation"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulation Origin Profile path/profile/evaluation helper.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    path_parser = subparsers.add_parser("write-path", help="Write a PX4 waypoint CSV for Origin Profile capture.")
    path_parser.add_argument("--output", required=True, type=Path)
    path_parser.add_argument("--simulation-dir", type=Path, default=_default_sim_dir())
    path_parser.add_argument("--drone-env", type=Path)
    path_parser.add_argument("--camera-env", type=Path)
    path_parser.add_argument(
        "--spawn-front-m",
        type=float,
        help="Override DRONE_SPAWN_Y_M when generating range-relative waypoints.",
    )
    path_parser.add_argument("--target-width-m", type=float, default=15.0)
    path_parser.add_argument("--overlap-margin-m", type=float, default=3.0)
    path_parser.add_argument(
        "--ranges-m",
        default="1100,900,700,500,380,320",
        help="Comma-separated camera-forward ranges for profile waypoints.",
    )
    path_parser.add_argument("--randomize", action="store_true", help="Generate a random overlap-safe waypoint path.")
    path_parser.add_argument("--random-seed", type=int, default=0, help="Seed for --randomize.")
    path_parser.add_argument("--random-waypoint-count", type=int, default=6)
    path_parser.add_argument("--random-range-min-m", type=float)
    path_parser.add_argument("--random-range-max-m", type=float)
    path_parser.add_argument("--random-lateral-fraction-limit", type=float, default=0.85)
    path_parser.add_argument("--random-altitude-jitter-m", type=float, default=2.5)

    profile_parser = subparsers.add_parser("build-profile", help="Build an Origin Profile from telemetry.csv.")
    profile_parser.add_argument("--telemetry", required=True, type=Path)
    profile_parser.add_argument("--profile", required=True, type=Path)
    profile_parser.add_argument("--comparison-csv", required=True, type=Path)
    profile_parser.add_argument("--summary-json", required=True, type=Path)
    profile_parser.add_argument("--profile-id", default="")
    profile_parser.add_argument("--min-samples", type=int, default=30)
    profile_parser.add_argument(
        "--validation-telemetry",
        type=Path,
        help="Optional validation telemetry used to select per-cell/per-axis Origin-BBOX gains.",
    )
    profile_parser.add_argument(
        "--profile-kind",
        choices=("alignment", "origin-bbox", "origin-bbox-reprojection"),
        default="alignment",
        help=(
            "Profile model to build. alignment is global rigid transform; "
            "origin-bbox is legacy 3D residual grid; origin-bbox-reprojection is pixel residual grid."
        ),
    )
    profile_parser.add_argument("--reprojection-u-bins", type=int, default=6)
    profile_parser.add_argument("--reprojection-v-bins", type=int, default=4)
    profile_parser.add_argument("--reprojection-depth-bins", type=int, default=6)
    profile_parser.add_argument("--reprojection-bin-axis", choices=("disparity", "range"), default="disparity")
    profile_parser.add_argument("--reprojection-lookup-policy", choices=("exact", "nearest"), default="exact")
    profile_parser.add_argument(
        "--reprojection-correction-mode",
        choices=("full", "disparity_only", "component_gains"),
        default=DEFAULT_REPROJECTION_CORRECTION_MODE,
        help=(
            "Pixel residual component policy. disparity_only changes only left/right horizontal separation; "
            "full applies raw du/dv medians; component_gains uses the explicit gain options."
        ),
    )
    profile_parser.add_argument("--reprojection-disparity-gain", type=float, default=DEFAULT_REPROJECTION_DISPARITY_GAIN)
    profile_parser.add_argument("--reprojection-screen-u-gain", type=float, default=DEFAULT_REPROJECTION_SCREEN_U_GAIN)
    profile_parser.add_argument("--reprojection-screen-v-gain", type=float, default=DEFAULT_REPROJECTION_SCREEN_V_GAIN)
    profile_parser.add_argument(
        "--reprojection-output-policy",
        choices=("triangulated", "range_only_forward_fusion"),
        default=DEFAULT_REPROJECTION_OUTPUT_POLICY,
        help=(
            "Corrected output policy. range_only_forward_fusion preserves raw Right/Down and applies only "
            "the reprojection-derived Forward value."
        ),
    )
    profile_parser.add_argument("--image-width-px", type=float, default=1280.0)
    profile_parser.add_argument("--image-height-px", type=float, default=720.0)
    profile_parser.add_argument("--focal-length-px", type=float)
    profile_parser.add_argument("--baseline-m", type=float)
    profile_parser.add_argument("--min-cell-samples", type=int, default=DEFAULT_ORIGIN_BBOX_MIN_CELL_SAMPLES)
    profile_parser.add_argument(
        "--residual-gain",
        type=float,
        default=DEFAULT_ORIGIN_BBOX_RESIDUAL_GAIN,
        help="Scale applied to Origin-BBOX residuals.",
    )
    profile_parser.add_argument(
        "--min-correction-norm-m",
        type=float,
        default=DEFAULT_ORIGIN_BBOX_MIN_CORRECTION_NORM_M,
        help="Disable Origin-BBOX vector correction below this post-gain correction norm.",
    )
    profile_parser.add_argument(
        "--min-range-correction-m",
        type=float,
        default=DEFAULT_ORIGIN_BBOX_MIN_RANGE_CORRECTION_M,
        help="Disable Origin-BBOX scalar range correction below this post-gain magnitude.",
    )
    profile_parser.add_argument(
        "--axis-gain-candidates",
        default=",".join(str(value) for value in DEFAULT_ORIGIN_BBOX_AXIS_GAIN_CANDIDATES),
        help="Comma-separated gain candidates for validation-gated axis correction.",
    )
    profile_parser.add_argument(
        "--axis-model-kinds",
        default=",".join(DEFAULT_ORIGIN_BBOX_AXIS_MODEL_KINDS),
        help="Comma-separated axis model kinds, or 'none' for gain-only Origin-BBOX validation.",
    )
    profile_parser.add_argument(
        "--disable-axis",
        action="append",
        choices=("right", "down", "forward"),
        default=[],
        help="Disable Origin-BBOX correction for an axis. Repeatable.",
    )
    profile_parser.add_argument(
        "--min-validation-cell-samples",
        type=int,
        default=DEFAULT_ORIGIN_BBOX_MIN_VALIDATION_CELL_SAMPLES,
    )
    profile_parser.add_argument(
        "--min-axis-improvement-m",
        type=float,
        default=DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_M,
    )
    profile_parser.add_argument(
        "--min-axis-improvement-ratio",
        type=float,
        default=DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_RATIO,
    )

    eval_parser = subparsers.add_parser("evaluate", help="Evaluate telemetry.csv with an Origin Profile.")
    eval_parser.add_argument("--telemetry", required=True, type=Path)
    eval_parser.add_argument("--profile", required=True, type=Path)
    eval_parser.add_argument("--comparison-csv", required=True, type=Path)
    eval_parser.add_argument("--summary-json", required=True, type=Path)
    eval_parser.add_argument("--label", default="")
    eval_parser.add_argument(
        "--profile-kind",
        choices=("auto", "alignment", "origin-bbox", "origin-bbox-reprojection"),
        default="auto",
        help="Profile model to evaluate. auto reads the profile method field.",
    )
    eval_parser.add_argument("--image-width-px", type=float, default=1280.0)
    eval_parser.add_argument("--image-height-px", type=float, default=720.0)
    eval_parser.add_argument("--focal-length-px", type=float)
    eval_parser.add_argument("--baseline-m", type=float)

    aggregate_parser = subparsers.add_parser("aggregate", help="Aggregate repeated evaluation summaries.")
    aggregate_parser.add_argument("--summary-json", required=True, type=Path)
    aggregate_parser.add_argument("inputs", nargs="+", type=Path)
    return parser


def _parse_ranges(text: str) -> list[float]:
    return [float(part.strip()) for part in text.split(",") if part.strip()]


def _parse_axis_model_kinds(text: str) -> list[str]:
    stripped = text.strip()
    if not stripped or stripped.lower() == "none":
        return []
    allowed = set(DEFAULT_ORIGIN_BBOX_AXIS_MODEL_KINDS)
    kinds = [part.strip() for part in stripped.split(",") if part.strip()]
    unknown = sorted(kind for kind in kinds if kind not in allowed)
    if unknown:
        raise ValueError(f"unknown axis model kind(s): {', '.join(unknown)}")
    return kinds


def main() -> int:
    args = _build_parser().parse_args()
    if args.command == "write-path":
        sim_dir = args.simulation_dir.resolve()
        drone_env = read_env_file(args.drone_env or sim_dir / "config" / "drone_mission.env")
        camera_env = read_env_file(args.camera_env or sim_dir / "config" / "camera_model.env")
        if args.spawn_front_m is not None:
            drone_env["DRONE_SPAWN_Y_M"] = str(float(args.spawn_front_m))
        ranges_m = _parse_ranges(args.ranges_m)
        if args.randomize:
            waypoints = generate_random_origin_waypoints(
                drone_env=drone_env,
                camera_env=camera_env,
                seed=args.random_seed,
                target_width_m=args.target_width_m,
                overlap_margin_m=args.overlap_margin_m,
                range_min_m=args.random_range_min_m if args.random_range_min_m is not None else min(ranges_m),
                range_max_m=args.random_range_max_m if args.random_range_max_m is not None else max(ranges_m),
                waypoint_count=args.random_waypoint_count,
                lateral_fraction_limit=args.random_lateral_fraction_limit,
                altitude_jitter_m=args.random_altitude_jitter_m,
            )
        else:
            waypoints = generate_origin_waypoints(
                drone_env=drone_env,
                camera_env=camera_env,
                target_width_m=args.target_width_m,
                overlap_margin_m=args.overlap_margin_m,
                ranges_m=ranges_m,
            )
        write_waypoint_csv(args.output, waypoints)
        print(
            json.dumps(
                {
                    "waypoint_count": len(waypoints),
                    "output": str(args.output),
                    "randomized": bool(args.randomize),
                    "random_seed": int(args.random_seed) if args.randomize else None,
                },
                indent=2,
            )
        )
        return 0

    if args.command == "build-profile":
        profile_id = args.profile_id or f"simulation_origin_{time.strftime('%Y%m%d_%H%M%S')}"
        if args.profile_kind == "origin-bbox-reprojection":
            profile, rows = build_origin_bbox_reprojection_profile_from_telemetry(
                args.telemetry,
                profile_id=profile_id,
                min_samples=args.min_samples,
                image_width_px=args.image_width_px,
                image_height_px=args.image_height_px,
                focal_length_px=args.focal_length_px,
                baseline_m=args.baseline_m,
                u_bins=args.reprojection_u_bins,
                v_bins=args.reprojection_v_bins,
                depth_bins=args.reprojection_depth_bins,
                min_cell_samples=args.min_cell_samples,
                bin_axis=args.reprojection_bin_axis,
                lookup_policy=args.reprojection_lookup_policy,
                correction_mode=args.reprojection_correction_mode,
                disparity_gain=args.reprojection_disparity_gain,
                screen_u_gain=args.reprojection_screen_u_gain,
                screen_v_gain=args.reprojection_screen_v_gain,
                output_policy=args.reprojection_output_policy,
            )
            evaluated, summary = evaluate_origin_bbox_reprojection_profile_rows(
                rows,
                profile,
                image_width_px=args.image_width_px,
                image_height_px=args.image_height_px,
                focal_length_px=args.focal_length_px,
                baseline_m=args.baseline_m,
            )
            profile_extra = {
                "profile_cell_count": len(profile.cells),
                "profile_method": profile.method,
                "profile_bin_axis": profile.bin_axis,
                "profile_lookup_policy": profile.lookup_policy,
                "profile_correction_mode": profile.correction_mode,
                "profile_disparity_gain": profile.disparity_gain,
                "profile_screen_u_gain": profile.screen_u_gain,
                "profile_screen_v_gain": profile.screen_v_gain,
            }
        elif args.profile_kind == "origin-bbox":
            profile, rows = build_origin_bbox_profile_from_telemetry(
                args.telemetry,
                profile_id=profile_id,
                min_samples=args.min_samples,
                validation_telemetry_csv=args.validation_telemetry,
                min_cell_samples=args.min_cell_samples,
                residual_gain=args.residual_gain,
                min_correction_norm_m=args.min_correction_norm_m,
                min_range_correction_m=args.min_range_correction_m,
                axis_gain_candidates=_parse_ranges(args.axis_gain_candidates),
                axis_model_kinds=_parse_axis_model_kinds(args.axis_model_kinds),
                disabled_axes=args.disable_axis,
                min_validation_cell_samples=args.min_validation_cell_samples,
                min_axis_improvement_m=args.min_axis_improvement_m,
                min_axis_improvement_ratio=args.min_axis_improvement_ratio,
            )
            evaluated, summary = evaluate_origin_bbox_profile_rows(rows, profile)
            profile_extra = {
                "profile_cell_count": len(profile.cells),
                "profile_method": profile.method,
            }
        else:
            profile, rows = build_profile_from_telemetry(
                args.telemetry,
                profile_id=profile_id,
                min_samples=args.min_samples,
            )
            evaluated, summary = evaluate_profile_rows(rows, profile)
            profile_extra = {
                "profile_rmse_m": profile.rmse_m,
                "profile_max_error_m": profile.max_error_m,
                "profile_method": profile.method,
            }
        summary.update(
            {
                "label": "origin-profile-build",
                "telemetry": str(args.telemetry),
                "profile": str(args.profile),
                "comparison_csv": str(args.comparison_csv),
            }
        )
        summary.update(profile_extra)
        if args.profile_kind == "origin-bbox-reprojection":
            save_reprojection_profile(args.profile, profile)
        else:
            write_json(args.profile, profile.to_dict())
        write_dynamic_csv(args.comparison_csv, evaluated)
        write_json(args.summary_json, summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    if args.command == "evaluate":
        rows = simulation_comparison_rows(load_telemetry_rows(args.telemetry))
        profile_kind = detect_profile_kind(args.profile) if args.profile_kind == "auto" else args.profile_kind
        if profile_kind == "origin-bbox-reprojection":
            profile = load_origin_bbox_reprojection_profile(args.profile)
            evaluated, summary = evaluate_origin_bbox_reprojection_profile_rows(
                rows,
                profile,
                image_width_px=args.image_width_px,
                image_height_px=args.image_height_px,
                focal_length_px=args.focal_length_px,
                baseline_m=args.baseline_m,
            )
        elif profile_kind == "origin-bbox":
            profile = load_origin_bbox_profile(args.profile)
            evaluated, summary = evaluate_origin_bbox_profile_rows(rows, profile)
        else:
            profile = load_profile(args.profile)
            evaluated, summary = evaluate_profile_rows(rows, profile)
        summary.update(
            {
                "label": args.label,
                "telemetry": str(args.telemetry),
                "profile": str(args.profile),
                "comparison_csv": str(args.comparison_csv),
            }
        )
        write_dynamic_csv(args.comparison_csv, evaluated)
        write_json(args.summary_json, summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    if args.command == "aggregate":
        summary = aggregate_summary_files(args.inputs)
        write_json(args.summary_json, summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
